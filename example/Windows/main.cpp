#include <atomic>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <string>
#include <thread>

// Specify drfl header.
#include "../../include/DRFLEx.h"
#include "util.hpp"

const std::string IP_ADDRESS = "127.0.0.1";

using namespace DRAFramework;
CDRFLEx robot;

static std::atomic<bool> g_run{ true };
static std::atomic<bool> g_got_access{ false };
static std::atomic<bool> g_is_standby{ false };
static std::atomic<bool> g_poll_run{ false };

MONITORING_ACCESS_CONTROL control_access = MONITORING_ACCESS_CONTROL_LAST;

static void log_msg(const char* tag, const std::string& msg)
{
    std::cout << "[" << tag << "] " << msg << std::endl;
}

void OnMonitoringStateCB(const ROBOT_STATE eState) {

    static std::atomic<int> last_state{ -1 };
    const int next = static_cast<int>(eState);
    if (last_state.exchange(next) != next) {
        log_msg("STATE", std::string("state: ") + to_str(eState));
    }
    g_is_standby = (eState == STATE_STANDBY);
    return;
}

void OnMonitroingAccessControlCB(
    const MONITORING_ACCESS_CONTROL eTrasnsitControl) {

    static std::atomic<int> last_access{ -1 };
    const int next = static_cast<int>(eTrasnsitControl);
    if (last_access.exchange(next) != next) {
        log_msg("ACCESS", std::string("status: ") + to_str(eTrasnsitControl));
    }
    control_access = eTrasnsitControl;
    g_got_access = (eTrasnsitControl == MONITORING_ACCESS_CONTROL_GRANT);
    return;
}

void OnLogAlarm(LPLOG_ALARM tLog) {
    std::cout << "[ALARM] "
        << "group(" << (unsigned int)tLog->_iGroup << "), index("
        << tLog->_iIndex << "), param(" << tLog->_szParam[0] << "), param("
        << tLog->_szParam[1] << "), param(" << tLog->_szParam[2] << ")" << std::endl;
}

static const char* to_str(SINGULARITY_FORCE_HANDLING m)
{
    switch (m) {
    case SINGULARITY_ERROR:  return "SINGULARITY_ERROR (error & stop)";
    case SINGULARITY_IGNORE: return "SINGULARITY_IGNORE (ignore error)";
    default:                 return "UNKNOWN";
    }
}

static void SigHandler(int) { g_run = false; }

static void run_force_singularity_test(CDRFLEx& robot, SINGULARITY_FORCE_HANDLING mode)
{
    std::cout << "\n=== run_force_singularity_test (" << to_str(mode) << ") ===\n";

    // Singularity force exception
    bool ok_bool = robot.set_singular_handling_force(mode);
    std::cout << "set_singular_handling_force -> " << (ok_bool ? "OK" : "FAIL") << std::endl;
    if (!ok_bool) return;

    // Executing force control by TOOL coordinate force
    int ret = robot.set_ref_coord(COORDINATE_SYSTEM_TOOL);
    std::cout << "set_ref_coord(COORDINATE_SYSTEM_TOOL) -> "
        << (ret == 1 ? "OK" : "FAIL") << std::endl;

    // Singularity Pose
    float q_sing[6] = { 0.0f, -40.0f, 80.0f, 0.0f, 40.0f, 0.0f };
    std::cout << "[INFO] movej to near-singular pose (slow)...\n";
    robot.movej(q_sing, 30.0f, 30.0f);
    std::this_thread::sleep_for(std::chrono::seconds(1));

    // Execute Compliance
    float stx[6] = { 300.f, 300.f, 300.f, 20.f, 20.f, 20.f };
    ret = robot.task_compliance_ctrl(stx);
    std::cout << "task_compliance_ctrl -> " << (ret == 1 ? "OK" : "FAIL") << std::endl;
    if (ret != 1) return;

    // Force/Axis 
    float fd[6] = { 0.f, 0.f, -5.0f, 0.f, 0.f, 0.f };
    unsigned char fdir[6] = { 0, 0, 1, 0, 0, 0 }; // Only Z axis

    ROBOT_STATE cur_state = robot.GetRobotState();
    std::cout << "[DEBUG] state before set_desired_force: "
        << to_str(cur_state) << std::endl;

    ret = robot.set_desired_force(fd, fdir);
    std::cout << "set_desired_force -> " << (ret == 1 ? "OK" : "FAIL") << std::endl;

    cur_state = robot.GetRobotState();
    std::cout << "[DEBUG] state after set_desired_force:  "
        << to_str(cur_state) << std::endl;

    if (ret != 1) {
        robot.release_compliance_ctrl();
        return;
    }

    for (int i = 0; i < 50 && g_run; ++i) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    // Release Compliance
    ret = robot.release_compliance_ctrl();
    std::cout << "release_compliance_ctrl -> " << (ret == 1 ? "OK" : "FAIL") << std::endl;

    std::cout << "=== End of Force Test (" << to_str(mode) << ") ===\n";
}

static void OnAccessCB(MONITORING_ACCESS_CONTROL ac)
{
    std::cout << "[Access] " << to_str(ac) << std::endl;
    g_got_access = (ac == MONITORING_ACCESS_CONTROL_GRANT);
}

static void print6(const char* tag, const float v[6])
{
    std::cout << tag << ": "
        << std::fixed << std::setprecision(6)
        << v[0] << " " << v[1] << " " << v[2] << " "
        << v[3] << " " << v[4] << " " << v[5] << std::endl;
}

static void print_link(const ROBOT_LINK_INFO& x, const char* title)
{
    std::cout << title << std::endl;
    print6("a[m]      ", x.a);
    print6("d[m]      ", x.d);
    print6("alpha[rad]", x.alpha);
    print6("theta[rad]", x.theta);
    print6("offset[rad]", x.offset);
    std::cout << "gradient: " << std::fixed << std::setprecision(6) << x.gradient
        << ", rotation: " << x.rotation << std::endl << std::endl;
}

static bool neq6(const float a[6], const float b[6], float eps = 1e-7f)
{
    for (int i = 0; i < 6; ++i) if (std::fabs(a[i] - b[i]) > eps) return true;
    return false;
}

// 200ms polling thread
static void dh_poll_thread(CDRFLEx* pRobot) {
    ROBOT_LINK_INFO prev{};
    bool has_prev = false;

    while (g_poll_run.load()) {
        ROBOT_LINK_INFO cur{};
        if (pRobot->get_robot_link_info(cur, 300)) {
            if (!has_prev ||
                neq6(prev.a, cur.a) || neq6(prev.d, cur.d) ||
                neq6(prev.alpha, cur.alpha) || neq6(prev.theta, cur.theta) ||
                neq6(prev.offset, cur.offset) ||
                std::fabs(prev.gradient - cur.gradient) > 1e-7f ||
                std::fabs(prev.rotation - cur.rotation) > 1e-7f) {

                print_link(cur, "[DH update]");
                prev = cur;
                has_prev = true;
            }
        }
        else {
            std::cout << "[WARN] get_robot_link_info timeout/fail" << std::endl;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }
}

static void print_menu()
{
    std::cout << "[MENU] 0:init 1:joint move 2:dh query 3:force err "
        "4:force ignore 5:emg stop 6:tool add/del 7:h2r test 8:reg test q:quit"
        << std::endl;
}


int main() {
    bool ret;
    std::cout << "FAILFAILFAILFAIL" << std::endl;
    ret = robot.open_connection(IP_ADDRESS);
    if (true != ret) {
        log_msg("BOOT", std::string("connect: FAIL (") + IP_ADDRESS + ")");
        return 1;
    }
    log_msg("BOOT", std::string("connect: OK (") + IP_ADDRESS + ")");

    if (true != robot.setup_monitoring_version(1)) {
        log_msg("BOOT", "monitoring version: FAIL (v1)");
        return 1;
    }
    log_msg("BOOT", "monitoring version: OK (v1)");

    robot.set_on_monitoring_state(OnMonitoringStateCB);
    robot.set_on_log_alarm(OnLogAlarm);

    // Manage Access Control seems to mean accessing monitoring data in controller.
    robot.set_on_monitoring_access_control(OnMonitroingAccessControlCB);
    if (true != robot.ManageAccessControl(MANAGE_ACCESS_CONTROL_FORCE_REQUEST)) {
        log_msg("BOOT", "access: REQUEST FAIL");
        return 1;
    }

    while (control_access != MONITORING_ACCESS_CONTROL_GRANT) {
        log_msg("BOOT", "access: waiting for grant...");
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }
    log_msg("BOOT", "access: GRANTED");

    for (size_t retry = 0; retry < 10; ++retry) {
        if (!g_got_access.load()) {
            robot.ManageAccessControl(MANAGE_ACCESS_CONTROL_FORCE_REQUEST);
        }
        if (!g_is_standby.load()) {
            robot.SetRobotControl(CONTROL_SERVO_ON);
        }
        if (g_got_access.load() && g_is_standby.load()) break;
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }
    if (!(g_got_access.load() && g_is_standby.load())) {
        log_msg("BOOT", "access/standby: FAIL");
        return 1;
    }

    ROBOT_STATE robot_state = robot.GetRobotState();
    log_msg("BOOT", std::string("robot state: ") + to_str(robot_state));
    robot.set_robot_mode(ROBOT_MODE_AUTONOMOUS);
    //robot.set_robot_mode(ROBOT_MODE_MANUAL);

    if (robot_state == STATE_SAFE_OFF) {
        if (true != robot.SetRobotControl(CONTROL_SERVO_ON)) {
            log_msg("BOOT", "servo on: FAIL");
            return 1;
        }
        log_msg("BOOT", "servo on: OK");
    }

    bool running = true;
    while (running) {
        log_msg("READY", "input mode (enter a menu key)");
        print_menu();
        char input;
        std::cin >> input;
        switch (input) {
        case '0': // Initialize robot: set REAL system, autonomous mode, and servo ON
        {
            //robot.set_robot_system(ROBOT_SYSTEM_VIRTUAL);
            robot.set_robot_system(ROBOT_SYSTEM_REAL);
            robot.set_robot_mode(ROBOT_MODE_AUTONOMOUS);
            robot.SetRobotControl(CONTROL_SERVO_ON);
            break;
        }
        case '1': // Simple joint motion test: move to two joint positions sequentially
        {
            float pos[6] = { 0., 0., 0., 0., 0.,0. };
            float fTargetVel = 30.0;
            float fTargetAcc = 30.0;
            robot.movej(pos, fTargetVel, fTargetAcc);

            float pos2[6] = { 0., 0., 90., 0., 90.,0. };
            float fTargetVel2 = 30.0;
            float fTargetAcc2 = 30.0;
            robot.movej(pos2, fTargetVel2, fTargetAcc2);

            break;
        }
        case '2': // Query DH parameters before and after motion (with periodic polling)
        {
            ROBOT_LINK_INFO link{};
            if (robot.get_robot_link_info(link, 500))
                print_link(link, "=== DH at START ===");
            else
                std::fprintf(stderr, "[ERR] initial get_robot_link_info failed\n");

            g_poll_run = true;
            std::thread th(dh_poll_thread, &robot);

            // Query once before and after motion execution
            float q[6] = { 0,0,30,0,0,0 };

            // Move #1
            if (robot.get_robot_link_info(link, 500))
                print_link(link, "[Before move #1]");
            robot.movej(q, 50, 50);
            if (robot.get_robot_link_info(link, 500))
                print_link(link, "[After  move #1]");

            // Move #2
            q[2] = 0;
            if (robot.get_robot_link_info(link, 500))
                print_link(link, "[Before move #2]");
            robot.movej(q, 50, 50);
            if (robot.get_robot_link_info(link, 500))
                print_link(link, "[After  move #2]");

            // Polling Stop
            g_poll_run = false;
            if (th.joinable()) th.join();

            break;
        }
        case '3': // Force control test near singularity (ERROR mode)
        {
            bool ok = robot.set_singular_handling_force(SINGULARITY_ERROR);
            std::cout << "set_singular_handling_force(ERROR) -> " << (ok ? "OK" : "FAIL") << "\n";
            run_force_singularity_test(robot, SINGULARITY_ERROR);
            break;
        }
        case '4': // Force control test near singularity (IGNORE mode)
        {
            bool ok = robot.set_singular_handling_force(SINGULARITY_IGNORE);
            std::cout << "set_singular_handling_force(IGNORE) -> " << (ok ? "OK" : "FAIL") << "\n";
            run_force_singularity_test(robot, SINGULARITY_IGNORE);
            break;
        }
        case '5': // Emergency stop and recovery sequence
        {
            robot.MoveStop(STOP_TYPE_QUICK_STO);
            robot.SetRobotControl(CONTROL_RECOVERY_SAFE_OFF);
            robot.SetRobotControl(CONTROL_SERVO_ON);
            if (true != robot.SetRobotControl(CONTROL_SERVO_ON))
            {
                std::cout << "SetRobotControl(Servo On) failure " << std::endl;
                return 1;
            }
            std::cout << "SetRobotControl(Servo On) done " << std::endl;
            robot.set_robot_mode(ROBOT_MODE_MANUAL);
            robot.SetRobotControl(CONTROL_SERVO_ON);
            break;
        }
        case '6': // Add and remove tool payload dynamically
        {
            std::string Symbol;
            float Weight;
            float Cog[3] = { 0., 0., 0. };
            float Inertia[6] = { 0., 0., 0., 0., 0., 0. };
            bool result_check = false;
            LPSAFETY_CONFIGURATION_EX2 check_safety;

            Symbol = "generateRandomString";
            //generateRandomValue(Weight, 0.1, g_max_fWeight);
            //generateRandomArray(Cog, 3, 0);
            //generateRandomArray(Inertia, 6, 0);
            Weight = 0.5;
            robot.add_tool(Symbol, Weight, Cog, Inertia);

            //DEBUG_PRINT("[Debug] Random Symbol = " << Symbol);
            //DEBUG_PRINT("[Debug] Ramdom Weight = " << Weight);
            //DEBUG_PRINT_ARRAY("[Debug] Random Cog = ", Cog, 3);
            //DEBUG_PRINT_ARRAY("[Debug] Random Inertia = ", Inertia, 6);
            std::this_thread::sleep_for(std::chrono::milliseconds(2000));

            robot.del_tool(Symbol);
            break;
        }
        case '7': // Hold-to-run test APIs
        {
            bool ok = robot.hold2run();
            if (ok) {
                std::cout << "hold2run -> OK" << std::endl;
            }
            else {
                std::cout << "hold2run -> FAIL" << std::endl;
            }

            ok = robot.jog_h2r(JOG_AXIS_JOINT_1, MOVE_REFERENCE_BASE, 5.0f);
            if (ok) {
                std::cout << "jog_h2r(J1, BASE, 5.0) -> OK" << std::endl;
            }
            else {
                std::cout << "jog_h2r(J1, BASE, 5.0) -> FAIL" << std::endl;
            }

            float jpos[NUM_JOINT] = { 0.f, 0.f, 10.f, 0.f, 0.f, 0.f };
            float jvel[NUM_JOINT] = { 30.f, 30.f, 30.f, 30.f, 30.f, 30.f };
            float jacc[NUM_JOINT] = { 30.f, 30.f, 30.f, 30.f, 30.f, 30.f };
            ok = robot.movej_h2r(jpos, jvel, jacc);
            if (ok) {
                std::cout << "movej_h2r -> OK" << std::endl;
            }
            else {
                std::cout << "movej_h2r -> FAIL" << std::endl;
            }

            float tpos[NUM_TASK] = { 0.f, 0.f, 10.f, 0.f, 0.f, 0.f };
            float tvel[2] = { 30.f, 30.f };
            float tacc[2] = { 30.f, 30.f };
            ok = robot.movel_h2r(tpos, tvel, tacc);
            if (ok) {
                std::cout << "movel_h2r -> OK" << std::endl;
            }
            else {
                std::cout << "movel_h2r -> FAIL" << std::endl;
            }
            break;
        }
        case '8': // Register set/get test
        {
            unsigned short addr_bit = -1;
            unsigned short addr_int = 0;
            unsigned short addr_float = 0;

            bool ok = robot.set_output_register_bit(addr_bit, 1);
            if (ok) {
                std::cout << "set_output_register_bit -> OK" << std::endl;
            }
            else {
                std::cout << "set_output_register_bit -> FAIL" << std::endl;
            }
            ok = robot.set_output_register_int(addr_int, 1);
            if (ok) {
                std::cout << "set_output_register_int -> OK" << std::endl;
            }
            else {
                std::cout << "set_output_register_int -> FAIL" << std::endl;
            }
            ok = robot.set_output_register_float(addr_float, 1.00f);
            if (ok) {
                std::cout << "set_output_register_float -> OK" << std::endl;
            }
            else {
                std::cout << "set_output_register_float -> FAIL" << std::endl;
            }

            int out_bit = -1;
            int out_int = 0;
            float out_float = 0.f;
            ok = robot.get_output_register_bit(addr_bit, out_bit);
            if (ok) {
                std::cout << "get_output_register_bit -> OK val=" << out_bit << std::endl;
            }
            else {
                std::cout << "get_output_register_bit -> FAIL val=" << out_bit << std::endl;
            }
            ok = robot.get_output_register_int(addr_int, out_int);
            if (ok) {
                std::cout << "get_output_register_int -> OK val=" << out_int << std::endl;
            }
            else {
                std::cout << "get_output_register_int -> FAIL val=" << out_int << std::endl;
            }
            ok = robot.get_output_register_float(addr_float, out_float);
            if (ok) {
                std::cout << "get_output_register_float -> OK val=" << out_float << std::endl;
            }
            else {
                std::cout << "get_output_register_float -> FAIL val=" << out_float << std::endl;
            }

            ok = robot.get_input_register_bit(addr_bit, out_bit);
            if (ok) {
                std::cout << "get_input_register_bit -> OK val=" << out_bit << std::endl;
            }
            else {
                std::cout << "get_input_register_bit -> FAIL val=" << out_bit << std::endl;
            }
            ok = robot.get_input_register_int(addr_int, out_int);
            if (ok) {
                std::cout << "get_input_register_int -> OK val=" << out_int << std::endl;
            }
            else {
                std::cout << "get_input_register_int -> FAIL val=" << out_int << std::endl;
            }
            ok = robot.get_input_register_float(addr_float, out_float);
            if (ok) {
                std::cout << "get_input_register_float -> OK val=" << out_float << std::endl;
            }
            else {
                std::cout << "get_input_register_float -> FAIL val=" << out_float << std::endl;
            }
            break;
        }
        case 'q': // Quit
        {
            running = false;
            break;
        }
        case '9': // (Reserved)
        {
            break;
        }
        default:
        {
            std::cout << "unknown option" << std::endl;
            break;
        }
        }
    }

    robot.close_connection();
    std::cout << "Connection closed" << std::endl;
    return 0;
}