#include <chrono>
#include <iostream>
#include <ostream>
#include <string>
#include <thread>
#include <cstring>
#include <ctime>
#include <atomic>

#include <iostream>
#include <cstring>



//
// Specify drfl header.
#include "../../include/DRFLEx.h"
#include "util.hpp"


const std::string IP_ADDRESS = "127.0.0.1";

using namespace DRAFramework;
CDRFLEx robot;

bool g_bHasControlAuthority = FALSE;
bool g_TpInitailizingComplted = FALSE;
bool control_authority_granted = false;

static std::atomic<bool> g_run{ true };
static std::atomic<bool> g_got_access{ false };
static std::atomic<bool> g_is_standby{ false };
static std::atomic<bool> g_poll_run{ false };

MONITORING_ACCESS_CONTROL control_access = MONITORING_ACCESS_CONTROL_LAST;

void OnMonitoringStateCB(const ROBOT_STATE eState) {

    std::cout << "[USER][OnMonitoringStateCB] state : " << to_str(eState) << std::endl;
    return;
}

void OnMonitroingAccessControlCB(
    const MONITORING_ACCESS_CONTROL eTrasnsitControl) {

    std::cout << "[OnMonitroingAccessControlCB] : " << to_str(eTrasnsitControl) << std::endl;
    control_access = eTrasnsitControl;
    return;
}

void OnLogAlarm(LPLOG_ALARM tLog) {
    cout << "Alarm Info: "
        << "group(" << (unsigned int)tLog->_iGroup << "), index("
        << tLog->_iIndex << "), param(" << tLog->_szParam[0] << "), param("
        << tLog->_szParam[1] << "), param(" << tLog->_szParam[2] << ")" << endl;
}

void OnMonitoringDataExCB(const LPMONITORING_DATA_EX pData) {
    //cout << "OnMonitoringDataExCB"<<std::endl;
}

void OnMonitoringDataExCB2(const LPMONITORING_CTRLIO_EX2 pData) {
    //cout << "OnMonitoringDataExCB2"<<std::endl;
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
static void OnStateCB(ROBOT_STATE st)
{
    std::cout << "[State ] " << to_str(st) << std::endl;
    g_is_standby = (st == STATE_STANDBY);
}

static void print6(const char* tag, const float v[6]) 
{
    std::printf("%s: %.6f %.6f %.6f %.6f %.6f %.6f\n",
        tag, v[0], v[1], v[2], v[3], v[4], v[5]);
}

static void print_link(const ROBOT_LINK_INFO& x, const char* title) 
{
    std::puts(title);
    print6("a[m]      ", x.a);
    print6("d[m]      ", x.d);
    print6("alpha[rad]", x.alpha);
    print6("theta[rad]", x.theta);
    print6("offset[rad]", x.offset);
    std::printf("gradient: %.6f, rotation: %.6f\n\n", x.gradient, x.rotation);
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
            std::fprintf(stderr, "[WARN] get_robot_link_info timeout/fail\n");
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }
}


int main() {
    bool ret;

    ret = robot.open_connection(IP_ADDRESS);
    if (true != ret) {
        std::cout << "Cannot open connection to robot @ " << IP_ADDRESS
            << std::endl;
        return 1;
    }

    if (true != robot.setup_monitoring_version(1)) {
        std::cout << "Cannot set monitoring version " << std::endl;
        return 1;
    }
    std::cout << "Set setup_monitoring_version done" << std::endl;

    robot.set_on_monitoring_data_ex(OnMonitoringDataExCB);
    robot.set_on_monitoring_state(OnMonitoringStateCB);
    robot.set_on_log_alarm(OnLogAlarm);
    robot.set_on_monitoring_ctrl_io_ex(OnMonitoringDataExCB2);

    // Manage Access Control seems to mean accessing monitoring data in controller.
    robot.set_on_monitoring_access_control(OnMonitroingAccessControlCB);
    if (true != robot.ManageAccessControl(MANAGE_ACCESS_CONTROL_FORCE_REQUEST)) {
        std::cout << "Cannot set ManageAccessControl - MANAGE_ACCESS_CONTROL_FORCE_REQUEST " << std::endl;
        return 1;
    }

    while (control_access != MONITORING_ACCESS_CONTROL_GRANT) {
        std::cout << "Sleep until control grant... " << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }
    std::cout << "Control Granted ! " << std::endl;

    ROBOT_STATE robot_state = robot.GetRobotState();
    std::cout << "Robot State : " << to_str(robot_state) << std::endl;
    robot.set_robot_mode(ROBOT_MODE_AUTONOMOUS);
    //robot.set_robot_mode(ROBOT_MODE_MANUAL);

    if (robot_state == STATE_SAFE_OFF) {
        if (true != robot.SetRobotControl(CONTROL_SERVO_ON)) {
            std::cout << "SetRobotControl(Servo On) failure " << std::endl;
            return 1;
        }
        std::cout << "SetRobotControl(Servo On) done " << std::endl;
    }

    while (true) {
        cout << "READY TO INPUT : " << endl;
        char input;
        cin >> input;
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
        case '3': // Test task-space singularity handling modes
        {
            bool ok;
            ok = robot.set_singularity_handling(SINGULARITY_AVOIDANCE_AVOID);
            std::cout << "set_singularity_handling(AVOID) -> " << (ok ? "OK" : "FAIL") << "\n";
            ok = robot.set_singularity_handling(SINGULARITY_AVOIDANCE_STOP);
            std::cout << "set_singularity_handling(STOP)  -> " << (ok ? "OK" : "FAIL") << "\n";
            ok = robot.set_singularity_handling(SINGULARITY_AVOIDANCE_VEL);
            std::cout << "set_singularity_handling(VEL)   -> " << (ok ? "OK" : "FAIL") << "\n";
            break;
        }
        case '4': // Force control test near singularity (ERROR mode)
        {
            bool ok = robot.set_singular_handling_force(SINGULARITY_ERROR);
            std::cout << "set_singular_handling_force(ERROR) -> " << (ok ? "OK" : "FAIL") << "\n";
            run_force_singularity_test(robot, SINGULARITY_ERROR);
            break;
        }
        case '5': // Force control test near singularity (IGNORE mode)
        {
            bool ok = robot.set_singular_handling_force(SINGULARITY_IGNORE);
            std::cout << "set_singular_handling_force(IGNORE) -> " << (ok ? "OK" : "FAIL") << "\n";
            run_force_singularity_test(robot, SINGULARITY_IGNORE);
            break;
        }
        case '6': // Emergency stop and recovery sequence
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
        case '7': // Add and remove tool payload dynamically
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
        }
        case '8': // (Reserved)
        {
            break;
        }
        case '9': // (Reserved)
        {
            break;
        }
        default:
        {
            std::cout << "none option" << std::endl;
            break;
        }
        }
    }

    robot.close_connection();
    std::cout << "Connection closed" << std::endl;
    return 0;
}
