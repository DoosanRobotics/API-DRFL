//#define DRCF_VERSION 2

#include "../../include/DRFLEx.h"
#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdio>
#include <cstring>
#include <iostream>
#include <thread>

using namespace DRAFramework;

// ===== 전역 상태 =====
static std::atomic<bool> g_got_access{false};
static std::atomic<bool> g_is_standby{false};
static std::atomic<bool> g_run{true};

// ===== enum → 문자열 =====
static const char* to_str(MONITORING_ACCESS_CONTROL x)
{
    switch (x) {
        case MONITORING_ACCESS_CONTROL_REQUEST: return "REQUEST";
        case MONITORING_ACCESS_CONTROL_DENY:    return "DENY";
        case MONITORING_ACCESS_CONTROL_GRANT:   return "GRANT";
        case MONITORING_ACCESS_CONTROL_LOSS:    return "LOSS";
        default:                                return "UNKNOWN";
    }
}

static const char* to_str(ROBOT_STATE x)
{
    switch (x) {
        case STATE_INITIALIZING:   return "INITIALIZING";
        case STATE_STANDBY:        return "STANDBY";
        case STATE_MOVING:         return "MOVING";
        case STATE_SAFE_OFF:       return "SAFE_OFF";
        case STATE_TEACHING:       return "TEACHING";
        case STATE_SAFE_STOP:      return "SAFE_STOP";
        case STATE_EMERGENCY_STOP: return "EMERGENCY_STOP";
        case STATE_HOMMING:        return "HOMMING";
        case STATE_RECOVERY:       return "RECOVERY";
        case STATE_SAFE_STOP2:     return "SAFE_STOP2";
        case STATE_SAFE_OFF2:      return "SAFE_OFF2";
        case STATE_NOT_READY:      return "NOT_READY";
        default:                   return "UNKNOWN";
    }
}

// force singular mode
static const char* to_str(SINGULARITY_FORCE_HANDLING m)
{
    switch (m) {
        case SINGULARITY_ERROR:  return "SINGULARITY_ERROR (error & stop)";
        case SINGULARITY_IGNORE: return "SINGULARITY_IGNORE (ignore error)";
        default:                 return "UNKNOWN";
    }
}

// ===== Force-Control 테스트 함수 =====
static void run_force_singularity_test(CDRFLEx& robot, SINGULARITY_FORCE_HANDLING mode)
{
    std::cout << "\n=== run_force_singularity_test (" << to_str(mode) << ") ===\n";

    // 0) 특이점 force 예외 처리 모드 설정
    bool ok_bool = robot.set_singular_handling_force(mode);
    std::cout << "set_singular_handling_force -> " << (ok_bool ? "OK" : "FAIL") << std::endl;
    if (!ok_bool) return;

    // TOOL 기준으로 force 제어 수행
    int ret = robot.set_ref_coord(COORDINATE_SYSTEM_TOOL);
    std::cout << "set_ref_coord(COORDINATE_SYSTEM_TOOL) -> " << (ret == 1 ? "OK" : "FAIL") << std::endl;

    // 특이점 근처 자세로 천천히 이동
    float q_sing[6] = {0.0f, -40.0f, 80.0f, 0.0f, 40.0f, 0.0f}; 
    std::cout << "[INFO] movej to near-singular pose (slow)...\n";
    robot.movej(q_sing, 30.0f, 30.0f);  
    std::this_thread::sleep_for(std::chrono::seconds(1));

    // 컴플라이언스 시작 (stiffness 설정)
    float stx[6] = {300.f, 300.f, 300.f, 20.f, 20.f, 20.f};
    ret = robot.task_compliance_ctrl(stx);
    std::cout << "task_compliance_ctrl -> " << (ret == 1 ? "OK" : "FAIL") << std::endl;
    if (ret != 1) return;

    float fd[6] = {0.f, 0.f, -5.f, 0.f, 0.f, 0.f};
    unsigned char fdir[6] = {0, 0, 1, 0, 0, 0}; // Z축만 force-control, 나머지 축은 compliance

    ROBOT_STATE cur_state = robot.get_robot_state();
    std::cout << "[DEBUG] state before set_desired_force: " << to_str(cur_state) << std::endl;
    ret = robot.set_desired_force(fd, fdir);
    std::cout << "set_desired_force -> " << (ret == 1 ? "OK" : "FAIL") << std::endl;
    cur_state = robot.get_robot_state();
    std::cout << "[DEBUG] state after set_desired_force:  " << to_str(cur_state) << std::endl;
    if (ret != 1) {
        robot.release_compliance_ctrl();
        return;
    }

    for (int i = 0; i < 50 && g_run; ++i) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    // 컴플라이언스 해제 → 위치 제어로 복귀
    ret = robot.release_compliance_ctrl();
    std::cout << "release_compliance_ctrl -> " << (ret == 1 ? "OK" : "FAIL") << std::endl;

    std::cout << "=== End of Force Test (" << to_str(mode) << ") ===\n";
}

// ===== 콜백 (캡처 없음 → 함수포인터로 OK) =====
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

void OnLogAlarm(LPLOG_ALARM tLog) {
  cout << "Alarm Info: "
       << "group(" << (unsigned int)tLog->_iGroup << "), index("
       << tLog->_iIndex << "), param(" << tLog->_szParam[0] << "), param("
       << tLog->_szParam[1] << "), param(" << tLog->_szParam[2] << ")" << endl;
}

// Ctrl+C
static void SigHandler(int){ g_run = false; }


// ======================================================================
// MAIN
// ======================================================================

int main()
{
    std::signal(SIGINT, SigHandler);

    const std::string IP_ADDRESS = "192.168.137.100";
    CDRFLEx robot;

    robot.set_on_monitoring_access_control(OnAccessCB);
    robot.set_on_monitoring_state(OnStateCB);
    robot.set_on_log_alarm(OnLogAlarm);

    if (!robot.open_connection(IP_ADDRESS)) {
        std::cerr << "[ERR] open_connection failed\n";
        return 1;
    }

    robot.setup_monitoring_version(1);

    SYSTEM_VERSION ver{};
    if (robot.get_system_version(&ver)) {
        std::cout << "DRCF: " << ver._szController
                  << " | LIB: " << robot.get_library_version() << std::endl;
    }
    
    // 제어권 확보
    for (int i = 0; i < 10; ++i) {
        if (!g_got_access) robot.ManageAccessControl(MANAGE_ACCESS_CONTROL_FORCE_REQUEST);
        if (!g_is_standby) robot.set_robot_control(CONTROL_SERVO_ON);

        if (g_got_access && g_is_standby) break;
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }

    if (!(g_got_access && g_is_standby)) {
        std::cerr << "[ERR] Not ready\n";
        return 1;
    }

    robot.set_robot_system(ROBOT_SYSTEM_REAL);
    robot.set_robot_mode(ROBOT_MODE_AUTONOMOUS);

    std::cout << "\n=== Motion singularity handling test ===\n";
    robot.set_singularity_handling(SINGULARITY_AVOIDANCE_AVOID);
    robot.set_singularity_handling(SINGULARITY_AVOIDANCE_STOP);
    robot.set_singularity_handling(SINGULARITY_AVOIDANCE_VEL);

    std::cout << "\n=== Force singularity handling REAL TEST ===\n";

    // 테스트 #1 — ERROR 모드
    run_force_singularity_test(robot, SINGULARITY_ERROR);

    // 특이점 진입 시 Fault가 뜰 가능성 높으므로 컨트롤러 리셋 필요할 수 있음
    std::cout << "\n[INFO] 테스트 1 종료. 컨트롤러 Fault 상태면 reset 후 IGNORE 테스트하세요.\n";

    // 테스트 #2 — IGNORE 모드 (Fault 없음)
    //run_force_singularity_test(robot, SINGULARITY_IGNORE);

    robot.close_connection();
    return 0;
}
