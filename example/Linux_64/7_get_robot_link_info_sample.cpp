//#define DRCF_VERSION 2

#include "../../include/DRFLEx.h"
#include "../../include/DRFS.h"   // ROBOT_LINK_INFO

#include <atomic>
#include <iostream>
#include <thread>
#include <cstdio>
#include <chrono>
#include <cmath>

using namespace DRAFramework;

// ===== 전역 상태 (콜백에서 캡처 없이 접근) =====
static std::atomic<bool> g_got_access{false};
static std::atomic<bool> g_is_standby{false};
static std::atomic<bool> g_poll_run{false};  // 폴링 스레드 on/off

// ===== 도우미 출력/비교 함수 =====
static std::string to_str(const MONITORING_ACCESS_CONTROL x)
{
    switch (x) {
        case MONITORING_ACCESS_CONTROL_REQUEST: return "REQUEST";
        case MONITORING_ACCESS_CONTROL_DENY:    return "DENY";
        case MONITORING_ACCESS_CONTROL_GRANT:   return "GRANT";
        case MONITORING_ACCESS_CONTROL_LOSS:    return "LOSS";
        default:                                 return "UNKNOWN";
    }
}
static std::string to_str(const ROBOT_STATE x)
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
static void print6(const char* tag, const float v[6]){
    std::printf("%s: %.6f %.6f %.6f %.6f %.6f %.6f\n",
                tag, v[0], v[1], v[2], v[3], v[4], v[5]);
}
static void print_link(const ROBOT_LINK_INFO& x, const char* title){
    std::puts(title);
    print6("a[m]      ", x.a);
    print6("d[m]      ", x.d);
    print6("alpha[rad]", x.alpha);
    print6("theta[rad]", x.theta);
    print6("offset[rad]", x.offset);
    std::printf("gradient: %.6f, rotation: %.6f\n\n", x.gradient, x.rotation);
}
static bool neq6(const float a[6], const float b[6], float eps=1e-7f){
    for(int i=0;i<6;++i) if (std::fabs(a[i]-b[i])>eps) return true;
    return false;
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

// ===== 200ms 폴링 스레드 (값 변할 때만 출력) =====
static void dh_poll_thread(CDRFLEx* pRobot){
    ROBOT_LINK_INFO prev{}; 
    bool has_prev=false;

    while(g_poll_run.load()){
        ROBOT_LINK_INFO cur{};
        if (pRobot->get_robot_link_info(cur, 300)) {
            if (!has_prev ||
                neq6(prev.a,cur.a) || neq6(prev.d,cur.d) ||
                neq6(prev.alpha,cur.alpha) || neq6(prev.theta,cur.theta) ||
                neq6(prev.offset,cur.offset) ||
                std::fabs(prev.gradient-cur.gradient)>1e-7f ||
                std::fabs(prev.rotation-cur.rotation)>1e-7f) {

                print_link(cur, "[DH update]");
                prev = cur; 
                has_prev = true;
            }
        } else {
            std::fprintf(stderr, "[WARN] get_robot_link_info timeout/fail\n");
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(200)); // 5Hz
    }
}

int main()
{
    const std::string IP_ADDRESS = "127.0.0.1";
    CDRFLEx robot;

    // 콜백 등록
    robot.set_on_monitoring_access_control(OnAccessCB);
    robot.set_on_monitoring_state(OnStateCB);

    // 연결
    if (!robot.open_connection(IP_ADDRESS)) {
        std::cerr << "[ERR] open_connection failed: " << IP_ADDRESS << std::endl;
        return 1;
    }
    robot.setup_monitoring_version(1);

    SYSTEM_VERSION ver = {'\0'};
    robot.get_system_version(&ver);
    std::cout << "DRCF: " << ver._szController
              << " | LIB: " << robot.get_library_version() << std::endl;

    // 제어권/대기상태 확보
    for (int i = 0; i < 10; ++i) {
        if (!g_got_access) robot.ManageAccessControl(MANAGE_ACCESS_CONTROL_FORCE_REQUEST);
        if (!g_is_standby) robot.set_robot_control(CONTROL_SERVO_ON);
        if (g_got_access && g_is_standby) break;
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }
    if (!(g_got_access && g_is_standby)) {
        std::cerr << "[ERR] Not ready (access/standby)" << std::endl;
        return 1;
    }

    // (옵션) 가상/자율 모드
    robot.set_robot_system(ROBOT_SYSTEM_VIRTUAL);
    robot.set_robot_mode(ROBOT_MODE_AUTONOMOUS);

    // ===== 시작 스냅샷 + 폴링 시작 =====
    ROBOT_LINK_INFO link{};
    if (robot.get_robot_link_info(link, 500))
        print_link(link, "=== DH at START ===");
    else
        std::fprintf(stderr, "[ERR] initial get_robot_link_info failed\n");

    g_poll_run = true;
    std::thread th(dh_poll_thread, &robot);

    // ===== 모션 전/후 1회씩 조회 =====
    float q[6] = {0,0,30,0,0,0};

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

    // ===== 폴링 종료 =====
    g_poll_run = false;
    if (th.joinable()) th.join();

    robot.close_connection();
    return 0;
}
