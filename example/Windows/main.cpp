
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

const std::string IP_ADDRESS = "192.168.137.100";

using namespace DRAFramework;
CDRFLEx robot;

bool g_bHasControlAuthority = FALSE;
bool g_TpInitailizingComplted = FALSE;
bool control_authority_granted = false;
static std::atomic<bool> g_run{true}; 


// void OnTpInitializingCompleted() {
//   // Tp 초기화 이후 제어권 요청.
//   g_TpInitailizingComplted = TRUE;
//   robot.ManageAccessControl(MANAGE_ACCESS_CONTROL_FORCE_REQUEST);
// }
MONITORING_ACCESS_CONTROL control_access = MONITORING_ACCESS_CONTROL_LAST;

void OnMonitoringStateCB(const ROBOT_STATE eState) {
  // 50msec 이내 작업만 수행할 것.

  std::cout << "[USER][OnMonitoringStateCB] state : " << to_str(eState) << std::endl;
  return;

  // switch ((unsigned char)eState) {
  //   case STATE_EMERGENCY_STOP:
  //     // popup
  //     break;
  //   case STATE_STANDBY:
  //   case STATE_MOVING:
  //   case STATE_TEACHING:
  //     break;
  //   case STATE_SAFE_STOP:
  //     if (g_bHasControlAuthority) {
  //       robot.SetSafeStopResetType(SAFE_STOP_RESET_TYPE_DEFAULT);
  //       robot.SetRobotControl(CONTROL_RESET_SAFET_STOP);
  //     }
  //     break;
  //   case STATE_SAFE_OFF:
  //     // if (g_bHasControlAuthority) {
  //     //   robot.SetRobotControl(CONTROL_SERVO_ON);
  //     // }
  //     break;
  //   case STATE_SAFE_STOP2:
  //     if (g_bHasControlAuthority)
  //       robot.SetRobotControl(CONTROL_RECOVERY_SAFE_STOP);
  //     break;
  //   case STATE_SAFE_OFF2:
  //     if (g_bHasControlAuthority) {
  //       robot.SetRobotControl(CONTROL_RECOVERY_SAFE_OFF);
  //     }
  //     break;
  //   case STATE_RECOVERY:
  //     // Drfl.SetRobotControl(CONTROL_RESET_RECOVERY);
  //     break;
  //   default:
  //     break;
  // }
  // return;
}

void OnMonitroingAccessControlCB(
    const MONITORING_ACCESS_CONTROL eTrasnsitControl) {
  // 50msec 이내 작업만 수행할 것.
  std::cout << "[OnMonitroingAccessControlCB] : " << to_str(eTrasnsitControl) << std::endl;
  control_access = eTrasnsitControl;
  return;
  // switch (eTrasnsitControl) {
  //   case MONITORING_ACCESS_CONTROL_REQUEST:
  //     robot.ManageAccessControl(MANAGE_ACCESS_CONTROL_RESPONSE_NO);
  //     // Drfl.ManageAccessControl(MANAGE_ACCESS_CONTROL_RESPONSE_YES);
  //     break;
  //   case MONITORING_ACCESS_CONTROL_GRANT:
  //     g_bHasControlAuthority = TRUE;

  //     if (true != robot.reset_sequence(0)) { // if we make sure turning on servo on, should be exist.
  //       std::cout << "reset_sequence failure " << std::endl;
  //       return;
  //     }
  //     std::cout << "reset_sequence done" <<std::endl;

  //     if(robot.GetRobotState() == STATE_SAFE_OFF){
  //       if(true!=robot.SetRobotControl(CONTROL_SERVO_ON)){
  //         std::cout << "SetRobotControl(Servo On) failure " << std::endl;
  //         return;
  //       }
  //       std::cout << "SetRobotControl(Servo On) done " << std::endl;
  //     }
  //     break;
  //   case MONITORING_ACCESS_CONTROL_DENY:
  //   case MONITORING_ACCESS_CONTROL_LOSS:
  //     g_bHasControlAuthority = FALSE;
  //     if (g_TpInitailizingComplted) {
  //       // assert(Drfl.ManageAccessControl(MANAGE_ACCESS_CONTROL_REQUEST));
  //       robot.ManageAccessControl(MANAGE_ACCESS_CONTROL_FORCE_REQUEST);
  //     }
  //     break;
  //   default:
  //     break;
  // }
}


void OnLogAlarm(LPLOG_ALARM tLog) {
  // cout << "OnLogAlarm"<<std::endl;
}
void OnMonitoringDataExCB(const LPMONITORING_DATA_EX pData) {
  // cout << "OnMonitoringDataExCB"<<std::endl;
}

void OnMonitoringCrtlIoExCB(const LPMONITORING_CTRLIO_EX pData) {
  // cout << "OnMonitoringDataExCB"<<std::endl;
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

// Ctrl+C
static void SigHandler(int){ g_run = false; }

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
    std::cout << "set_ref_coord(COORDINATE_SYSTEM_TOOL) -> "
              << (ret == 1 ? "OK" : "FAIL") << std::endl;

    // 특이점 근처 자세 (예시 값, 실제 로봇에 맞게 조정 가능)
    float q_sing[6] = {0.0f, -40.0f, 80.0f, 0.0f, 40.0f, 0.0f};
    std::cout << "[INFO] movej to near-singular pose (slow)...\n";
    robot.movej(q_sing, 30.0f, 30.0f);
    std::this_thread::sleep_for(std::chrono::seconds(1));

    // 컴플라이언스 시작 (stiffness)
    float stx[6] = {300.f, 300.f, 300.f, 20.f, 20.f, 20.f};
    ret = robot.task_compliance_ctrl(stx);
    std::cout << "task_compliance_ctrl -> " << (ret == 1 ? "OK" : "FAIL") << std::endl;
    if (ret != 1) return;

    // 원하는 힘/축 설정 (예시: Z- 방향으로 5N)
    float fd[6]      = {0.f, 0.f, -5.f, 0.f, 0.f, 0.f};
    unsigned char fdir[6] = {0, 0, 1, 0, 0, 0}; // Z축만 force 제어

    ROBOT_STATE cur_state = robot.GetRobotState(); // 또는 get_robot_state()
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

    std::cout << "[INFO] 외력을 주어 특이점 방향으로 살짝 밀어 보세요.\n";
    std::cout << "       " << to_str(mode) << " 모드에 따라 Fault 발생 여부가 달라집니다.\n";

    for (int i = 0; i < 50 && g_run; ++i) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    // 컴플라이언스 해제
    ret = robot.release_compliance_ctrl();
    std::cout << "release_compliance_ctrl -> " << (ret == 1 ? "OK" : "FAIL") << std::endl;

    std::cout << "=== End of Force Test (" << to_str(mode) << ") ===\n";
}



int main() {
  bool ret;
  
  ret = robot.open_connection(IP_ADDRESS);
  if (true != ret) {
    std::cout << "Cannot open connection to robot @ " << IP_ADDRESS
              << std::endl;
    return 1;
  }
  //if (true != robot.reset_sequence(0)) { // if we make sure turning on servo on, should be exist.
  //    std::cout << "reset_sequence failure " << std::endl;
  //    return 1;
  //}
  //std::cout << "Set open_connection done" <<std::endl;

  if(true != robot.setup_monitoring_version(1)) {
    std::cout << "Cannot set monitoring version " << std::endl;
    return 1;
  }
  std::cout << "Set setup_monitoring_version done" <<std::endl;

  // robot.set_on_tp_initializing_completed(OnTpInitializingCompleted);

  // Registered empty callbacks (drfl down if received none for 3 sec.)
  
  robot.set_on_monitoring_data_ex(OnMonitoringDataExCB);
  robot.set_on_monitoring_state(OnMonitoringStateCB);
  robot.set_on_log_alarm(OnLogAlarm);
  //robot.set_on_monitoring_ctrl_io_ex(OnMonitoringCrtlIoExCB);
  // robot.set_auto_safety_move_stop(true);
  
  
  // Manage Access Control seems to mean accessing monitoring data in controller.
  robot.set_on_monitoring_access_control(OnMonitroingAccessControlCB);
  if(true != robot.ManageAccessControl(MANAGE_ACCESS_CONTROL_FORCE_REQUEST)) { 
    std::cout << "Cannot set ManageAccessControl - MANAGE_ACCESS_CONTROL_FORCE_REQUEST " << std::endl;
    return 1;
  }

  while(control_access != MONITORING_ACCESS_CONTROL_GRANT) {
    std::cout << "Sleep until control grant... " << std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  }
  std::cout << "Control Granted ! " << std::endl;
  robot.set_robot_mode(ROBOT_MODE_MANUAL);

  ROBOT_STATE robot_state = robot.GetRobotState();
  std::cout << "Robot State : " << to_str(robot_state);

  if(robot_state == STATE_SAFE_OFF){
    if(true!=robot.SetRobotControl(CONTROL_SERVO_ON)){
      std::cout << "SetRobotControl(Servo On) failure " << std::endl;
      return 1;
    }
    std::cout << "SetRobotControl(Servo On) done " << std::endl;
  }
  robot.set_robot_mode(ROBOT_MODE_AUTONOMOUS);


  while(true){
    std::cout << "READY TO INPUT : " << std::endl;
    char input;
    cin >> input;
    switch(input){
      case '1':
      {
        float pos[6] = {0., 0., 0., 0., 0.,0.};
        float fTargetVel = 30.0;
        float fTargetAcc = 30.0;
        robot.movej(pos, fTargetVel, fTargetAcc);

        float pos2[6] = { 0., 0., 90., 0., 90.,0. };
        float fTargetVel2 = 30.0;
        float fTargetAcc2 = 30.0;
        robot.movej(pos2, fTargetVel2, fTargetAcc2);
        
        break;
      }
      case '2':
      {
        // DRCF_VERSION 3 
        // POSITION_EX tTargetPos;
        // float pos[6] = {400., 500., 800., 0., 180.,0.};
        // memcpy(tTargetPos._posx._pos, pos, sizeof(float)*6);
        // tTargetPos._pos_type = POSX;
        // float fTargetVel[2] = {30,30};
        // float fTargetAcc[2] = {30,30};
        // if(true != robot.movel(&tTargetPos, fTargetVel, fTargetAcc)) {
        //   std::cout << "Cannot set movel1 " << std::endl;
        //   return 1;
        // }
        // std::cout << "movel 1 " << std::endl;

        // float pos1[6] = {400., 500., 500., 0., 180.,0.};
        // memcpy(tTargetPos._posx._pos, pos1, sizeof(float)*6);
        // if(true != robot.movel(&tTargetPos, fTargetVel, fTargetAcc)) {
        //   std::cout << "Cannot set movel2 " << std::endl;
        //   return 1;
        // }
        // std::cout << "movel2 " << std::endl;

        // float pos2[6] = {400., 500., 800., 0., 180.,0.};
        // memcpy(tTargetPos._posx._pos, pos2, sizeof(float)*6);
        // if(true != robot.movel(&tTargetPos, fTargetVel, fTargetAcc)) {
        //   std::cout << "Cannot set movel3 " << std::endl;
        //   return 1;
        // }
        // std::cout << "movel3 " << std::endl;
        break;
      }
      case '3':
      {
        bool ok = false;

        std::cout << "\n=== Test: set_singularity_handling (motion) ===\n";

        ok = robot.set_singularity_handling(SINGULARITY_AVOIDANCE_AVOID);
        std::cout << "set_singularity_handling(AVOID) result: "
                  << (ok ? "OK" : "FAIL") << std::endl;

        ok = robot.set_singularity_handling(SINGULARITY_AVOIDANCE_STOP);
        std::cout << "set_singularity_handling(STOP)  result: "
                  << (ok ? "OK" : "FAIL") << std::endl;

        ok = robot.set_singularity_handling(SINGULARITY_AVOIDANCE_VEL);
        std::cout << "set_singularity_handling(VEL)   result: "
                  << (ok ? "OK" : "FAIL") << std::endl;

        std::cout << "\n=== Test: set_singular_handling_force (force/compliance) ===\n";

        SINGULARITY_FORCE_HANDLING mode = SINGULARITY_ERROR;
        ok = robot.set_singular_handling_force(mode);
        std::cout << "set_singular_handling_force(" << to_str(mode)
                  << ") result: " << (ok ? "OK" : "FAIL") << std::endl;

        mode = SINGULARITY_IGNORE;
        ok = robot.set_singular_handling_force(mode);
        std::cout << "set_singular_handling_force(" << to_str(mode)
                  << ") result: " << (ok ? "OK" : "FAIL") << std::endl;

        break;
      }
      case '4':
      {
        std::cout << "\n[WARN] 실제 로봇에서 테스트 시 속도/힘을 꼭 낮게 설정하고,"
                  << " 비상정지 준비한 상태에서 실행하세요.\n";
        run_force_singularity_test(robot, SINGULARITY_ERROR);
        break;
      }
      case '5':
      {
        std::cout << "\n[INFO] 컨트롤러 Fault 상태가 아니어야 합니다.\n";
        run_force_singularity_test(robot, SINGULARITY_IGNORE);
        break;
      }
      case '6':
      {
          //robot.MoveStop(STOP_TYPE_QUICK_STO);
          // robot.SetRobotControl(CONTROL_RECOVERY_SAFE_OFF);
          //robot.SetRobotControl(CONTROL_SERVO_ON);
          //if(true!=robot.SetRobotControl(CONTROL_SERVO_ON)){
        //  std::cout << "SetRobotControl(Servo On) failure " << std::endl;
        //  return 1;
        //}
        //std::cout << "SetRobotControl(Servo On) done " << std::endl;
          //robot.set_robot_mode(ROBOT_MODE_MANUAL);
          //robot.SetRobotControl(CONTROL_SERVO_ON);
          break;
      }
      case 'a':
      {
          
          robot.set_robot_mode(ROBOT_MODE_MANUAL);
          robot.SetRobotControl(CONTROL_SERVO_ON);
          break;
      }
      case '7':
      {


        //std::cout <<  e << std::endl;
        //std::cout << "3333" <<std::endl;
      }
      case '8':
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
      case '9':
      {
          robot.set_robot_mode(ROBOT_MODE_MANUAL);
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
