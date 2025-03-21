
#include <chrono>
#include <iostream>
#include <ostream>
#include <string>
#include <thread>
#include <cstring>
#include <ctime>

#include <iostream>
#include <cstring>

//typedef struct _SERIAL_BODY_HEAD {
//    /* serial sequnece number */
//    unsigned short              _iSeqNo;
//    /* serial command id */
//    unsigned short              _iCmdId;
//} SERIAL_BODY_HEAD;
//
//int main(){
//    SERIAL_BODY_HEAD tHead = { 0, 1758, };
//    std::cout << "HEADER - tHead._iSeqNo : " << tHead._iSeqNo << ", tHead._iCmdId : " << tHead._iCmdId << std::endl;
//    int nTotalLength = 0;
//    char szPacket[8192] = {'\0', };
//    // ��� ���
//    memcpy(szPacket + nTotalLength, &tHead, sizeof(SERIAL_BODY_HEAD));
//    nTotalLength += sizeof(SERIAL_BODY_HEAD);
//    std::cout << "111PACKET CONTENTS " << std::endl;
//    for(int i=0;i<nTotalLength; i++){
//        std::cout << "szPacket[" << i << "]: " << (int) szPacket[i] << std::endl;
//    }
//    return 0;
//}

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

void OnMonitoringDataExCB2(const LPMONITORING_CTRLIO_EX2 pData) {
  // cout << "OnMonitoringDataExCB"<<std::endl;
}



/*
1. move 연속으로 했을 때 수행되지 않음. 
2. mwait 모션 수행하지 않음 -> amovej 이동 및 mwait 이용.
*/
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
  robot.set_on_monitoring_ctrl_io_ex2(OnMonitoringDataExCB2);
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

  if (true != robot.reset_sequence(0)) { // if we make sure turning on servo on, should be exist.
    std::cout << "reset_sequence failure " << std::endl;
    return 1;
  }
  std::cout << "reset_sequence done" <<std::endl;
  
  ROBOT_STATE robot_state = robot.GetRobotState();
  std::cout << "Robot State : " << to_str(robot_state);

  if(robot_state == STATE_SAFE_OFF){
    if(true!=robot.SetRobotControl(CONTROL_SERVO_ON)){
      std::cout << "SetRobotControl(Servo On) failure " << std::endl;
      return 1;
    }
    std::cout << "SetRobotControl(Servo On) done " << std::endl;
  }




  while(true){
    std::cout << "READY TO INPUT " << std::endl;
    char input;
    cin >> input;
    switch(input){
      case '1':
      {
        POSITION_EX tTargetPos;
        float pos[6] = {0.,0.,0.,0.,0.,0.};
        memcpy(tTargetPos._posj._pos, pos, sizeof(float)*6);
        tTargetPos._pos_type = POSJ;
        float fTargetVel[NUMBER_OF_JOINT] = {30., 30., 30., 30., 30., 30.};
        float fTargetAcc[NUMBER_OF_JOINT] = {30., 30., 30., 30., 30., 30.};
        if(true != robot.movej(&tTargetPos, fTargetVel, fTargetAcc, 0.f, 0, MOVE_MODE_ABSOLUTE, 0.f, 0 )) {
          std::cout << "Cannot set movej " << std::endl;
          break;
        }
        std::cout << "Set movej done111" <<std::endl;


        float pos2[6] = {0.,0.,30.,0.,30.,0.};
        memcpy(tTargetPos._posj._pos, pos2, sizeof(float)*6);
        if(true != robot.movej(&tTargetPos, fTargetVel, fTargetAcc, 0.f, 0, MOVE_MODE_ABSOLUTE, 0.f, 0 )) {
          std::cout << "Cannot set movej " << std::endl;
          break;
        }
        std::cout << "Set movej done222" <<std::endl;
        break;

 
      }
      case '2':
      {
        POSITION_EX tTargetPos;
        float pos[6] = {0.,0.,30.,0.,30.,0.};
        memcpy(tTargetPos._posj._pos, pos, sizeof(float)*6);
        tTargetPos._pos_type = POSJ;
        float fTargetVel[NUMBER_OF_JOINT] = {30., 30., 30., 30., 30., 30.};
        float fTargetAcc[NUMBER_OF_JOINT] = {30., 30., 30., 30., 30., 30.};
        
        if(true != robot.movej(&tTargetPos, fTargetVel, fTargetAcc, 0.f, 0, MOVE_MODE_ABSOLUTE, 0.f, 0 )) {
          std::cout << "Cannot set movej " << std::endl;
          return 1;
        }
        std::cout << "Set movej done" <<std::endl;
        break;
      }
      case '3':
      {
        POSITION_EX tTargetPos;
        float pos[6] = {400., 500., 800., 0., 180.,0.};
        memcpy(tTargetPos._posx._pos, pos, sizeof(float)*6);
        tTargetPos._pos_type = POSX;
        float fTargetVel[2] = {30,30};
        float fTargetAcc[2] = {30,30};
        if(true != robot.movel(&tTargetPos, fTargetVel, fTargetAcc)) {
          std::cout << "Cannot set movel1 " << std::endl;
          return 1;
        }
        std::cout << "movel 1 " << std::endl;

        float pos1[6] = {400., 500., 500., 0., 180.,0.};
        memcpy(tTargetPos._posx._pos, pos1, sizeof(float)*6);
        if(true != robot.movel(&tTargetPos, fTargetVel, fTargetAcc)) {
          std::cout << "Cannot set movel2 " << std::endl;
          return 1;
        }
        std::cout << "movel2 " << std::endl;

        float pos2[6] = {400., 500., 800., 0., 180.,0.};
        memcpy(tTargetPos._posx._pos, pos2, sizeof(float)*6);
        if(true != robot.movel(&tTargetPos, fTargetVel, fTargetAcc)) {
          std::cout << "Cannot set movel3 " << std::endl;
          return 1;
        }
        std::cout << "movel3 " << std::endl;
        // float pos2[6] = {30., 30., 30., 0., 0.,0.};
        // memcpy(tTargetPos._posx._pos, pos2, sizeof(float)*6);
        // robot.movel(&tTargetPos, fTargetVel, fTargetAcc);
        break;
      }
      case '4':
      {
        POSITION_EX tTargetPos;
        float pos[6] = {0.,0.,50.,0.,50.,0.};
        memcpy(tTargetPos._posj._pos, pos, sizeof(float)*6);
        tTargetPos._pos_type = POSJ;
        float fTargetVel[NUMBER_OF_JOINT] = {10., 10., 10., 10., 10., 10.};
        float fTargetAcc[NUMBER_OF_JOINT] = {10., 10., 10., 10., 10., 10.};
        
        if(true != robot.amovej(&tTargetPos, fTargetVel, fTargetAcc, 0.f, 0, MOVE_MODE_ABSOLUTE, 0.f, 0 )) {
          std::cout << "Cannot set movej " << std::endl;
          break;
        }
         
        std::cout << "Set movej done" <<std::endl;
        robot.mwait();
        std::string e = "QUERY STATE  : " + to_str(robot.query_operating_state());
        std::cout <<  e << std::endl;

        break;
      }
      case '5':
      {
        POSITION_EX tTargetPos;
        float pos[6] = {0.,0.,0.,0.,0.,0.};
        memcpy(tTargetPos._posj._pos, pos, sizeof(float)*6);
        tTargetPos._pos_type = POSJ;
        float fTargetVel[NUMBER_OF_JOINT] = {10., 10., 10., 10., 10., 10.};
        float fTargetAcc[NUMBER_OF_JOINT] = {10., 10., 10., 10., 10., 10.};
        
        if(true != robot.amovej(&tTargetPos, fTargetVel, fTargetAcc, 0.f, 0, MOVE_MODE_ABSOLUTE, 0.f, 0 )) {
          std::cout << "Cannot set movej " << std::endl;
          break;
        }
        std::cout << "Set movej done" <<std::endl;
        std::cout << "1111" <<std::endl;
        robot.mwait();
        std::string e = "QUERY STATE  : " + to_str(robot.query_operating_state());
        std::cout <<  e << std::endl;
        std::cout << "3333" <<std::endl;
        break;
      }
      case '6':
      {
          //robot.MoveStop(STOP_TYPE_QUICK_STO);
          robot.SetRobotControl(CONTROL_RECOVERY_SAFE_OFF);
          //robot.SetRobotControl(CONTROL_SERVO_ON);
          //if(true!=robot.SetRobotControl(CONTROL_SERVO_ON)){
        //  std::cout << "SetRobotControl(Servo On) failure " << std::endl;
        //  return 1;
        //}
        //std::cout << "SetRobotControl(Servo On) done " << std::endl;
          //robot.set_robot_mode(ROBOT_MODE_MANUAL);
          //robot.SetRobotControl(CONTROL_SERVO_ON);
      }
      case 'a':
      {
          
          robot.set_robot_mode(ROBOT_MODE_MANUAL);
          robot.SetRobotControl(CONTROL_SERVO_ON);
      }
      case '7':
      {
          POSITION_EX tTargetPos;

          float posj[6] = { 180., 0., 90., 0., 90., 0. };
          memcpy(tTargetPos._posj._pos, posj, sizeof(float) * 6);

          float fTargetVel[NUMBER_OF_JOINT] = { 30., 30., 30., 30., 30., 30. };
          float fTargetAcc[NUMBER_OF_JOINT] = { 30., 30., 30., 30., 30., 30. };
          robot.amovej(&tTargetPos, fTargetVel, fTargetAcc, 0.0);
          std::cout << "after amovej/" << std::endl;
          //robot.MoveStop(STOP_TYPE_EMERGENCY);
          robot.stop(STOP_TYPE_EMERGENCY);
          
        //std::string e = "QUERY STATE  : " + to_str(robot.query_operating_state());
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

      }
      case '9':
      {
          robot.set_robot_mode(ROBOT_MODE_MANUAL);

      }
      default:
      {
        std::cout << "none option" << std::endl;
      }
    }
  }

  

  robot.close_connection();
  std::cout << "Connection closed" << std::endl;
  return 0;
}
