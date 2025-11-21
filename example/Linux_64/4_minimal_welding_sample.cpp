#define DRCF_VERSION 2

#include <iostream>
#include <cstring>
#include <thread>
#include <chrono>
#include <termios.h>
#include <unistd.h>

#include "../../include/DRFLEx.h"
using namespace DRAFramework;
using namespace std::chrono_literals;

#undef NDEBUG
#include <assert.h>
 
CDRFLEx Drfl;
bool g_bHasControlAuthority = FALSE;
bool g_TpInitailizingComplted = FALSE;
bool moving = FALSE;
bool bAlterFlag = FALSE;
bool is_standby = FALSE;

std::string IP_ADDR = "192.168.137.100";
std::string getWeldingSampleDrl();

void OnTpInitializingCompleted() {
  // Request control authority after TP initialization is completed
  g_TpInitailizingComplted = TRUE;
  Drfl.ManageAccessControl(MANAGE_ACCESS_CONTROL_FORCE_REQUEST);
}

void OnHommingCompleted() {
  // Only do work that finishes within 50 msec
  cout << "homming completed" << endl;
}

void OnProgramStopped(const PROGRAM_STOP_CAUSE v) {
}

void OnMonitoringDataCB(const LPMONITORING_DATA pData) {
}

void OnMonitoringDataExCB(const LPMONITORING_DATA_EX pData) {

}

void OnMonitoringCtrlIOCB(const LPMONITORING_CTRLIO pData) {
}

void OnMonitoringCtrlIOExCB(const LPMONITORING_CTRLIO_EX2 pData) {

}

void OnMonitoringStateCB(const ROBOT_STATE eState) {
  // std::cout << "monitoring state cb " << std::endl;
  // // Only do work that finishes within 50 msec.
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
  //       Drfl.SetSafeStopResetType(SAFE_STOP_RESET_TYPE_DEFAULT);
  //       Drfl.SetRobotControl(CONTROL_RESET_SAFET_STOP);
  //     }
  //     break;
  //   case STATE_SAFE_OFF:
  //     cout << "STATE_SAFE_OFF1" << endl;
  //     if (g_bHasControlAuthority) {
  //       cout << "STATE_SAFE_OFF2" << endl;
  //       std::cout << "servo on  : " <<Drfl.SetRobotControl(CONTROL_SERVO_ON) << std::endl;
  //       cout << "STATE_SAFE_OFF3" << endl;
  //     }
  //     break;
  //   case STATE_SAFE_STOP2:
  //     if (g_bHasControlAuthority)
  //       Drfl.SetRobotControl(CONTROL_RECOVERY_SAFE_STOP);
  //     break;
  //   case STATE_SAFE_OFF2:
  //     if (g_bHasControlAuthority) {
  //       Drfl.SetRobotControl(CONTROL_RECOVERY_SAFE_OFF);
  //     }
  //     break;
  //   case STATE_RECOVERY:
  //     // Drfl.SetRobotControl(CONTROL_RESET_RECOVERY);
  //     break;
  //   default:
  //     break;
  // }
  // return;
    if (eState == STATE_STANDBY) {
        is_standby = true;
        std::cout << "Successfully Servo On !!" << std::endl;
    } else {
        is_standby = false;
    }
}

void OnMonitroingAccessControlCB(const MONITORING_ACCESS_CONTROL transit) {
    std::cout << "[AccessControl] state = " << int(transit) << std::endl;

    switch (transit) {
        case MONITORING_ACCESS_CONTROL_GRANT:
            // State where control authority is granted
            g_bHasControlAuthority = true;
            break;

        case MONITORING_ACCESS_CONTROL_LOSS:
        case MONITORING_ACCESS_CONTROL_DENY:
            // State where control authority is lost or denied
            g_bHasControlAuthority = false;
            break;

        case MONITORING_ACCESS_CONTROL_REQUEST:
        case MONITORING_ACCESS_CONTROL_LAST:
        default:
            break;
    }

  }

void OnLogAlarm(LPLOG_ALARM tLog) {
  cout << "Alarm Info: "
       << "group(" << (unsigned int)tLog->_iGroup << "), index("
       << tLog->_iIndex << "), param(" << tLog->_szParam[0] << "), param("
       << tLog->_szParam[1] << "), param(" << tLog->_szParam[2] << ")" << endl;
}

void OnTpPopup(LPMESSAGE_POPUP tPopup) {
  cout << "Popup Message: " << tPopup->_szText << endl;
  cout << "Message Level: " << tPopup->_iLevel << endl;
  cout << "Button Type: " << tPopup->_iBtnType << endl;
}

void OnTpLog(const char* strLog) { cout << "Log Message: " << strLog << endl; }

void OnTpProgress(LPMESSAGE_PROGRESS tProgress) {
  cout << "Progress cnt : " << (int)tProgress->_iTotalCount << endl;
  cout << "Current cnt : " << (int)tProgress->_iCurrentCount << endl;
}

void OnTpGetuserInput(LPMESSAGE_INPUT tInput) {
  cout << "User Input : " << tInput->_szText << endl;
  cout << "Data Type : " << (int)tInput->_iType << endl;
}

void OnRTMonitoringData(LPRT_OUTPUT_DATA_LIST tData)
{

}

void OnDisConnected() {
  while (!Drfl.open_connection(IP_ADDR)) {
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  }
}


int main(int argc, char** argv) {

  Drfl.set_on_homming_completed(OnHommingCompleted);
  Drfl.set_on_monitoring_data(OnMonitoringDataCB);
  Drfl.set_on_monitoring_data_ex(OnMonitoringDataExCB);
  Drfl.set_on_monitoring_ctrl_io(OnMonitoringCtrlIOCB);
  //Drfl.set_on_monitoring_ctrl_io_ex(OnMonitoringCtrlIOExCB);
  Drfl.set_on_monitoring_state(OnMonitoringStateCB);
  Drfl.set_on_monitoring_access_control(OnMonitroingAccessControlCB);
  Drfl.set_on_tp_initializing_completed(OnTpInitializingCompleted);
  Drfl.set_on_log_alarm(OnLogAlarm);
  Drfl.set_on_tp_popup(OnTpPopup);
  Drfl.set_on_tp_log(OnTpLog);
  Drfl.set_on_tp_progress(OnTpProgress);
  Drfl.set_on_tp_get_user_input(OnTpGetuserInput);
  Drfl.set_on_rt_monitoring_data(OnRTMonitoringData);
  Drfl.set_on_program_stopped(OnProgramStopped);
  Drfl.set_on_disconnected(OnDisConnected);

  assert(Drfl.open_connection(IP_ADDR));

  Drfl.setup_monitoring_version(1); // (0 -> older version, 1 -> latest)

  SYSTEM_VERSION tSysVerion = {'\0',};
  Drfl.get_system_version(&tSysVerion);

  // 1) Retry requesting access + Servo ON up to 10 times
  for (size_t retry = 0; retry < 10;
      ++retry, std::this_thread::sleep_for(std::chrono::milliseconds(1000))) {

      if (!g_bHasControlAuthority) {
          // Force request for control authority
          Drfl.ManageAccessControl(MANAGE_ACCESS_CONTROL_FORCE_REQUEST);
          continue;
      }

      if (!is_standby) {
           // Try Servo ON
          Drfl.set_robot_control(CONTROL_SERVO_ON);
          continue;
      }

      if (g_bHasControlAuthority && is_standby)
          break;
  }

  // 2) State Check
  if (!(g_bHasControlAuthority && is_standby)) {
      std::cout << "Failed to reach GRANT + STANDBY state" << std::endl;
      Drfl.CloseConnection();
      return 1;
  }

  std::cout << "System version: " << tSysVerion._szController << std::endl;
  std::cout << "Library version: " << Drfl.get_library_version() << std::endl;

  std::cout << "set robot mode ret : " << Drfl.set_robot_mode(ROBOT_MODE_AUTONOMOUS) << std::endl;

  Drfl.set_robot_system(ROBOT_SYSTEM_REAL);

  while (true) {
    cout << "\ninput key : ";
    char ch;
    cin >> ch;

    if (ch == 'q') {
      std::cout << "Exit program..." << std::endl;
      break;   // exit while loop
    }

    switch (ch) {
      case '0':
      {
        // drl sample.
        Drfl.drl_start(ROBOT_SYSTEM_REAL, getWeldingSampleDrl());
      }
      break;
      case '1':
      {
        // Set ready pos and get Target Welding position
        float ready_weld_posj[6] = {85.8353, -53.5421, -61.888, 6.7839, -76.0373, -19.6588};
        Drfl.movej(ready_weld_posj, 50, 50);
        LPROBOT_TASK_POSE posx = Drfl.get_current_posx();
        float target_wel_posx[6];
        for(int i=0; i<NUM_JOINT; i++) {
          target_wel_posx[i] = posx->_fTargetPos[i];
        }
        target_wel_posx[0] += 200;

        // Refer to drl Sample Code.
        Drfl.set_singularity_handling(SINGULARITY_AVOIDANCE_AVOID);
        unsigned char dry_run = 1; // (flag_dry_run: real welding(0) / dry-run welding(1))
        unsigned char weld_con_adj = 1; // Whether to adjust welding condition (apply condition set by app_weld_set_weld_cond_analog(1) / apply adjustment values(0))

        float velx[2] = { 250.0, 80.625 };
        float accx[2] = { 1000., 322.5 };
        float g_op_speed = 100;
        Drfl.set_robot_mode(ROBOT_MODE_AUTONOMOUS);

        // Pre-configure TCP: Welding_Torch(0, 200, 300, 90, 67.5, 0) for EWM welder
        if (Drfl.get_tcp() != "Welding Torch")
          Drfl.set_tcp("Welding Torch");
        this_thread::sleep_for(200ms);
        // Drfl.change_operation_speed(g_op_speed);
        CONFIG_DIGITAL_WELDING_INTERFACE_PROCESS r2m_process_data = {
        // _bEnable / _nDataType / _nPositionalNumber / _fMinData / _fMaxData / _nByteOffset / _nBitOffset / _nComnDataType / _nMaxDigitSize
          {1, 0, 0, 0, 0, 0, 4, 0, 1}, // _tWeldingStart
          { 1, 0, 0, 0, 0, 0, 5, 0, 1}, // _tRobotReady
          {1, 0, 0, 0, 0, 1, 4, 0, 1} // _tErrorReset
        };
        Drfl.app_weld_set_interface_eip_r2m_process(r2m_process_data);
        CONFIG_DIGITAL_WELDING_INTERFACE_MODE r2m_mode_data = {
        // _bEnable / _nDataType / _nPositionalNumber / _fMinData / _fMaxData / _nByteOffset / _nBitOffset / _nComnDataType / _nMaxDigitSize
          {1, 1, 0, 0, 1, 0, 0, 2, 2}, // _tWeldingMode
          { 0, 0, 0, 0, 0, 0, 0, 0, 0}, // _t2T2TSpecial
          { 0, 0, 0, 0, 0, 0, 0, 0, 0 }, // _tPulseMode
          { 0, 0, 0, 0, 0, 0, 0, 0, 0 } // _tWMopt1
        };
        Drfl.app_weld_set_interface_eip_r2m_mode(r2m_mode_data);
        CONFIG_DIGITAL_WELDING_INTERFACE_TEST r2m_test_data = {
        // _bEnable / _nDataType / _nPositionalNumber / _fMinData / _fMaxData / _nByteOffset / _nBitOffset / _nComnDataType / _nMaxDigitSize
          {1, 0, 0, 0, 0, 0, 6, 0, 1}, // _tGasTest
          {1, 0, 0, 0, 0, 1, 0, 0, 1}, // _tInchingP
          {1, 0, 0, 0, 0, 1, 2, 0, 1}, // _tInchingM
          { 0, 0, 0, 0, 0, 0, 0, 0, 0 }, // _tBlowOutTorch
          { 1, 0, 0, 0, 0, 1, 7, 0, 1}, // _tSimulation
          { 0, 0, 0, 0, 0, 0, 0, 0, 0 }, // _tTSopt1
          { 0, 0, 0, 0, 0, 0, 0, 0, 0 } // _tTSopt2
        };
        Drfl.app_weld_set_interface_eip_r2m_test(r2m_test_data);
        CONFIG_DIGITAL_WELDING_INTERFACE_CONDITION r2m_condition_data = {
        // _bEnable / _nDataType / _nPositionalNumber / _fMinData / _fMaxData / _nByteOffset / _nBitOffset / _nComnDataType / _nMaxDigitSize
          { 1, 1, 0, 1, 255, 3, 0, 4, 8 }, // _tJobNumber
          { 1, 1, 0, 0, 15, 2, 0, 3, 4}, // _tSynegicID
          { 0, 0, 0, 0, 0, 0, 0, 0, 0 }, // _tWireFeedSpeed
          { 0, 0, 0, 0, 0, 0, 0, 0, 0 }, // _tArclengthCorrection
          { 0, 0, 0, 0, 0, 0, 0, 0, 0 } // _tDynamicCorrection
        };
        Drfl.app_weld_set_interface_eip_r2m_condition(r2m_condition_data);
        CONFIG_DIGITAL_WELDING_INTERFACE_OPTION r2m_option_data = {
        // _bEnable / _nDataType / _nPositionalNumber / _fMinData / _fMaxData / _nByteOffset / _nBitOffset / _nComnDataType / _nMaxDigitSize
          { 0, 0, 0, 0, 0, 0, 0, 0, 0 }, // _tOption1
          { 0, 0, 0, 0, 0, 0, 0, 0, 0 }, // _tOption2
          { 0, 0, 0, 0, 0, 0, 0, 0, 0 }, // _tOption3
          { 0, 0, 0, 0, 0, 0, 0, 0, 0 }, // _tOption4
          { 0, 0, 0, 0, 0, 0, 0, 0, 0 }, // _tOption5
          { 0, 0, 0, 0, 0, 0, 0, 0, 0 }, // _tOption6
          { 0, 0, 0, 0, 0, 0, 0, 0, 0 }, // _tOption7
          { 0, 0, 0, 0, 0, 0, 0, 0, 0 }, // _tOption8
          { 0, 0, 0, 0, 0, 0, 0, 0, 0 }, // _tOption9
          { 0, 0, 0, 0, 0, 0, 0, 0, 0 }, // _tOption10
          { 0, 0, 0, 0, 0, 0, 0, 0, 0 }, // _tOption11
          { 0, 0, 0, 0, 0, 0, 0, 0, 0 }, // _tOption12
          { 0, 0, 0, 0, 0, 0, 0, 0, 0 }, // _tOption13
          { 0, 0, 0, 0, 0, 0, 0, 0, 0 }, // _tOption14
          { 0, 0, 0, 0, 0, 0, 0, 0, 0 } // _tOption15
        };
        Drfl.app_weld_set_interface_eip_r2m_option(r2m_option_data);
        CONFIG_DIGITAL_WELDING_INTERFACE_PROCESS2 m2r_process2_data = {
        // _bEnable / _nDataType / _nPositionalNumber / _fMinData / _fMaxData / _nByteOffset / _nBitOffset / _nComnDataType / _nMaxDigitSize
          { 1, 0, 0, 0, 0, 0, 0, 0, 1 }, // _tCurrentFlow
          { 1, 0, 0, 0, 0, 0, 6, 0, 1}, // _tProcessActive
          { 1, 0, 0, 0, 0, 0, 5, 0, 1 }, // _tMainCurrent
          {1, 0, 0, 0, 0, 0, 1, 0, 1}, // _tMachineReady
          { 0, 0, 0, 0, 0, 0, 0, 0, 0 } // _tCommReady
        };
        Drfl.app_weld_set_interface_eip_m2r_process2(m2r_process2_data);

        CONFIG_DIGITAL_WELDING_INTERFACE_MONITORING m2r_monitoring_data = {
        // _bEnable / _nDataType / _nPositionalNumber / _fMinData / _fMaxData / _nByteOffset / _nBitOffset / _nComnDataType / _nMaxDigitSize
          { 1, 2, 1, 0.0, 100.0, 6, 0, 6, 16 }, // _tWeldingVoltage
          { 1, 2, 1, 0.0, 1000.0, 8, 0, 6, 16}, // _tWeldingCurrent
          { 1, 2, 1, 0.0, 40.0, 10, 0, 6, 16}, // _tWireFeedSpeed
          { 0, 0, 0, 0, 0, 0, 0, 0, 0 }, // _tWireStick
          { 1, 0, 0, 0, 0, 2, 0, 4, 1}, // _tError
          { 1, 0, 0, 0, 0, 1, 0, 4, 8} // _ZtErrorNumber
        };
        Drfl.app_weld_set_interface_eip_m2r_monitoring(m2r_monitoring_data);
        CONFIG_DIGITAL_WELDING_INTERFACE_OTHER m2r_other_data = {
        // _bEnable / _nDataType / _nPositionalNumber / _fMinData / _fMaxData / _nByteOffset / _nBitOffset / _nComnDataType / _nMaxDigitSize
          { 0, 0, 0, 0, 0, 0, 0, 0, 0 }, // _tOption1
          { 0, 0, 0, 0, 0, 0, 0, 0, 0 }, // _tOption2
          { 0, 0, 0, 0, 0, 0, 0, 0, 0 }, // _tOption3
          { 0, 0, 0, 0, 0, 0, 0, 0, 0 }, // _tOption4
          { 0, 0, 0, 0, 0, 0, 0, 0, 0 }, // _tOption5
          { 0, 0, 0, 0, 0, 0, 0, 0, 0 }, // _tOption6
          { 0, 0, 0, 0, 0, 0, 0, 0, 0 }, // _tOption7
          { 0, 0, 0, 0, 0, 0, 0, 0, 0 }, // _tOption8
          { 0, 0, 0, 0, 0, 0, 0, 0, 0 }, // _tOption9
          { 0, 0, 0, 0, 0, 0, 0, 0, 0 } // _tOption10
        };
        Drfl.app_weld_set_interface_eip_m2r_other(m2r_other_data);
        this_thread::sleep_for(500ms);
        Drfl.app_weld_enable_digital(1);
        this_thread::sleep_for(500ms);

        CONFIG_DIGITAL_WELDING_CONDITION weld_condition = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
        weld_condition._cVirtualWelding = dry_run;
        weld_condition._fTargetVel = 3.f;
        weld_condition._fMaxVelLimit = 100.f;
        weld_condition._nWeldingMode = 1;
        weld_condition._nPulseMode = 0;
        weld_condition._nJobNumber = 4;
        weld_condition._nSynergicID = 1;
        Drfl.app_weld_set_weld_cond_digital(weld_condition);

        // app_weld_set_weld_cond_digital(flag_dry_run=dry_run, vel_target=3.0000, vel_min=0.0000, vel_max=100.0000, welding_mode=1, s_2t=0, pulse_mode=0, wm_opt1=0, simulation=0, ts_opt1=0, ts_opt2=0, job_num=4, synergic_id=1, r_wire_feed_speed=0, voltage_correct=0, dynamic_correct=0, r_opt1=0, r_opt2=0, r_opt3=0, r_opt4=0, r_opt5=0, r_opt6=0, r_opt7=0, r_opt8=0, r_opt9=0, r_opt10=0, r_opt11=0, r_opt12=0, r_opt13=0, r_opt14=0, r_opt15=0)

        this_thread::sleep_for(1000ms);

        // Configure weaving condition (flag_dry_run: real welding(0) / dry-run welding(1))

        CONFIG_TRAPEZOID_WEAVING_SETTING weaving_trap = { //Weaving_trap 설정
          0.f,
          0.f,
          0.f,
          { 0.f, 3.f },
          { 0.f, -3.f },
          0.3f,
          0.1f,
          0.2f,
          0.3f,
          0.1f,
          0.2f
        };
        weaving_trap._fOffsetY = 0;
        weaving_trap._fOffsetZ = 0;
        weaving_trap._fGradient = 0;
        weaving_trap._fwPT1[0] = 0;
        weaving_trap._fwPT1[1] = 3;
        weaving_trap._fwPT2[0] = 0;
        weaving_trap._fwPT2[1] = -3;
        weaving_trap._fwT1 = 0.3;
        weaving_trap._fwTAcc1 = 0.1;
        weaving_trap._fwTTD1 = 0.2;
        weaving_trap._fwT2 = 0.3;
        weaving_trap._fwTAcc2 = 0.1;
        weaving_trap._fwTTD2 = 0.2;

        // wv_offset=[0.000,0.000], wv_ang=0.00, wv_param=[0.000,3.0, 0.000,-3.0, 0.30, 0.10,       0.20, 0.30, 0.10, 0.20]

        Drfl.app_weld_weave_cond_trapezoidal(weaving_trap);

        // Ready pose already setted.
        // float ready_weld_pos[6] = { -77.41, -640.26, 119.09, 54.84, 166.79, -48.59 }; // specified posx in m1509
        // // Drfl.movel(ready_weld_pos, velx, accx);
        // // this_thread::sleep_for(1000ms);
        
        float welding_velx[2] = { 3,3 };
        float welding_accx[2] = { 70,70 };
        Drfl.amovel(target_wel_posx, welding_velx, welding_accx, 0, MOVE_MODE_ABSOLUTE, MOVE_REFERENCE_BASE, BLENDING_SPEED_TYPE_DUPLICATE, DR_MV_APP_WELD);
        Drfl.mwait();
        break;
      }
      case '2':{
        

      }
      break;
      default:
        break;
    }
  }

  Drfl.CloseConnection();
  return 0;
}


std::string getWeldingSampleDrl() {
  return R"(
set_singular_handling(DR_AVOID)
set_velj(60.0)
set_accj(100.0)
set_velx(250.0, 80.625)
set_accx(1000.0, 322.5)

dry_run = 1 
weld_con_adj = 1 
arc_sensing_test = False 


app_weld_set_interface_eip_r2m_process(welding_start=[1,0,0,0,4,0,1,0,0], robot_ready=[1,0,0,0,5,0,1,0,0], error_reset=[1,0,0,1,4,0,1,0,0])
app_weld_set_interface_eip_r2m_mode(welding_mode=[1,1,0,0,0,2,2,0,1], s_2t=[0,0,0,0,0,0,0,0,0], pulse_mode=[0,0,0,0,0,0,0,0,0],wm_opt1=[0,0,0,0,0,0,0,0,0])
app_weld_set_interface_eip_r2m_test(gas_test=[1,0,0,0,6,0,1,0,0], inching_plus=[1,0,0,1,0,0,1,0,0], inching_minus=[1,0,0,1,2,0,1,0,0], blow_out_torch=[0,0,0,0,0,0,0,0,0], simulation=[1,0,0,1,7,0,1,0,0], ts_opt1=[0,0,0,0,0,0,0,0,0], ts_opt2=[0,0,0,0,0,0,0,0,0])
app_weld_set_interface_eip_r2m_condition(job_num=[1,1,0,3,0,4,8,1,255], synergic_id=[1,1,0,2,0,3,4,0,15], r_wire_feed_speed=[0,0,0,0,0,0,0,0,0], dynamic_correct=[0,0,0,0,0,0,0,0,0])
app_weld_set_interface_eip_r2m_option(opt1=[0,0,0,0,0,0,0,0,0], opt2=[0,0,0,0,0,0,0,0,0], opt3=[0,0,0,0,0,0,0,0,0], opt4=[0,0,0,0,0,0,0,0,0], opt5=[0,0,0,0,0,0,0,0,0], opt6=[0,0,0,0,0,0,0,0,0], opt7=[0,0,0,0,0,0,0,0,0], opt8=[0,0,0,0,0,0,0,0,0], opt9=[0,0,0,0,0,0,0,0,0], opt10=[0,0,0,0,0,0,0,0,0], opt11=[0,0,0,0,0,0,0,0,0], opt12=[0,0,0,0,0,0,0,0,0], opt13=[0,0,0,0,0,0,0,0,0], opt14=[0,0,0,0,0,0,0,0,0], opt15=[0,0,0,0,0,0,0,0,0])
app_weld_set_interface_eip_m2r_process(current_flow=[1,0,0,0,0,0,1,0,0], process_active=[1,0,0,0,6,0,1,0,0], main_current=[1,0,0,0,5,0,1,0,0], machine_ready=[1,0,0,0,1,0,1,0,0], comm_ready=[0,0,0,0,0,0,0,0,0])
app_weld_set_interface_eip_m2r_monitoring(welding_voltage=[1,2,1,6,0,6,16,0.0,100.0], welding_current=[1,2,1,8,0,6,16,0.0,1000.0], wire_feed_speed=[1,2,1,10,0,6,16,0.0,40.0], wire_stick=[0,0,0,0,0,0,0,0,0], error=[1,0,0,2,0,4,1,0,0], error_num=[1,0,0,1,0,4,8,0,0])
app_weld_set_interface_eip_m2r_other(opt1=[0,0,0,0,0,0,0,0,0], opt2=[0,0,0,0,0,0,0,0,0], opt3=[0,0,0,0,0,0,0,0,0], opt4=[0,0,0,0,0,0,0,0,0], opt5=[0,0,0,0,0,0,0,0,0], opt6=[0,0,0,0,0,0,0,0,0], opt7=[0,0,0,0,0,0,0,0,0], opt8=[0,0,0,0,0,0,0,0,0], opt9=[0,0,0,0,0,0,0,0,0], opt10=[0,0,0,0,0,0,0,0,0])
wait(0.5)

app_weld_enable_digital()
wait(1)


app_weld_set_weld_cond_digital(flag_dry_run=dry_run, vel_target=3.0000, vel_min=0.1000, vel_max=100.0000, welding_mode=1, s_2t=0, pulse_mode=0, wm_opt1=0, simulation=0, ts_opt1=0, ts_opt2=0, job_num=4, synergic_id=1, r_wire_feed_speed=0, voltage_correct=0, dynamic_correct=0, r_opt1=0, r_opt2=0, r_opt3=0, r_opt4=0, r_opt5=0, r_opt6=0, r_opt7=0, r_opt8=0, r_opt9=0, r_opt10=0, r_opt11=0, r_opt12=0, r_opt13=0, r_opt14=0, r_opt15=0)

app_weld_weave_cond_trapezoidal(wv_offset=[0.000,0.000], wv_ang=0.00, wv_param=[0.000,3.0,0.000,-3.0,0.30,0.10,0.20,0.30,0.10,0.20])

ready_weld_pos = posx(-77.41, -640.26, 119.09, 54.84, 166.79, -48.59 )
movel(ready_weld_pos, radius=0.00, ref=0, mod=DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE, app_type=DR_MV_APP_NONE)
wait(1)
#if weld_con_adj == 1:
# app_weld_adj_welding_cond_digital(flag_reset=weld_con_adj)

#elif weld_con_adj == 0:
# app_weld_adj_welding_cond_digital(flag_reset=weld_con_adj, wv_width_ratio=0.5)

end_weld_pos = posx(306.16,-617.1,173.77,54.18,166.75,-49.15)
amovel(end_weld_pos, v=3.0000, a=70.0000, ref=0, mod=DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE, app_type=DR_MV_APP_WELD)
tp_log("Start welding motion")
mwait(1)
app_weld_disable_digital()

wait(1))";
};


