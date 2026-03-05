#ifdef __XENO__
#include <rtdk.h>
#include <native/task.h>
#include <native/timer.h>
#else
//#include "stdafx.h"
//#include <Windows.h>
//#include <process.h>
//#include <conio.h>
#endif // __XENO__
//#include <stdio.h>
#include <iostream>
#include <cstring>
#include <thread>
#include <chrono>
#include <termios.h>
#include <unistd.h>


#include "../../include/DRFLEx.h"
using namespace DRAFramework;

#undef NDEBUG
#include <assert.h>

CDRFLEx Drfl;
bool g_bHasControlAuthority = FALSE;
bool g_TpInitailizingComplted = FALSE;
bool g_mStat = FALSE;
bool g_Stop = FALSE;
bool moving = FALSE;
string strDrl =
    "\r\n\
loop = 0\r\n\
while loop < 1003:\r\n\
 movej(posj(10,10.10,10,10.10), vel=60, acc=60)\r\n\
 movej(posj(00,00.00,00,00.00), vel=60, acc=60)\r\n\
 loop+=1\r\n";

bool bAlterFlag = FALSE;

int linux_kbhit(void)
{
	struct termios oldt, newt;
	int ch;
	tcgetattr( STDIN_FILENO, &oldt );
	newt = oldt;

	newt.c_lflag &= ~( ICANON | ECHO );
	tcsetattr( STDIN_FILENO, TCSANOW, &newt );


	ch = getchar();

	tcsetattr( STDIN_FILENO, TCSANOW, &oldt );
	return ch;
}

int getch()
{
    int c;
    struct termios oldattr, newattr;

    tcgetattr(STDIN_FILENO, &oldattr);           // Save current terminal settings
    newattr = oldattr;
    newattr.c_lflag &= ~(ICANON | ECHO);         // Disable canonical mode and echo
    newattr.c_cc[VMIN] = 1;                      // Minimum number of bytes for read
    newattr.c_cc[VTIME] = 0;                     // Read timeout (deciseconds)
    tcsetattr(STDIN_FILENO, TCSANOW, &newattr);  // Apply terminal settings
    c = getchar();                               // Read one character
    tcsetattr(STDIN_FILENO, TCSANOW, &oldattr);  // Restore terminal settings
    return c;
}

void OnTpInitializingCompleted() {
  // Request control authority after TP initialization.
  g_TpInitailizingComplted = TRUE;
  Drfl.ManageAccessControl(MANAGE_ACCESS_CONTROL_FORCE_REQUEST);
}

void OnHommingCompleted() {
  // Do only work under 50 msec.
  cout << "homming completed" << endl;
}

void OnProgramStopped(const PROGRAM_STOP_CAUSE) {
  Drfl.drl_stop();
  // Do only work under 50 msec.
  // assert(Drfl.SetRobotMode(ROBOT_MODE_MANUAL));
  cout << "program stopped" << endl;
}

void OnMonitoringDataCB(const LPMONITORING_DATA pData) {
  // Do only work under 50 msec.

  return;
  cout << "# monitoring 0 data " << pData->_tCtrl._tTask._fActualPos[0][0]
       << pData->_tCtrl._tTask._fActualPos[0][1]
       << pData->_tCtrl._tTask._fActualPos[0][2]
       << pData->_tCtrl._tTask._fActualPos[0][3]
       << pData->_tCtrl._tTask._fActualPos[0][4]
       << pData->_tCtrl._tTask._fActualPos[0][5] << endl;
}

void OnMonitoringDataExCB(const LPMONITORING_DATA_EX pData) {
  return;
  cout << "# monitoring 1 data " << pData->_tCtrl._tWorld._fTargetPos[0]
       << pData->_tCtrl._tWorld._fTargetPos[1]
       << pData->_tCtrl._tWorld._fTargetPos[2]
       << pData->_tCtrl._tWorld._fTargetPos[3]
       << pData->_tCtrl._tWorld._fTargetPos[4]
       << pData->_tCtrl._tWorld._fTargetPos[5] << endl;
}

void OnMonitoringCtrlIOCB(const LPMONITORING_CTRLIO pData) {
  return;
  cout << "# monitoring ctrl 0 data" << endl;
  for (int i = 0; i < 16; i++) {
    cout << (int)pData->_tInput._iActualDI[i] << endl;
  }
}

void OnMonitoringCtrlIOExCB(const LPMONITORING_CTRLIO_EX2 pData) {
  return;
  cout << "# monitoring ctrl 1 data" << endl;
  for (int i = 0; i < 16; i++) {
    cout << (int)pData->_tInput._iActualDI[i] << endl;
  }
  for (int i = 0; i < 16; i++) {
    cout << (int)pData->_tOutput._iTargetDO[i] << endl;
  }
}

void OnMonitoringStateCB(const ROBOT_STATE eState) {
  // Do only work under 50 msec.
  // WARNING: Do NOT call blocking APIs (SetRobotControl, etc.) from this callback.
  // This callback runs on the library's run() thread. Blocking APIs wait for responses
  // processed by the same thread, causing a deadlock.
  // State recovery is handled in the main thread (e.g., key '4' while loop).
  cout << "\n>> State changed: " << (int)eState << endl;
}

void OnMonitroingAccessControlCB(
    const MONITORING_ACCESS_CONTROL eTrasnsitControl) {
  // Do only work under 50 msec.

  switch (eTrasnsitControl) {
    case MONITORING_ACCESS_CONTROL_REQUEST:
      assert(Drfl.ManageAccessControl(MANAGE_ACCESS_CONTROL_RESPONSE_YES));
      // Drfl.ManageAccessControl(MANAGE_ACCESS_CONTROL_RESPONSE_YES);
      break;
    case MONITORING_ACCESS_CONTROL_GRANT:
      g_bHasControlAuthority = TRUE;
      // cout << "GRANT1" << endl;
      // cout << "MONITORINGCB : " << (int)Drfl.GetRobotState() << endl;
      OnMonitoringStateCB(Drfl.GetRobotState());
      // cout << "GRANT2" << endl;
      break;
    case MONITORING_ACCESS_CONTROL_DENY:
    case MONITORING_ACCESS_CONTROL_LOSS:
      g_bHasControlAuthority = FALSE;
      if (g_TpInitailizingComplted) {
        // assert(Drfl.ManageAccessControl(MANAGE_ACCESS_CONTROL_REQUEST));
        Drfl.ManageAccessControl(MANAGE_ACCESS_CONTROL_FORCE_REQUEST);
      }
      break;
    default:
      break;
  }
}

void OnLogAlarm(LPLOG_ALARM tLog) {
  g_mStat = true;
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
//    static int td = 0;
//    if (td++ == 1000) {
//    	td = 0;
//    	printf("timestamp : %.3f\n", tData->time_stamp);
//    	printf("joint : %f %f %f %f %f %f\n", tData->actual_joint_position[0], tData->actual_joint_position[1], tData->actual_joint_position[2], tData->actual_joint_position[3], tData->actual_joint_position[4], tData->actual_joint_position[5]);
//		printf("q = %7.4f, %7.4f, %7.4f, %7.4f, %7.4f, %7.4f\n",
//				tData->actual_joint_position[0], tData->actual_joint_position[1], tData->actual_joint_position[2],
//				tData->actual_joint_position[3], tData->actual_joint_position[4], tData->actual_joint_position[5]);
//		printf("q_dot = %7.4f, %7.4f, %7.4f, %7.4f, %7.4f, %7.4f\n",
//				tData->actual_joint_velocity[0], tData->actual_joint_velocity[1], tData->actual_joint_velocity[2],
//				tData->actual_joint_velocity[3], tData->actual_joint_velocity[4], tData->actual_joint_velocity[5]);
//		printf("trq_g = %7.4f, %7.4f, %7.4f, %7.4f, %7.4f, %7.4f\n",
//				tData->gravity_torque[0], tData->gravity_torque[1], tData->gravity_torque[2],
//				tData->gravity_torque[3], tData->gravity_torque[4], tData->gravity_torque[5]);
//    }
}


uint32_t ThreadFunc(void* arg) {
	printf("start ThreadFunc\n");

	while (true) {
		if(linux_kbhit()){
			char ch = getch();
			switch (ch) {
				case 's': {
					printf("Stop!\n");
					g_Stop = true;
					Drfl.MoveStop(STOP_TYPE_SLOW);
				} break;
				case 'p': {
					printf("Pause!\n");
					Drfl.MovePause();
				} break;
				case 'r': {
					printf("Resume!\n");
					Drfl.MoveResume();
				} break;
			}
		}

		//Sleep(100);
		std::this_thread::sleep_for(std::chrono::milliseconds(100));
	}
	std::cout << "exit ThreadFunc" << std::endl;

	return 0;
}

void OnDisConnected() {
  while (!Drfl.open_connection("127.0.0.1")) {
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  }
}

struct PlanParam
{
	float time;

	float ps[6];
	float vs[6];
	float as[6];
	float pf[6];
	float vf[6];
	float af[6];

	float A0[6];
	float A1[6];
	float A2[6];
	float A3[6];
	float A4[6];
	float A5[6];
};

struct TraParam
{
	float time;

	float pos[6];
	float vel[6];
	float acc[6];
};

void TrajectoryPlan(PlanParam* plan)
{
    float ps[6],vs[6],as[6];
    float pf[6],vf[6],af[6];
    float tf;

	tf = plan->time;

    for(int i=0; i<6; i++)
    {
        ps[i] = plan->ps[i];
        vs[i] = plan->vs[i];
        as[i] = plan->as[i];
        pf[i] = plan->pf[i];
        vf[i] = plan->vf[i];
        af[i] = plan->af[i];
    }

    for(int i=0; i<6; i++)
    {
        plan->A0[i] = ps[i];
        plan->A1[i] = vs[i];
        plan->A2[i] = as[i]/2;
        plan->A3[i] = (20*pf[i]-20*ps[i]-(8*vf[i]+12*vs[i])*tf-(3*as[i]-af[i])*tf*tf)/(2*tf*tf*tf);
        plan->A4[i] = (30*ps[i]-30*pf[i]+(14*vf[i]+16*vs[i])*tf+(3*as[i]-2*af[i])*tf*tf)/(2*tf*tf*tf*tf);
        plan->A5[i] = (12*pf[i]-12*ps[i]-(6*vf[i]+6*vs[i])*tf-(as[i]-af[i])*tf*tf)/(2*tf*tf*tf*tf*tf);
    }
}

void TrajectoryGenerator(PlanParam *plan, TraParam *tra)
{
    double A0[6],A1[6],A2[6],A3[6],A4[6],A5[6];
	double t = tra->time;

    for(int i=0; i<6; i++)
    {
        A0[i] = plan->A0[i];
        A1[i] = plan->A1[i];
        A2[i] = plan->A2[i];
        A3[i] = plan->A3[i];
        A4[i] = plan->A4[i];
        A5[i] = plan->A5[i];
    }

    for(int i=0; i<6; i++)
    {
        tra->pos[i] = A0[i] + A1[i]*t + A2[i]*t*t + A3[i]*t*t*t + A4[i]*t*t*t*t + A5[i]*t*t*t*t*t;
        tra->vel[i] = A1[i] + 2*A2[i]*t + 3*A3[i]*t*t + 4*A4[i]*t*t*t + 5*A5[i]*t*t*t*t;
        tra->acc[i] = 2*A2[i] + 6*A3[i]*t + 12*A4[i]*t*t + 20*A5[i]*t*t*t;
    }
}

int main(int argc, char** argv) {
  // Drfl.ConfigCreateModbus("mr1", "192.168.137.70", 552,
  // MODBUS_REGISTER_TYPE_HOLDING_REGISTER, 3, 5);

  typedef enum {
    EXAMPLE_JOG,
    EXAMPLE_HOME,
    EXAMPLE_MOVEJ_ASYNC,
    EXAMPLE_MOVEL_SYNC,
    EXAMPLE_MOVEJ_SYNC,
    EXAMPLE_DRL_PROGRAM,
    EXAMPLE_GPIO,
    EXAMPLE_MODBUS,
    EXAMPLE_LAST,
    EXAMPLE_SERVO_OFF
  } EXAMPLE;

  EXAMPLE eExample = EXAMPLE_LAST;

  bool bLoop = TRUE;
  while (bLoop) {
    g_mStat = false;
    g_Stop = false;
#ifdef __XENO__
    unsigned long overrun = 0;
    const double tick = 1000000;  // 1ms
    rt_task_set_periodic(nullptr, TM_NOW, tick);
    if (rt_task_wait_period(&overrun) == -ETIMEDOUT) {
      std::cout << __func__ << ": \x1B[37m\x1B[41mover-runs: " << overrun
                << "\x1B[0m\x1B[0K" << std::endl;
    }
#else
    std::this_thread::sleep_for(std::chrono::microseconds(1000));
#endif  // __XENO__
#if 0
        static char ch = '0';
        if (ch == '7') ch = '0';
        else if (ch == '0') ch = '7';
#else
    cout << "\n\033[1;33minput key : \033[0m";
    // char ch = _getch();
    char ch;
    cin >> ch;
    // cout << ch << endl;
#endif
    switch (ch) {
      case 'q':
        bLoop = FALSE;
        Drfl.close_connection();
        std::cout << "Close Connection !! " << std::endl;
        break;
      case '1':
          {
              // Register callbacks
              Drfl.set_on_monitoring_data_ex(OnMonitoringDataExCB);
              Drfl.set_on_monitoring_ctrl_io_ex(OnMonitoringCtrlIOExCB);
              Drfl.set_on_log_alarm(OnLogAlarm);
              // Drfl.set_on_disconnected(OnDisConnected);
              Drfl.set_on_monitoring_state(OnMonitoringStateCB);
              // Drfl.set_on_tp_initializing_completed(OnTpInitializingCompleted);
              // Drfl.set_on_tp_popup(OnTpPopup);
              // Drfl.set_on_tp_log(OnTpLog);
              // Drfl.set_on_tp_progress(OnTpProgress);
              // Drfl.set_on_tp_get_user_input(OnTpGetuserInput);
              // Drfl.set_on_rt_monitoring_data(OnRTMonitoringData);
              std::cout << "Register Callbacks !! " << std::endl;
          }
          break;
      case '2':
          {
              // Establish connection
              assert(Drfl.open_connection("127.0.0.1"));
              std::cout << "Open Connection !! " << std::endl;
          }
          break;
      case '3':
      {
          //Acquire control authority - register callback BEFORE sending request
        Drfl.set_on_monitoring_access_control([](const MONITORING_ACCESS_CONTROL eTrasnsitControl)
          {
            if(MONITORING_ACCESS_CONTROL_GRANT == eTrasnsitControl) {
              g_bHasControlAuthority = TRUE;
              std::cout << "\nSuccessfully got Control Authority !! " << std::endl;
            }else {
              std::cout << "Unintended autority.. " << std::endl;
            }
          }
        );

        // FORCE_REQUEST goes through server's async message queue (SendMessageToRC),
        // which can be delayed if the server is processing previous disconnect events.
        // Retry once if the first attempt doesn't get GRANT within 5 seconds.
        g_bHasControlAuthority = FALSE;
        Drfl.ManageAccessControl(MANAGE_ACCESS_CONTROL_FORCE_REQUEST);
        std::cout << "Try to get Control Authority !!" << std::endl;

        for (int i = 0; i < 50; i++) {
            if (g_bHasControlAuthority) break;
            this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        if (!g_bHasControlAuthority) {
            // Retry once - server may have been busy processing previous disconnect
            Drfl.ManageAccessControl(MANAGE_ACCESS_CONTROL_FORCE_REQUEST);
            std::cout << "Retrying..." << std::endl;
            for (int i = 0; i < 50; i++) {
                if (g_bHasControlAuthority) break;
                this_thread::sleep_for(std::chrono::milliseconds(100));
            }
        }
        if (!g_bHasControlAuthority) {
            std::cout << "Failed to get Control Authority." << std::endl;
        }
      }
      break;
      case '4':
		  {
          if (!g_bHasControlAuthority) {
              std::cout << "No control authority. Run step 3 first." << std::endl;
              break;
          }
          // Turn servo on and set monitoring version
          Drfl.setup_monitoring_version(1);
          Drfl.set_digital_output(GPIO_CTRLBOX_DIGITAL_INDEX_10, TRUE);
          SYSTEM_VERSION tSysVerion = {'\0',};
          Drfl.get_system_version(&tSysVerion);
          cout << "System version: " << tSysVerion._szController << endl;
          cout << "Library version: " << Drfl.get_library_version() << endl;

          Drfl.set_robot_control(CONTROL_SERVO_ON);

          int nRetryCount = 0;
          ROBOT_STATE ePrevState = STATE_LAST;
          int nSameStateCount = 0;
          const int RETRY_INTERVAL = 10; // retry every 10*500ms = 5 seconds
          while ((Drfl.get_robot_state() != STATE_STANDBY) || !g_bHasControlAuthority) {
              ROBOT_STATE eState = Drfl.get_robot_state();
              bool bSendCommand = false;
              if (eState != ePrevState) {
                  ePrevState = eState;
                  nSameStateCount = 0;
                  bSendCommand = true;
              } else if (++nSameStateCount >= RETRY_INTERVAL) {
                  nSameStateCount = 0;
                  bSendCommand = true;
              }
              if (bSendCommand) {
                  switch ((unsigned char)eState) {
                    case STATE_SAFE_STOP:
                      cout << ">> Recovery: SAFE_STOP -> resetting..." << endl;
                      Drfl.SetSafeStopResetType(SAFE_STOP_RESET_TYPE_DEFAULT);
                      Drfl.SetRobotControl(CONTROL_RESET_SAFET_STOP);
                      break;
                    case STATE_SAFE_OFF:
                      cout << ">> Recovery: SAFE_OFF -> servo on..." << endl;
                      Drfl.SetRobotControl(CONTROL_SERVO_ON);
                      break;
                    case STATE_SAFE_STOP2:
                      cout << ">> Recovery: SAFE_STOP2 -> recovering..." << endl;
                      Drfl.SetRobotControl(CONTROL_RECOVERY_SAFE_STOP);
                      break;
                    case STATE_SAFE_OFF2:
                      cout << ">> Recovery: SAFE_OFF2 -> recovering..." << endl;
                      Drfl.SetRobotControl(CONTROL_RECOVERY_SAFE_OFF);
                      break;
                    case STATE_NOT_READY:
                      cout << ">> Recovery: NOT_READY -> servo on..." << endl;
                      Drfl.SetRobotControl(CONTROL_SERVO_ON);
                      break;
                    default:
                      cout << ">> Unknown state: " << (int)eState << endl;
                      break;
                  }
              }
              this_thread::sleep_for(std::chrono::milliseconds(500));
              if (++nRetryCount > 20) { // 10 second timeout
                  cout << ">> Timeout waiting for STANDBY (state=" << (int)eState << ")" << endl;
                  break;
              }
          }
          if (Drfl.get_robot_state() == STATE_STANDBY && g_bHasControlAuthority)
              std::cout << "Servo On !! " << std::endl;
          else
              std::cout << "Failed to reach STANDBY state." << std::endl;
		  }
      break;
      case '5':
      {
        //Set robot mode and system
        assert(Drfl.set_robot_mode(ROBOT_MODE_AUTONOMOUS));
        assert(Drfl.set_robot_system(ROBOT_SYSTEM_REAL));
        std::cout << "Set Auto Mode !!" << std::endl;
      }

      break;
      case '6':
      {
        //Execute motion
        std::cout << "Move Start !!  " << std::endl;
        float p1[6] = {0, 0, -90, 0, 90, 0};
        Drfl.movej(p1, 60, 30);  // Move to posj(0,0,10,0,10,0) with v=60, a=30
        // float p2[6] = {0, 0, 9, 0, 90, 0};
        // Drfl.movej(p2, 30, 30); // Move back to posj(0,0,0,0,0,0) with v=30, a=30
        //You cannot specify only selected parameters; even when using time, you must provide velocity and acceleration first.
        std::cout << "Move End !!  " << std::endl;	
        LPROBOT_POSE pCur = Drfl.get_current_pose();
        std::cout << " Current Pose : "
                  << pCur->_fPosition[0] << ", "
                  << pCur->_fPosition[1] << ", "
                  << pCur->_fPosition[2] << ", "
                  << pCur->_fPosition[3] << ", "
                  << pCur->_fPosition[4] << ", "
                  << pCur->_fPosition[5] << std::endl;
      } break;
      case '7': 
		  {
        // Motion using kinematics functions
        float x1[6] = {370.9, 719.7, 651.5, 90, -180, 0}; //Task-space pose for ikin conversion
        float fIterThreshold[2] = {0.005, 0.01};
        LPROBOT_POSE res = Drfl.ikin(x1, 255); //ROBOT_POSE holds float[6] elements.
        //Store the ikin result (shallow copy)
        float q1[6] = {0,};
        for(int i=0; i<6; i++){
            q1[i] = res->_fPosition[i]; //Read elements from LPROBOT_POSE (return value)
        }
        std::cout << " Get ikin Success (Task Space -> Joint Space) !! " << std::endl;
        std::cout << " Joint Values [q1]: "
                  << q1[0] << ", "
                  << q1[1] << ", "
                  << q1[2] << ", "
                  << q1[3] << ", "
                  << q1[4] << ", "
                  << q1[5] << std::endl;
        this_thread::sleep_for(std::chrono::milliseconds(100));

        LPINVERSE_KINEMATIC_RESPONSE res2 = Drfl.ikin_norm(x1, 255, COORDINATE_SYSTEM_BASE, 0); //ROBOT_POSE holds float[6] elements.
        //Store the ikin_norm result (shallow copy)
        float q2[6] = {0,};
        for(int i=0; i<6; i++){
            q2[i] = res2->_fTargetPos[i]; //Read elements from LPROBOT_POSE (return value)
        }
        std::cout << " Get ikin_norm Success (Task Space -> Joint Space) !! " << std::endl;
        std::cout << " Joint Values [q2]: "
                  << q2[0] << ", "
                  << q2[1] << ", "
                  << q2[2] << ", "
                  << q2[3] << ", "
                  << q2[4] << ", "
                  << q2[5] << std::endl;
        // Drfl.movej(q1, 60, 30);
		  }
		  break;
      case '8': 
		  {
        // DRL
        string strDrl = "movej([0, 0, 10, 0, 10, 0], 60, 30)\nmovej([10, 20, 30, 40, 50, 60], 60, 30)\n";
        //DRL script to move between two positions using movej
        Drfl.drl_start(ROBOT_SYSTEM_REAL, strDrl); //Set ROBOT_SYSTEM to VIRTUAL for a virtual robot, and pass the DRL script
        Drfl.set_on_program_stopped(OnProgramStopped);
        std::cout << "Sent DRL Script !!" << std::endl;
		  }
		  break;
      case '9': 
		  {
        LPROBOT_POSE pCur = Drfl.get_current_pose();
        std::cout << " Current Pose : "
                  << pCur->_fPosition[0] << ", "
                  << pCur->_fPosition[1] << ", "
                  << pCur->_fPosition[2] << ", "
                  << pCur->_fPosition[3] << ", "
                  << pCur->_fPosition[4] << ", "
                  << pCur->_fPosition[5] << std::endl;
		  }
		  break;
      default:
        break;
    }
    // Sleep(100);
    this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  Drfl.CloseConnection();

#ifdef __XENO__
  rt_task_join(&sub_task);
#endif // __XENO__

  return 0;
}
