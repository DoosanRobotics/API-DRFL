// DRFL Realtime Command Example
// Doosan Robotics Framework Library Real-time Control Example

#ifdef __XENO__
#include <rtdk.h>
#include <native/task.h>
#include <native/timer.h>
#endif

#include <stdio.h>
#include <iostream>
#include <cstring>
#include <thread>
#include <chrono>
#include <termios.h>
#include <unistd.h>
#include <iomanip>
#include <pthread.h>
#include <mutex>
#include <cmath>
#include <vector>
#include <fstream>
#include <sched.h>
#include <csignal>
#include <regex>
#include <assert.h>

#include "../../include/DRFLEx.h"
using namespace DRAFramework;

// ============================================================================
// Global Variables
// ============================================================================
CDRFLEx Drfl;
bool g_bHasControlAuthority = false;
bool g_TpInitailizingComplted = false;
bool g_mStat = false;
bool g_Stop = false;
bool moving = false;
bool bAlterFlag = false;
float k_ratio = 1.5;

string strDrl = 
    "\r\n\
loop = 0\r\n\
while loop < 1003:\r\n\
 movej(posj(10,10.10,10,10.10), vel=60, acc=60)\r\n\
 movej(posj(00,00.00,00,00.00), vel=60, acc=60)\r\n\
 loop+=1\r\n";

// ============================================================================
// Structures
// ============================================================================
struct PlanParam {
    float time;
    float ps[6], vs[6], as[6];
    float pf[6], vf[6], af[6];
    float A0[6], A1[6], A2[6], A3[6], A4[6], A5[6];
};

struct TraParam {
    float time;
    float pos[6], vel[6], acc[6];
};

struct LogData {
    std::chrono::steady_clock::time_point timestamp;
    std::string message;
};

// ============================================================================
// Utility Functions
// ============================================================================
int linux_kbhit(void) {
    struct termios oldt, newt;
    int ch;
    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);
    ch = getchar();
    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    return ch;
}

int getch() {
    int c;
    struct termios oldattr, newattr;
    tcgetattr(STDIN_FILENO, &oldattr);
    newattr = oldattr;
    newattr.c_lflag &= ~(ICANON | ECHO);
    newattr.c_cc[VMIN] = 1;
    newattr.c_cc[VTIME] = 0;
    tcsetattr(STDIN_FILENO, TCSANOW, &newattr);
    c = getchar();
    tcsetattr(STDIN_FILENO, TCSANOW, &oldattr);
    return c;
}

// ============================================================================
// Logging Functions
// ============================================================================
std::mutex log_mutex;
std::ofstream logfile("Doosanrobotics_realtime_api_logs_011425.txt", std::ios::app);

void log_timestamp(const std::string& message) {
    std::lock_guard<std::mutex> lock(log_mutex);
    auto now = std::chrono::steady_clock::now();
    auto time_since_epoch = std::chrono::duration_cast<std::chrono::microseconds>(now.time_since_epoch());
    logfile << time_since_epoch.count() << " us: " << message << std::endl;
    logfile.flush();
}

void handle_signal(int signal) {
    if (signal == SIGINT || signal == SIGTERM) {
        std::lock_guard<std::mutex> lock(log_mutex);
        logfile << "Program interrupted by signal " << signal << std::endl;
        logfile.close();
        exit(0);
    }
}

// ============================================================================
// Trajectory Planning Functions
// ============================================================================
void TrajectoryPlan(PlanParam* plan) {
    float ps[6], vs[6], as[6];
    float pf[6], vf[6], af[6];
    float tf = plan->time;

    for (int i = 0; i < 6; i++) {
        ps[i] = plan->ps[i];
        vs[i] = plan->vs[i];
        as[i] = plan->as[i];
        pf[i] = plan->pf[i];
        vf[i] = plan->vf[i];
        af[i] = plan->af[i];
    }

    for (int i = 0; i < 6; i++) {
        plan->A0[i] = ps[i];
        plan->A1[i] = vs[i];
        plan->A2[i] = as[i] / 2;
        plan->A3[i] = (20*pf[i] - 20*ps[i] - (8*vf[i] + 12*vs[i])*tf - (3*as[i] - af[i])*tf*tf) / (2*tf*tf*tf);
        plan->A4[i] = (30*ps[i] - 30*pf[i] + (14*vf[i] + 16*vs[i])*tf + (3*as[i] - 2*af[i])*tf*tf) / (2*tf*tf*tf*tf);
        plan->A5[i] = (12*pf[i] - 12*ps[i] - (6*vf[i] + 6*vs[i])*tf - (as[i] - af[i])*tf*tf) / (2*tf*tf*tf*tf*tf);
    }
}

void TrajectoryGenerator(PlanParam *plan, TraParam *tra) {
    double A0[6], A1[6], A2[6], A3[6], A4[6], A5[6];
    double t = tra->time;

    for (int i = 0; i < 6; i++) {
        A0[i] = plan->A0[i];
        A1[i] = plan->A1[i];
        A2[i] = plan->A2[i];
        A3[i] = plan->A3[i];
        A4[i] = plan->A4[i];
        A5[i] = plan->A5[i];
    }

    for (int i = 0; i < 6; i++) {
        tra->pos[i] = A0[i] + A1[i]*t + A2[i]*t*t + A3[i]*t*t*t + A4[i]*t*t*t*t + A5[i]*t*t*t*t*t;
        tra->vel[i] = A1[i] + 2*A2[i]*t + 3*A3[i]*t*t + 4*A4[i]*t*t*t + 5*A5[i]*t*t*t*t;
        tra->acc[i] = 2*A2[i] + 6*A3[i]*t + 12*A4[i]*t*t + 20*A5[i]*t*t*t;
    }
}

// ============================================================================
// Callback Functions
// ============================================================================
void OnTpInitializingCompleted() {
    g_TpInitailizingComplted = true;
    Drfl.ManageAccessControl(MANAGE_ACCESS_CONTROL_FORCE_REQUEST);
}

void OnHommingCompleted() {
    cout << "Homing completed" << endl;
}

void OnProgramStopped(const PROGRAM_STOP_CAUSE) {
    assert(Drfl.PlayDrlStop(STOP_TYPE_SLOW));
    cout << "Program stopped" << endl;
}

void OnMonitoringDataCB(const LPMONITORING_DATA pData) {
    return;
}

void OnMonitoringDataExCB(const LPMONITORING_DATA_EX pData) {
    return;
}

void OnMonitoringCtrlIOExCB(const LPMONITORING_CTRLIO_EX pData) {
    return;
}

void OnMonitroingAccessControlCB(const MONITORING_ACCESS_CONTROL eTrasnsitControl) {
    switch (eTrasnsitControl) {
        case MONITORING_ACCESS_CONTROL_REQUEST:
            assert(Drfl.ManageAccessControl(MANAGE_ACCESS_CONTROL_RESPONSE_NO));
            break;
        case MONITORING_ACCESS_CONTROL_GRANT:
            g_bHasControlAuthority = true;
            break;
        case MONITORING_ACCESS_CONTROL_DENY:
        case MONITORING_ACCESS_CONTROL_LOSS:
            g_bHasControlAuthority = false;
            if (g_TpInitailizingComplted) {
                Drfl.ManageAccessControl(MANAGE_ACCESS_CONTROL_FORCE_REQUEST);
            }
            break;
        default:
            break;
    }
}

void OnLogAlarm(LPLOG_ALARM tLog) {
    g_mStat = true;
    cout << "Alarm Info: group(" << (unsigned int)tLog->_iGroup 
         << "), index(" << tLog->_iIndex 
         << "), param(" << tLog->_szParam[0] << ")" << endl;
}

void OnTpPopup(LPMESSAGE_POPUP tPopup) {
    cout << "Popup Message: " << tPopup->_szText << endl;
}

void OnTpLog(const char* strLog) {
    cout << "Log Message: " << strLog << endl;
}

void OnTpProgress(LPMESSAGE_PROGRESS tProgress) {
    cout << "Progress: " << (int)tProgress->_iCurrentCount 
         << "/" << (int)tProgress->_iTotalCount << endl;
}

void OnTpGetuserInput(LPMESSAGE_INPUT tInput) {
    cout << "User Input: " << tInput->_szText << endl;
}

void OnRTMonitoringData(LPRT_OUTPUT_DATA_LIST tData) {
    return;
}

void OnDisConnected() {
    while (!Drfl.open_connection("192.168.137.100")) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }
}

// ============================================================================
// Thread Functions
// ============================================================================
uint32_t ThreadFunc(void* arg) {
    printf("Start ThreadFunc\n");
    
    while (true) {
        if (linux_kbhit()) {
            char ch = getch();
            switch (ch) {
                case 's':
                    printf("Stop!\n");
                    g_Stop = true;
                    Drfl.MoveStop(STOP_TYPE_SLOW);
                    break;
                case 'p':
                    printf("Pause!\n");
                    Drfl.MovePause();
                    break;
                case 'r':
                    printf("Resume!\n");
                    Drfl.MoveResume();
                    break;
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    
    return 0;
}

// ============================================================================
// Real-time Task Function
// ============================================================================
void* realtime_task(void* arg) {
    // Initialize positions
    float home[6] = {0, 0, 0, 0, 0, 0};
    float posj_s[6] = {0, 0, 0, 0, 0, 0};
    float posj_f[6] = {30, 30, 60, -30, -90, 30};
    const float None = -10000;
    
    // Set safety mode and move to home
    Drfl.movej(home, 60, 60);
    Drfl.set_safety_mode(SAFETY_MODE_AUTONOMOUS, SAFETY_MODE_EVENT_MOVE);
    
    // Motion parameters
    float target_time = 8.0;
    float vel[6] = {100, 100, 100, 100, 100, 100};
    float acc[6] = {120, 120, 120, 120, 120, 120};
    float vs[6] = {0, 0, 0, 0, 0, 0};
    float vf[6] = {0, 0, 0, 0, 0, 0};
    float as[6] = {0, 0, 0, 0, 0, 0};
    float af[6] = {0, 0, 0, 0, 0, 0};
    
    Drfl.set_velj_rt(vel);
    Drfl.set_accj_rt(acc);
    
    // Initialize trajectory parameters
    TraParam tra1, tra2;
    PlanParam plan1, plan2;
    
    plan1.time = target_time;
    plan2.time = target_time;
    
    memcpy(plan1.ps, posj_s, NUMBER_OF_JOINT * sizeof(float));
    memcpy(plan1.pf, posj_f, NUMBER_OF_JOINT * sizeof(float));
    memcpy(plan2.ps, posj_f, NUMBER_OF_JOINT * sizeof(float));
    memcpy(plan2.pf, posj_s, NUMBER_OF_JOINT * sizeof(float));
    
    memcpy(plan1.vs, vs, NUMBER_OF_JOINT * sizeof(float));
    memcpy(plan1.vf, vf, NUMBER_OF_JOINT * sizeof(float));
    memcpy(plan1.as, as, NUMBER_OF_JOINT * sizeof(float));
    memcpy(plan1.af, af, NUMBER_OF_JOINT * sizeof(float));
    memcpy(plan2.vs, vs, NUMBER_OF_JOINT * sizeof(float));
    memcpy(plan2.vf, vf, NUMBER_OF_JOINT * sizeof(float));
    memcpy(plan2.as, as, NUMBER_OF_JOINT * sizeof(float));
    memcpy(plan2.af, af, NUMBER_OF_JOINT * sizeof(float));
    
    TrajectoryPlan(&plan1);
    TrajectoryPlan(&plan2);
    
    // Control loop parameters
    const int loop_period_ms = 1;
    float dt = 0.001 * loop_period_ms;
    std::chrono::milliseconds loop_period(loop_period_ms);
    
    int repeat = 100000;
    float ratio = 15;
    double tt = dt * ratio;
    
    // Main control loop
    for (int i = 0; i < repeat; i++) {
        bool flag = true;
        int count = 0;
        float time = 0;
        
        PlanParam* current_plan = (i % 2 == 0) ? &plan1 : &plan2;
        TraParam* current_tra = (i % 2 == 0) ? &tra1 : &tra2;
        
        std::cout << "Loop " << i << ") Start position: ";
        for (size_t k = 0; k < 6; k++) {
            std::cout << current_plan->ps[k] << ", ";
        }
        std::cout << std::endl;
        
        while (flag) {
            auto loop_start_time = std::chrono::steady_clock::now();
            
            time = (++count) * dt;
            current_tra->time = time;
            
            TrajectoryGenerator(current_plan, current_tra);
            
            if (i % 2 != 0) {
                for (int j = 0; j < 6; j++) {
                    current_tra->vel[j] = None;
                    current_tra->acc[j] = None;
                }
            }
            
            Drfl.servoj_rt(current_tra->pos, current_tra->vel, current_tra->acc, tt);
            
            if (time > current_plan->time) {
                flag = false;
                std::cout << "Loop " << i << ") End position: ";
                for (size_t k = 0; k < 6; k++) {
                    std::cout << current_plan->pf[k] << ", ";
                }
                std::cout << std::endl;
                break;
            }
            
            auto loop_end_time = std::chrono::steady_clock::now();
            auto elapsed_time = std::chrono::duration_cast<std::chrono::milliseconds>(loop_end_time - loop_start_time);
            
            if (elapsed_time < loop_period) {
                std::this_thread::sleep_for(loop_period - elapsed_time);
            }
        }
    }
    
    return nullptr;
}

// ============================================================================
// Main Function
// ============================================================================
int main(int argc, char** argv) {
    // Set up callbacks
    Drfl.set_on_homming_completed(OnHommingCompleted);
    Drfl.set_on_monitoring_data(OnMonitoringDataCB);
    Drfl.set_on_monitoring_data_ex(OnMonitoringDataExCB);
    //Drfl.set_on_monitoring_ctrl_io_ex(OnMonitoringCtrlIOExCB);
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
    
    // Connect to robot
    assert(Drfl.open_connection("192.168.137.100"));
    
    SYSTEM_VERSION tSysVerion = {'\0'};
    Drfl.get_system_version(&tSysVerion);
    assert(Drfl.setup_monitoring_version(1));
    
    cout << "System version: " << tSysVerion._szController << endl;
    cout << "Library version: " << Drfl.get_library_version() << endl;
    
    // Wait for robot to be ready
    while (!g_bHasControlAuthority) {
        this_thread::sleep_for(std::chrono::milliseconds(1000));
        Drfl.set_robot_control(CONTROL_SERVO_ON);
    }
    
    cout << "Robot state: " << Drfl.get_robot_state() << endl;
    cout << "API control authority: " << g_bHasControlAuthority << endl;
    
    assert(Drfl.set_robot_mode(ROBOT_MODE_AUTONOMOUS));
    assert(Drfl.set_robot_system(ROBOT_SYSTEM_REAL));
    
    // Example types
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
    
#ifdef __XENO__
    RT_TASK sub_task;
    char sub_task_name[256] = {'\0'};
    sprintf(sub_task_name, "drfl_sub_t");
    uint32_t stack_size = 1024 * 64;
    uint32_t prio = 50;
    if (rt_task_spawn(&sub_task, sub_task_name, stack_size, prio,
                      T_CPU(3) | T_JOINABLE,
                      (void (*)(void*)) & ThreadFunc, nullptr) != 0) {
        cout << "Cannot create sub task" << endl;
    }
#endif
    
    // Main control loop
    bool bLoop = true;
    while (bLoop) {
        g_mStat = false;
        g_Stop = false;
        
#ifdef __XENO__
        unsigned long overrun = 0;
        const double tick = 1000000;  // 1ms
        rt_task_set_periodic(nullptr, TM_NOW, tick);
        if (rt_task_wait_period(&overrun) == -ETIMEDOUT) {
            std::cout << __func__ << ": over-runs: " << overrun << std::endl;
        }
#else
        std::this_thread::sleep_for(std::chrono::microseconds(1000));
#endif
        
        cout << "\nInput key: ";
        char ch;
        cin >> ch;
        cout << ch << endl;
        
        switch (ch) {
            case 'q':
                bLoop = false;
                break;
                
            case 's': {
                // Start realtime control sequence (combines cases 1,2,3,4)
                cout << "Starting realtime control sequence..." << endl;
                
                // Step 1: Connect RT control
                Drfl.connect_rt_control("192.168.137.100");
                cout << "RT control connected" << endl;
                
                // Step 2: Set RT control output
                string version = "v1.0";
                float period = 0.001;
                int losscount = 4;
                Drfl.set_rt_control_output(version, period, losscount);
                cout << "RT control output configured" << endl;
                std::this_thread::sleep_for(std::chrono::microseconds(1000));
                
                // Step 3: Set robot mode and start RT control
                Drfl.set_robot_mode(ROBOT_MODE_AUTONOMOUS);
                Drfl.set_safety_mode(SAFETY_MODE_AUTONOMOUS, SAFETY_MODE_EVENT_MOVE);
                Drfl.start_rt_control();
                cout << "RT control started" << endl;
                std::this_thread::sleep_for(std::chrono::microseconds(1000));
                pthread_t rt_thread;
                pthread_attr_t attr;
                struct sched_param param;
                
                pthread_attr_init(&attr);
                pthread_attr_setinheritsched(&attr, PTHREAD_EXPLICIT_SCHED);
                pthread_attr_setschedpolicy(&attr, SCHED_FIFO);
                param.sched_priority = 99;
                pthread_attr_setschedparam(&attr, &param);
                
                int v = pthread_create(&rt_thread, &attr, realtime_task, nullptr);
                if (v != 0) {
                    perror("Failed to create real-time thread");
                    std::cout << "Error: " << v << std::endl;
                    return 1;
                }
                cout << "Realtime thread created successfully" << endl;
                break;
            }            
            case 'e':
                Drfl.stop_rt_control();
                Drfl.disconnect_rt_control();
                break;
                
            default:
                break;
        }
        
        this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    
    // Cleanup
    Drfl.disconnect_rt_control();
    Drfl.CloseConnection();
    
#ifdef __XENO__
    rt_task_join(&sub_task);
#endif
    
    return 0;
}