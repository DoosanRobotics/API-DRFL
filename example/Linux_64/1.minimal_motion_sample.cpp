#define DRCF_VERSION 2 // Set 2 or 3 according to drcf version.

#include "../../include/DRFLEx.h"

#include <cstring>
#include <iostream>
#include <thread>

using namespace DRAFramework; 

/**
 * In thie sample code, we will introduce minimal sample to ensure robot movements.
 * 
 */

const std::string IP_ADDRESS = "127.0.0.1";
CDRFLEx robot; // Instance for APIs

bool get_control_access = false; // Variable to check control authority
bool is_standby = false; // Variable to check whether the robot state is standby.

std::string to_str(const MONITORING_ACCESS_CONTROL x);
std::string to_str(const ROBOT_STATE x);

int main(){
  // There several type of callbacks. we will only introduce "key property callbacks" for general movement.
	// Simple Sample Function about authority.

	// First of all, We are interest in getting 'control access' to manipluate the connected robot.
	auto OnMonitroingAccessControlCB = [](const MONITORING_ACCESS_CONTROL access) {
		std::cout << "[OnMonitroingAccessControlCB] Control Access : " << to_str(access) << std::endl;
		if(MONITORING_ACCESS_CONTROL_GRANT == access) {
			get_control_access = true;
		}
		if(MONITORING_ACCESS_CONTROL_LOSS == access) {
			get_control_access = false;
		}
	};
	robot.set_on_monitoring_access_control(OnMonitroingAccessControlCB);

	// We are also interest in "robot standby" because it says: the robot is ready for movement.
	auto OnMonitoringStateCB = [] (const ROBOT_STATE state) {
		std::cout << "[OnMonitoringStateCB] Robot State : " << to_str(state) << std::endl;
		if(STATE_STANDBY == state) {
			is_standby = true;
		}else {
			is_standby = false;
		}
	};
	robot.set_on_monitoring_state(OnMonitoringStateCB);
	
	// Connect to the drcf cotnroller. 
	bool ret = robot.open_connection(IP_ADDRESS);
	std::cout << "open connection return value " << ret << std::endl;
	if (true != ret) {
		std::cout << "Cannot open connection to robot @ " << IP_ADDRESS
					<< std::endl;
		return 1;
	}

	// This function is for "compatibilty".
	// e.g. In v2 drcf, 'set_on_monitoring_ctrl_io' will be invoked but which is depreciated.
	// Generally, please set '1' to being in the newer context.
	robot.setup_monitoring_version(1);

	SYSTEM_VERSION tSysVerion = {
		'\0',
	};
	robot.get_system_version(&tSysVerion);
	cout << "Controller (DRCF) version: " << tSysVerion._szController << endl;
	cout << "Library version: " << robot.get_library_version() << endl; // GLXXXXXX


	// In API, we typically set 'ROBOT_MODE_MANUAL' or 'ROBOT_MODE_AUTONOMOUS'.
	// Generally, user need to specify 'ROBOT_MODE_MANUAL' before setting robot configuration.
	// and specify 'ROBOT_MODE_AUTONOMOUS' before movements.
	// In thie example, we would like to move the robot. set 'ROBOT_MODE_AUTONOMOUS'. 
	if(!robot.set_robot_mode(ROBOT_MODE_AUTONOMOUS)) {
		return 1;
	}

	// We will make sure "getting control access" and "stand_by" state.
	// This means, being done of "ready to movement".
	for (size_t retry = 0; retry < 10; ++retry, std::this_thread::sleep_for(std::chrono::milliseconds(1000))) {
		if(!get_control_access) {
				robot.ManageAccessControl(MANAGE_ACCESS_CONTROL_FORCE_REQUEST);
				continue;
		}
		if(!is_standby) {
				robot.set_robot_control(CONTROL_SERVO_ON);
				continue;
		}
		if(get_control_access && is_standby)   break;
	}


	if(!(get_control_access && is_standby)) {
		std::cout << "Failed Setting intended states" << std::endl;
		return 1;
	}

	/*******************************************/
	/********** READY TO MOVEMENT **************/
	/*******************************************/

	std::cout << "Press Enter to continue..." ;
	std::cin.get();  // Waits for user to press Enter

	float targetPos[6] = {0.,0.,0.,0.,0.,0.};
	float targetVel = 30;
	float targetAcc = 30;
	robot.movej(targetPos, targetVel, targetAcc);

	targetPos[3] = 30;
	robot.movej(targetPos, targetVel, targetAcc);


	robot.close_connection();
	return 0;
}

std::string to_str(const MONITORING_ACCESS_CONTROL x)
{
    if(x == MONITORING_ACCESS_CONTROL_REQUEST)  return "MONITORING_ACCESS_CONTROL_REQUEST";
    if(x == MONITORING_ACCESS_CONTROL_DENY)  return "MONITORING_ACCESS_CONTROL_DENY";
    if(x == MONITORING_ACCESS_CONTROL_GRANT)  return "MONITORING_ACCESS_CONTROL_GRANT";
    if(x == MONITORING_ACCESS_CONTROL_LOSS)  return "MONITORING_ACCESS_CONTROL_LOSS";
    if(x == MONITORING_ACCESS_CONTROL_LAST)  return "MONITORING_ACCESS_CONTROL_LAST";
    return "to_str err";
}


std::string to_str(const ROBOT_STATE x)
{
    if(x == STATE_INITIALIZING)  return "STATE_INITIALIZING";
    if(x == STATE_STANDBY)  return "STATE_STANDBY";
    if(x == STATE_MOVING)  return "STATE_MOVING";
    if(x == STATE_SAFE_OFF)  return "STATE_SAFE_OFF";
    if(x == STATE_TEACHING)  return "STATE_TEACHING";
    if(x == STATE_SAFE_STOP)  return "STATE_SAFE_STOP";
    if(x == STATE_EMERGENCY_STOP)  return "STATE_EMERGENCY_STOP";
    if(x == STATE_HOMMING)  return "STATE_HOMMING";
    if(x == STATE_RECOVERY)  return "STATE_RECOVERY";
    if(x == STATE_SAFE_STOP2)  return "STATE_SAFE_STOP2";
    if(x == STATE_SAFE_OFF2)  return "STATE_SAFE_OFF2";
    if(x == STATE_RESERVED1)  return "STATE_RESERVED1";
    if(x == STATE_RESERVED2)  return "STATE_RESERVED2";
    if(x == STATE_RESERVED3)  return "STATE_RESERVED3";
    if(x == STATE_RESERVED4)  return "STATE_RESERVED4";
    if(x == STATE_NOT_READY)  return "STATE_NOT_READY";
    if(x == STATE_LAST)  return "STATE_LAST";
    return "to_str err";
}
