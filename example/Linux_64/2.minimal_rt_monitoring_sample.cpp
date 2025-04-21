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

const std::string IP_ADDRESS = "192.168.137.100";
CDRFLEx robot; // Instance for APIs

bool get_control_access = false; // Variable to check control authority
bool is_standby = false; // Variable to check whether the robot state is standby.


int main(){
	// Connect to the drcf cotnroller. 
	bool ret = robot.connect_rt_control(IP_ADDRESS);
	std::cout << "open connection return value " << ret << std::endl;
	if (true != ret) {
		std::cout << "Cannot open connection to robot @ " << IP_ADDRESS
						<< std::endl;
		return 1;
	}
	// For rt - monitoring, we don't need to have "Getting control access". 
	// however, for rt-writing like servoj_rt, we still need to have "control access" and "state standby".
	robot.set_on_rt_monitoring_data([](LPRT_OUTPUT_DATA_LIST data)->void{
		auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch());
		std::cout << "[set_on_rt_monitoring_data] Timestamp : (ms since epoch): " << duration.count() << std::endl;
	});
	
	string version = "v1.0";
	float period = 0.002;
	int losscount = 4;
	std::cout << "RT Result " << robot.set_rt_control_output(version, period, losscount) << std::endl;

	std::cout << "Press Enter to continue..." ;
	std::cin.get();  // Waits for user to press Enter
	robot.start_rt_control();

	std::cout << "Press Enter to terminate..." ;
	std::cin.get();  // Waits for user to press Enter
	robot.disconnect_rt_control();
	return 0;
}
