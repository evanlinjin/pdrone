#ifndef INITIATE_H_
#define INITIATE_H_

#include <ros/ros.h>
#include <unistd.h>
#include <fstream>

using namespace std;

void init_dto() {
	ifstream stream;
    string line, token = "EBB-GPIO-Pdrone";
    stream.open("/sys/devices/platform/bone_capemgr/slots");
    bool is_found = false;
    while (getline(stream, line)) {
        if (line.find(token) != string::npos) {
            is_found = true;
            ROS_INFO("DTO file already enabled.");
            break;
        }
    }
    if(!is_found) {
    	ROS_INFO("Enabling DTO file ...");
        system("sudo sh -c \"echo 'EBB-GPIO-Pdrone' > /sys/devices/platform/bone_capemgr/slots\" ");
    }
}

#endif /* INITIATE_H_ */
