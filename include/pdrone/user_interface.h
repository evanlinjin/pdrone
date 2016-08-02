#ifndef USER_INTERFACE_H_
#define USER_INTERFACE_H_

#include "utils.h"
#include <unistd.h>

class UserInterface {
    UserInterface();
    ~UserInterface();

    //  /sys/devices/platform/leds/leds/beaglebone:green:usr0
    //  /sys/devices/platform/leds/leds/beaglebone:green:usr1
    //  /sys/devices/platform/leds/leds/beaglebone:green:usr2
    //  /sys/devices/platform/leds/leds/beaglebone:green:usr3
};

UserInterface::UserInterface() {

}

UserInterface::~UserInterface() {

}

#endif /* USER_INTERFACE_H_ */