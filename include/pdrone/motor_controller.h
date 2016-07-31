#ifndef GROUND_NODE_H_
#define GROUND_NODE_H_

#include "PWM.h"
#include "GPIO_OUT.h"
#include <unistd.h>
#include <ros/ros.h>
#include <time.h>
#include <pdrone/SetGround.h>
#include <std_msgs/Bool.h>

class MotorController {
public:
    MotorController(uint n_bk0, uint n_bk1, uint n_pwmchip, uint period_pwm);
    ~MotorController();
    void set_0(uint speed, short direction);
    void set_1(uint speed, short direction);
    void set(uint spd_0, short dir_0, uint spd_1, short dir_1);
    void set(uint spd_0, uint spd_1, char direction);
    void stop();

    short dir[2];


private:
    GPIO_OUT* BK0;
    GPIO_OUT* BK1;
    PWM* FD;

    uint period;
};

class ServiceController {
public:
    ServiceController(ros::NodeHandle* nh);
    ~ServiceController();
    bool callback(pdrone::SetGround::Request &req, pdrone::SetGround::Response &res);
    bool is_new();
    void set_incomplete();
    void set_complete();

    int distance;
    ushort speed;
    char direction;
    char mode;
    bool new_bool;


private:
    std_msgs::Bool complete_msg;
    ros::Publisher pub;
};

class SpeedCalc {
public:
    SpeedCalc();
    ~SpeedCalc();
    void init(int min, int max, int n_values);
    int get(int i);
private:
    int values[10];
};

// DEFINITIONS : MotorController >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>

MotorController::MotorController(uint n_bk0, uint n_bk1, uint n_pwmchip, uint period_pwm) {
    this->period = period_pwm;
    dir[0] = 1; dir[1] = 1;
    BK0 = new GPIO_OUT(n_bk0);
    BK1 = new GPIO_OUT(n_bk1);
    FD = new PWM(n_pwmchip, period_pwm);
    this->stop();
}

MotorController::~MotorController() {
    delete BK0;
    delete BK1;
    delete FD;
}

void MotorController::set_0(uint n_speed, short direction) {
    dir[0] = direction;
    FD->set_duty(0, direction == 1 ? n_speed : period - n_speed);
    BK0->set_value(direction == 1 ? 0 : 1);
}

void MotorController::set_1(uint n_speed, short direction) {
    dir[1] = direction;
    FD->set_duty(1, direction == 1 ? n_speed : period - n_speed);
    BK1->set_value(direction == 1 ? 0 : 1);
}

void MotorController::set(uint spd_0, short dir_0, uint spd_1, short dir_1) {
    dir[0] = dir_0;
    dir[1] = dir_1;
    FD->set_duty(0, dir_0 == 1 ? spd_0 : period - spd_0);
    BK0->set_value(dir_0 == 1 ? 0 : 1);
    FD->set_duty(1, dir_1 == 1 ? spd_1 : period - spd_1);
    BK1->set_value(dir_1 == 1 ? 0 : 1);
}

void MotorController::set(uint spd_0, uint spd_1, char direction) {
    uint spd[2] = {spd_0, spd_1};
    switch (direction) {
        case 'F': dir[0] = 1; dir[1] = 1; break;
        case 'B': dir[0] = -1; dir[1] = -1; break;
        case 'L': dir[0] = -1; dir[1] = 1; break;
        case 'R': dir[0] = 1; dir[1] = -1; break;
        case 'l': dir[0] = 0; dir[1] = 1; spd[0] = 0; break;
        case 'r': dir[0] = 1; dir[1] = 0; spd[1] = 0; break;
    }
    FD->set_duty(0, dir[0] == 1 ? spd_0 : period - spd_0);
    BK0->set_value(dir[0] == 1 ? 0 : 1);
    FD->set_duty(1, dir[1] == 1 ? spd_1 : period - spd_1);
    BK1->set_value(dir[1] == 1 ? 0 : 1);
    // ROS_INFO(
    //     "DUTY: [%d, %d], VALUE: [%d, %d]",
    //     dir[0] == 1 ? spd_0 : period - spd_0,
    //     dir[1] == 1 ? spd_1 : period - spd_1,
    //     dir[0] == 1 ? 0 : 1,
    //     dir[1] == 1 ? 0 : 1
    // );
}

void MotorController::stop() {
    BK0->set_value(0);
    BK1->set_value(0);
    FD->set_duty(0, 0);
    FD->set_duty(1, 0);
}

// DEFINITIONS : ServiceController >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>

ServiceController::ServiceController(ros::NodeHandle* nh) {
    distance = 0;
    speed = 0;
    direction = 'f';
    mode = '0';
    complete_msg.data = true;
    pub = nh->advertise<std_msgs::Bool>("ground/set_done", 10);
}

ServiceController::~ServiceController() {}

bool ServiceController::callback(pdrone::SetGround::Request &req, pdrone::SetGround::Response &res) {
    distance = req.distance;
    speed = req.speed;
    direction = req.direction;
    mode = req.mode;

    new_bool = true;
    
    return true;
}

bool ServiceController::is_new() {
    return new_bool;
}

 void ServiceController::set_incomplete() {
     new_bool = false;
 }

void ServiceController::set_complete() {
    pub.publish(complete_msg);
    new_bool = false;
}



SpeedCalc::SpeedCalc() {}
SpeedCalc::~SpeedCalc() {}

void SpeedCalc::init(int min, int max, int n_values) {
    int step = (max - min) / n_values;

    for (int i = 0; i <= n_values; i++) {
        values[i] = min + step * i;
        ROS_INFO("Speed%d: [%d]", i, values[i]);
    }
}

int SpeedCalc::get(int i) {return values[i];}

#endif /* GROUND_NODE_H_ */