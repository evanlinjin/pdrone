#include <cstdlib>
#include <string>
#include <iostream>
#include <thread>
#include <functional>
#include "motor_controller.h"
#include "GPIO_IN.h"

#define PERIOD 500000

using namespace std;

// DECLARE : FUNCTIONS >>
void spin_td();
void ticks_td(GPIO_IN* en, int* ticks, short* dir);

bool move_to_pos_raw(
    ServiceController* sc,
    MotorController* mc,
    ros::Rate* waiter,
    int* spd_0,
    int* spd_1,
    int* error,
    int* kp,
    int* t0_old,
    int* t0_new,
    int* t1_old,
    int* t1_new,
    int* en0_ticks,
    int* en1_ticks,
    int distance,
    double timeout
);

// DEFINE : MAIN LOOP >>
int main(int argc, char ** argv) {

    // SETUP : ROS >>
    ros::init(argc, argv, "ground_node");
    ros::NodeHandle nh;

    // SETUP : Rotary Encoder >>
    int en0_ticks = -1;
    int en1_ticks = -1;

    // SETUP : Rotary Encoders >>
    GPIO_IN en0_pin(30, "both", 0);
    GPIO_IN en1_pin(60, "both", 0);
    en0_pin.epoll_setup();
    en1_pin.epoll_setup();

    // SETUP : Motor Controller & Service Server >>
        // GPIO_31, GPIO_48, pwmchip2, 2000Hz
    MotorController* mc = new MotorController(31, 48, 2, PERIOD);
    ServiceController* sc = new ServiceController(&nh);
    ros::ServiceServer service = nh.advertiseService("ground/set", &ServiceController::callback, sc);

    // SETUP : Threads >>
    thread spin_thread(spin_td);
    thread ticks0_thread(ticks_td, &en0_pin, &en0_ticks, &mc->dir[0]);
    thread ticks1_thread(ticks_td, &en1_pin, &en1_ticks, &mc->dir[1]);

    // SETUP : Loop Variables >>
    int error, kp, loop_freq;
    int spd_0, spd_1;
    int t0_old, t0_new;
    int t1_old, t1_new;
    int ticks_speedup[5];
    SpeedCalc spd_calc;

    // SETUP : ROS Parameters >>
    ros::param::param<int>("~error", error, 0);
    ros::param::param<int>("~kp", kp, 500);
    ros::param::param<int>("~loop_freq", loop_freq, 10);
    ROS_INFO(
        "Set Parameters : error[%d], kp[%d], loop_freq[%d]",
        error, kp, loop_freq
    );

    // SETUP : Waiters >>
    ros::Rate waiter(10); // 10Hz = 0.1s
    ros::Rate mode0_waiter(loop_freq); // 10Hz = 0.1s

    // SETUP : Binder >>
    auto move_to_pos = std::bind(
        move_to_pos_raw,
        sc,
        mc,
        placeholders::_2,
        &spd_0,
        &spd_1,
        &error,
        &kp,
        &t0_old,
        &t0_new,
        &t1_old,
        &t1_new,
        &en0_ticks,
        &en1_ticks,
        placeholders::_1,
        placeholders::_3
    );

    // LOOP : Main Algorithm >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
    while (ros::ok()) {

        // Wait for new service call :
        while (ros::ok() && !sc->is_new()) {waiter.sleep();}
        ROS_INFO("Request Recieved : OK");
        sc->set_incomplete();
        mc->stop();

        // Reset Variables :
        en0_ticks = en1_ticks = 0; /* for encoder */
        error = t0_old = t0_new = t1_old = t1_new = 0; /* for feedback */

        // Begin Movement :
        switch (sc->mode) {

            case '0': // MODE '0' : No Speedup or Speeddown >>>>>>>>>>>>>>>>>>>>>>>>>>>>>

            spd_0 = spd_1 = PERIOD * sc->speed/100;
            if (move_to_pos(sc->distance, &mode0_waiter, -1)) {
                sc->set_complete();
            }

            break;
            case '1': // MODE '1' : Set Speedup & Speeddown with duration 2s >>>>>>>>>>>>

            // Initialise speeds :
            spd_calc.init(PERIOD * 1/2, PERIOD * sc->speed/100, 3);

            // Speedup section :
            spd_0 = spd_1 = spd_calc.get(0);
            if (move_to_pos(sc->distance, &mode0_waiter, 0.7)) {sc->set_complete(); break;}
            ticks_speedup[0] = (en0_ticks + en1_ticks)/2;
            ROS_INFO("Request Processing : TICKS[%d, %d]", en0_ticks, en1_ticks);

            spd_0 = spd_1 = spd_calc.get(1);
            if (move_to_pos(sc->distance, &mode0_waiter, 0.7)) {sc->set_complete(); break;}
            ticks_speedup[1] = (en0_ticks + en1_ticks)/2;
            ROS_INFO("Request Processing : TICKS[%d, %d]", en0_ticks, en1_ticks);

            spd_0 = spd_1 = spd_calc.get(2);
            if (move_to_pos(sc->distance, &mode0_waiter, 0.7)) {sc->set_complete(); break;}
            ticks_speedup[2] = (en0_ticks + en1_ticks)/2;
            ROS_INFO("Request Processing : TICKS[%d, %d]", en0_ticks, en1_ticks);

            // Max speed section :
            spd_0 = spd_1 = spd_calc.get(3);
            if (move_to_pos(sc->distance - ticks_speedup[2], &mode0_waiter, -1.0)) {sc->set_complete();}
            ROS_INFO("Request Processing : TICKS[%d, %d]", en0_ticks, en1_ticks);

            // Speeddown section :
            spd_0 = spd_1 = spd_calc.get(2);
            if (move_to_pos(sc->distance - ticks_speedup[1], &mode0_waiter, 0.7)) {sc->set_complete();}
            ROS_INFO("Request Processing : TICKS[%d, %d]", en0_ticks, en1_ticks);

            spd_0 = spd_1 = spd_calc.get(1);
            if (move_to_pos(sc->distance - ticks_speedup[0], &mode0_waiter, 0.7)) {sc->set_complete();}
            ROS_INFO("Request Processing : TICKS[%d, %d]", en0_ticks, en1_ticks);

            spd_0 = spd_1 = spd_calc.get(0);
            if (move_to_pos(sc->distance, &mode0_waiter, 1)) {sc->set_complete();}
            ROS_INFO("Request Completed : TICKS[%d, %d]", en0_ticks, en1_ticks);

            break;
        }

        // Complete Service :
        mc->stop();
        ROS_INFO("Request Ended : TICKS[%d, %d]", en0_ticks, en1_ticks);
    }

    // CLOSURE >>
    spin_thread.join();
    ticks0_thread.join();
    ticks1_thread.join();
    delete sc, mc;
    return 0;
}

// DEFINE : FUNCTIONS >>

void spin_td() {ros::spin();}

void ticks_td(GPIO_IN* en, int* ticks, short* dir) {
    while (ros::ok()) {
        if (en->epoll_wait_once()) {*ticks += 1;}
    }
}

int glob_start_time; // Fuck I'm lazy

// Returns false if incomplete.
bool move_to_pos_raw(
    ServiceController* sc,
    MotorController* mc,
    ros::Rate* waiter,
    int* spd_0,
    int* spd_1,
    int* error,
    int* kp,
    int* t0_old,
    int* t0_new,
    int* t1_old,
    int* t1_new,
    int* en0_ticks,
    int* en1_ticks,
    int distance,
    double timeout
) {
    glob_start_time = (int)ros::Time::now().toSec();

    while (ros::ok() && !sc->is_new()) {
        // Change motor speeds :
        mc->set(*spd_0, *spd_1, sc->direction);
        waiter->sleep(); ros::spinOnce();

        // Stop motors when distance reached : or timeout expired :
        if (*en0_ticks >= distance  || *en1_ticks >= distance) {
            *spd_0 = 0; *spd_1 = 0; return true;
        }
        if (timeout >= 0) {
            if (ros::Time::now().toSec() - glob_start_time >= timeout) {
                return false;
            }
        }

        // Calculate error :
        *t0_old = *t0_new; *t1_old = *t1_new;
        *t0_new = *en0_ticks; *t1_new = *en1_ticks;
        *error = (*t0_new - *t0_old) - (*t1_new - *t1_old);

        // Correct motor speed settings :
        *spd_1 += *error * *kp;
        if (*spd_1 > PERIOD) {*spd_1 = PERIOD;} else if (*spd_1 < 0) {*spd_1 = 0;}

        // Print info :
        ROS_INFO(
            "ticks[%d, %d] spd[%d, %d] err[%d, %d]",
            *en0_ticks, *en1_ticks, *spd_0, *spd_1, *error, *en0_ticks - *en1_ticks
        );
    }
    return false;
}