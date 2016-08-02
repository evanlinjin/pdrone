#include <ros/ros.h>
#include <time.h>
#include <pdrone/SetGround.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "main_node");
    ros::NodeHandle nh;
    ros::ServiceClient ground_client = nh.serviceClient<pdrone::SetGround>("ground/set");
    //ros::Subscriber ground_sub = nh.subscribe("ground/set_done", 10, )
    ros::Rate loopRate(0.02);

    while (ros::ok()) {
        pdrone::SetGround ground_srv;
        ground_srv.request.distance = 500;
        ground_srv.request.speed = 80;
        ground_srv.request.direction = 'F';
        ground_srv.request.mode = '1';
        ground_srv.request.stamp = ros::Time::now();

        if (ground_client.call(ground_srv)) {
            ROS_INFO("REQUEST COMPLETE");
        }

        loopRate.sleep(); ros::spinOnce();

        ground_srv.request.distance = 500;
        ground_srv.request.speed = 85;
        ground_srv.request.direction = 'L';
        ground_srv.request.mode = '0';
        ground_srv.request.stamp = ros::Time::now();

        if (ground_client.call(ground_srv)) {
            ROS_INFO("REQUEST COMPLETE");
        }

        loopRate.sleep(); ros::spinOnce();

        ground_srv.request.distance = 500;
        ground_srv.request.speed = 90;
        ground_srv.request.direction = 'R';
        ground_srv.request.mode = '1';
        ground_srv.request.stamp = ros::Time::now();

        if (ground_client.call(ground_srv)) {
            ROS_INFO("REQUEST COMPLETE");
        }

        loopRate.sleep(); ros::spinOnce();
    }

    return 0;
}