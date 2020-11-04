#include <ros/ros.h>
#include "./src/fncs_header.hpp"


int main(int argc, char **argv)
{
    ros::init(argc, argv, "drone_contest");
    ROS_INFO("drone_contest, Start~ :)");
    UAV_control_class * UAV_controller = new UAV_control_class();

    UAV_controller->nh.getParam("wp_vel",UAV_controller->wp_vel);
    UAV_controller->nh.getParam("height_vel",UAV_controller->height_vel);
    UAV_controller->nh.getParam("landing_vel",UAV_controller->landing_vel);
    UAV_controller->nh.getParam("yaw_vel",UAV_controller->yaw_vel);//rad
    UAV_controller->nh.getParam("take_off_height",UAV_controller->take_off_height);
    UAV_controller->nh.getParam("threshold_distance",UAV_controller->threshold_distance);
    UAV_controller->nh.getParam("arrive_distance",UAV_controller->arrive_distance);
    UAV_controller->nh.getParam("threshold_yaw",UAV_controller->threshold_yaw);
    UAV_controller->nh.getParam("LOS_radius",UAV_controller->LOS_radius);
    UAV_controller->nh.getParam("auto_arrive_delay",UAV_controller->auto_arrive_delay);
    UAV_controller->nh.getParam("test_mode",UAV_controller->test_mode);


    ros::Rate loop_rate(50);

    while(ros::ok()){
        ros::spinOnce();
//        ROS_INFO("uav_control UAV control mode : %d",UAV_controller->UAV_control_mode);
        UAV_controller->spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
