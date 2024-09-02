#pragma once

#include <ros/ros.h>
#include <ctrl_fb.h>
#include <sensor_msgs/NavSatFix.h>
#include <ekf.h>
#include <GeographicLib/LocalCartesian.hpp>
class ros_node
{
private:
    ros::NodeHandle nh_;            // ROS节点句柄
    ros::Subscriber wheel_sub_;     // 轮速计数据订阅者
    ros::Subscriber rtk_sub_;       // GPS数据订阅者
    

    // 计算gnss观测速度
    double last_time_, last_x_, last_y_;

    
public:
    GeographicLib::LocalCartesian wgs_enu_converter_;
    ekf ekf_sys_;
    ros_node();
    // ~ros_node();
    void wheelCallback(const yhs_can_msgs::ctrl_fbConstPtr &wheel_msg);
    void rtkCallback(const sensor_msgs::NavSatFixConstPtr &rtk_msg);
};
