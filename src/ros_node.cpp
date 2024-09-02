#include <ros_node.h>

ros_node::ros_node(){
    
    wheel_sub_ = nh_.subscribe<yhs_can_msgs::ctrl_fb>("/ctrl_fb", 200, &ros_node::wheelCallback, this);
    rtk_sub_ = nh_.subscribe<sensor_msgs::NavSatFix>("/fix", 10, &ros_node::rtkCallback, this);
}

void ros_node::wheelCallback(const yhs_can_msgs::ctrl_fbConstPtr& wheel_msg){
    const double timestamp = wheel_msg->header.stamp.toSec();
    const double v = wheel_msg->ctrl_fb_linear;
    const double w = wheel_msg->ctrl_fb_angular* M_PI / 180;
    ekf_sys_.propagate(v, w, timestamp);
    // ROS_INFO("[v: %f, w: %f]", v, w);
    // filter_fusion_sys->FeedWheelData(timestamp, linear_v, angular_v);
    return;
}

void ros_node::rtkCallback(const sensor_msgs::NavSatFixConstPtr& rtk_msg){
    if (rtk_msg->status.status) return;

    if (rtk_msg->position_covariance[0] > 800 ||
        rtk_msg->position_covariance[4] > 800 ||
        rtk_msg->position_covariance[8] > 800) return;
    
    const double timestamp = rtk_msg->header.stamp.toSec();
    double rtk_lat = rtk_msg->latitude;
    double rtk_lon = rtk_msg->longitude;
    double rtk_alt = rtk_msg->altitude;
    if(!ekf_sys_.enu_initialized_){
        // ekf_sys_.init_lat = rtk_lat;
        // ekf_sys_.init_lon = rtk_lon;
        // ekf_sys_.init_alt = rtk_alt;
        wgs_enu_converter_.Reset(rtk_lat, rtk_lon, rtk_alt);
        last_time_ = timestamp;
        ekf_sys_.enu_initialized_ = true;
    }
    double x, y, z, x_v, y_v, delta_t;
    wgs_enu_converter_.Forward(rtk_lat, rtk_lon, rtk_alt, x, y, z);
    delta_t = timestamp - last_time_;
    x_v = (x - last_x_) / delta_t;
    y_v = (y - last_y_) / delta_t;
    if (hypot(x_v, y_v) > 1.1) return;
    
    double theta = atan2(y_v, x_v);

    last_time_ = timestamp;
    last_x_ = x;
    last_y_ = y;
    // ROS_INFO("[vx: %f, vy: %f, theta: %f]", x_v, y_v, theta);    
    ekf_sys_.update(x, y, theta, timestamp);

    // const boost::array<double, 9>& position_covariance = rtk_msg->position_covariance;
    // Eigen::Matrix<double, 3, 3> covariance_matrix;
    // covariance_matrix << position_covariance[0], position_covariance[1], position_covariance[2],
    //                      position_covariance[3], position_covariance[4], position_covariance[5],
    //                      position_covariance[6], position_covariance[7], position_covariance[8];
    // filter_fusion_sys->FeedGpsData(timestamp, rtk_longitude, rtk_latitude, rtk_altitude, covariance_matrix);
}
