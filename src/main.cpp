#include <ros_node.h>
#include <viewer.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ekf_filter_node");
    ros_node ekf_node;
    
    ros::spin();
    return 0;
}
