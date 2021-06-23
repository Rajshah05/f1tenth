#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "std_msgs/Float64.h"
#include "f1tenth/scan_range.h"


ros::Publisher scan_range;
// ros::Publisher scan_min;
// ros::Rate loop_rate(1);

void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan_msg) {
    std_msgs::Float64 max_range, min_range;
    max_range.data = scan_msg->range_min;
    min_range.data = scan_msg->range_max;
    ros::Rate loop(1);

    for(auto x : scan_msg->ranges) {
        if(x > max_range.data) max_range.data = x;
        if(x < min_range.data) min_range.data = x;
    }

    f1tenth::scan_range range_scan;
    range_scan.max_range = max_range.data;
    range_scan.min_range = min_range.data;

    scan_range.publish(range_scan);

    ros::spinOnce();
    // loop_rate.sleep();
    loop.sleep();
}


int main(int argc, char** argv) {
    ros::init(argc, argv, "laser_pub_sub");
    ros::NodeHandle n;
    ros::Subscriber scan_sub = n.subscribe("/scan", 1000, scanCallback);
    
    scan_range = n.advertise<f1tenth::scan_range>("scan_range", 1000);
    // scan_max = n.advertise<std_msgs::Float64>("fartherst_point", 1000);
    ros::spin();
}