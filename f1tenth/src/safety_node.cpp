#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
// TODO: include ROS msg type headers and libraries
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <std_msgs/Bool.h>
#include <cmath>


class Safety {
// The class that handles emergency braking
private:
    ros::NodeHandle n;
    double speed;
    // TODO: create ROS subscribers and publishers
    ros::Publisher brake_pub;
    ros::Publisher brake_bool_pub;
    ros::Subscriber scan_sub;
    ros::Subscriber odo_sub;

public:

    Safety() {
        n = ros::NodeHandle();
        speed = 0.0;
        /*
        One publisher should publish to the /brake topic with an
        ackermann_msgs/AckermannDriveStamped brake message.

        One publisher should publish to the /brake_bool topic with a
        std_msgs/Bool message.

        You should also subscribe to the /scan topic to get the
        sensor_msgs/LaserScan messages and the /odom topic to get
        the nav_msgs/Odometry messages

        The subscribers should use the provided odom_callback and 
        scan_callback as callback methods

        NOTE that the x component of the linear velocity in odom is the speed
        */

        // TODO: create ROS subscribers and publishers
        scan_sub = n.subscribe("/scan", 1000, &Safety::scan_callback, this);
        odo_sub = n.subscribe("/odom", 1000, &Safety::odom_callback, this);
        brake_pub = n.advertise<ackermann_msgs::AckermannDriveStamped>("/brake", 1000);
        brake_bool_pub = n.advertise<std_msgs::Bool>("/brake_bool", 1000);
        
    }
    void odom_callback(const nav_msgs::Odometry::ConstPtr &odom_msg) {
        // TODO: update current speed
        
        speed = odom_msg->twist.twist.linear.x;
    }

    void scan_callback(const sensor_msgs::LaserScan::ConstPtr &scan_msg) {
        // TODO: calculate TTC
        std::vector<float> TTC(scan_msg->ranges.size(), 99999.0);
        float theta = scan_msg->angle_min;
        float TTC_min = 99999.0;
        for(int i = 0; i < scan_msg->ranges.size(); i++) {
            if(!std::isnan(scan_msg->ranges[i]) && !std::isinf(scan_msg->ranges[i])) {
                TTC[i] = scan_msg->ranges[i]/(std::max(speed*std::cos(theta),0.001));
                theta += scan_msg->angle_increment;
                if(TTC[i] < TTC_min) {
                    TTC_min = TTC[i];
                }
            }
        }
        

        // TODO: publish drive/brake message
        if(TTC_min < 0.35) {
            std_msgs::Bool Bool_msg;
            Bool_msg.data = true;
            brake_bool_pub.publish(Bool_msg);

            ackermann_msgs::AckermannDriveStamped drive_msg;
            drive_msg.drive.speed = 0.0;
            brake_pub.publish(drive_msg);

            ROS_INFO("brake applied\n");
        } else {
            std_msgs::Bool Bool_msg;
            Bool_msg.data = false;
            brake_bool_pub.publish(Bool_msg);
        }
    }

};
int main(int argc, char ** argv) {
    ros::init(argc, argv, "safety_node");
    Safety sn;
    ros::spin();
    return 0;
}