#include <ros/ros.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <sensor_msgs/LaserScan.h>
// #include <math.h>








class WallFollow {
private:
    ros::NodeHandle n;
    ros::Subscriber scan_sub;
    ros::Publisher drive_pub;
    float Dt_future = 0;
    static constexpr float L = 1.0;
    float alpha = 0;
    static constexpr float DESIRED_LEFT_DIST = 0.7;
    float error = 0;
    float prev_error = 0;
    float integral = 0;
    // float str_ang_vel = 0;
    static constexpr float Kp = 3;
    static constexpr float Ki = 0.0002;
    static constexpr float Kd = 0.09;


   
public:
    WallFollow() {
        n = ros::NodeHandle();
        scan_sub = n.subscribe("/scan", 1000, &WallFollow::scan_callback, this);
        drive_pub = n.advertise<ackermann_msgs::AckermannDriveStamped>("/nav_wall_follow", 1000);
    }
    void scan_callback(const sensor_msgs::LaserScan::ConstPtr& msg) {
        float rangeM90 = msg->ranges[(int)((3*M_PI/2)/msg->angle_increment)];
        float rangeM30 = msg->ranges[(int)((7*M_PI/6)/msg->angle_increment)];
        if(!std::isnan(rangeM30) && !std::isnan(rangeM90) && !std::isinf(rangeM90) && !std::isinf(rangeM30)) {
            alpha = std::atan2(rangeM30*0.5 - rangeM90, rangeM30*0.866);
            Dt_future = rangeM90*cos(alpha)+ L*sin(alpha);
            error =  Dt_future - DESIRED_LEFT_DIST;
            integral += error;
            // str_ang_vel = ;
            ackermann_msgs::AckermannDriveStamped drive_msg;
            drive_msg.drive.steering_angle = Kp*(error) + Ki*(integral) + Kd*(error - prev_error);
            // std::cout <<"M90: " << (int)((3*M_PI/2)/msg->angle_increment) << " " << "M30: " << (int)((7*M_PI/6)/msg->angle_increment) << " " <<"alpha: " << alpha*180/M_PI << " "<< "D: " << Dt_future <<  '\n';
            if(std::abs(alpha) > 0  && std::abs(alpha) < (10*M_PI)/180) {
                drive_msg.drive.speed = 1.5;
            } else if(std::abs(alpha) > (10*M_PI)/180 && std::abs(alpha) < (20*M_PI)/180) {
                drive_msg.drive.speed = 1.0;
            } else {
                drive_msg.drive.speed = 0.5;
            }
            drive_pub.publish(drive_msg);
            prev_error = error;
            // ros::spinOnce();
        }
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "wall_follow");
    WallFollow wf;
    ros::spin();
    return 0;
}