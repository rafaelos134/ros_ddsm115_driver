#include "ros/ros.h"
#include "sensor_msgs/Joy.h"
#include "std_msgs/Float64.h"

class JoyTeleop {
public:
    JoyTeleop(ros::NodeHandle& nh) {
        sub_ = nh.subscribe("/joy", 10, &JoyTeleop::joyCallback, this);

        pub_front_left_  = nh.advertise<std_msgs::Float64>("wheel/front_left/cmd_vel", 1);
        pub_front_right_ = nh.advertise<std_msgs::Float64>("wheel/front_right/cmd_vel", 1);
        pub_back_left_   = nh.advertise<std_msgs::Float64>("wheel/back_left/cmd_vel", 1);
        pub_back_right_  = nh.advertise<std_msgs::Float64>("wheel/back_right/cmd_vel", 1);
    }

private:
    void joyCallback(const sensor_msgs::Joy::ConstPtr& joy) {
        std_msgs::Float64 out;
        
        double eixo_frente = joy->axes[1];  

        if (eixo_frente > 0.5) { 
            out.data = 10.0;
        } else if (eixo_frente < -0.5) {
            out.data = -10.0;
        } else {
            out.data = 0.0;
        }

        pub_front_left_.publish(out);
        pub_front_right_.publish(out);
        pub_back_left_.publish(out);
        pub_back_right_.publish(out);
    }

    ros::Subscriber sub_;
    ros::Publisher pub_front_left_, pub_front_right_, pub_back_left_, pub_back_right_;
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "joy_teleop_node");
    ros::NodeHandle nh;

    ROS_INFO("controle inciado")
    JoyTeleop teleop(nh);

    ros::spin();
    return 0;
}
