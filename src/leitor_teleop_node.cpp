#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/Float64.h"

double current_speed = 0.0;
const double step = 5.0; 
const double max_speed = 100.0;
const double min_speed = -100.0;



class LeitorTeleopNode {
public:
    LeitorTeleopNode(ros::NodeHandle& nh) {
        // Configura subscriber e publisher
        sub_ = nh.subscribe("/cmd_vel", 10, &LeitorTeleopNode::teleopCallback, this);

        pub_front_left_  = nh.advertise<std_msgs::Float64>("/front_left_wheel/target_velocity", 1);
        pub_front_right_ = nh.advertise<std_msgs::Float64>("/front_right_wheel/target_velocity", 1);
        pub_back_left_   = nh.advertise<std_msgs::Float64>("/rear_left_wheel/target_velocity", 1);
        pub_back_right_  = nh.advertise<std_msgs::Float64>("/rear_right_wheel/target_velocity", 1);

        ROS_INFO("No 'leitor_teleop_node' iniciado.");
    }

private:
    void teleopCallback(const geometry_msgs::Twist::ConstPtr& msg) {
        std_msgs::Float64 out;

        if (msg->linear.x > 0.0) {
            // Frente -> aumenta velocidade
            current_speed += step;
            if (current_speed > max_speed) current_speed = max_speed;
            ROS_INFO("Frente pressionada -> velocidade atual %.2f", current_speed);
        } 
        else if (msg->linear.x < 0.0) {
            // Trás -> diminui velocidade
            current_speed -= step;
            if (current_speed < min_speed) current_speed = min_speed;
            ROS_INFO("Trás pressionada -> velocidade atual %.2f", current_speed);
        } 
        else {
            // Nenhuma tecla -> mantém velocidade (ou zera se quiser)
            // current_speed = 0; // <-- descomente se quiser parar ao soltar tecla
        }

        out.data = current_speed;
        pub_front_left_.publish(out);
        pub_front_right_.publish(out);
        pub_back_left_.publish(out);
        pub_back_right_.publish(out);
    }

    ros::Subscriber sub_;
    ros::Publisher pub_front_left_, pub_front_right_, pub_back_left_, pub_back_right_;
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "leitor_teleop_node");
    ros::NodeHandle nh;

    LeitorTeleopNode leitor(nh);

    ros::spin();
    return 0;
}
