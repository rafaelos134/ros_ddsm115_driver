#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/Float64.h"
#include "ros_ddsm115_driver.hpp"


class LeitorTeleopNode {
    public:
        LeitorTeleopNode(ros::NodeHandle& nh) {
            // Speeds
            step_ = 0.02;
            maxSpeed_ = 0.2;
            minSpeed_ = -0.2;

            // Robot
            wheelRadius_ = 0.05;
            wheelSeparation_ = 0.22;

            currentSpeed_ = 0.0; // precisa pegar do ros



            // Configura subscriber e publisher
            sub_ = nh.subscribe("/cmd_vel", 10, &LeitorTeleopNode::teleopCallback, this);

            pub_front_left_  = nh.advertise<std_msgs::Float64>("/front_left_wheel/target_velocity", 1);
            pub_front_right_ = nh.advertise<std_msgs::Float64>("/front_right_wheel/target_velocity", 1);
            pub_back_left_   = nh.advertise<std_msgs::Float64>("/rear_left_wheel/target_velocity", 1);
            pub_back_right_  = nh.advertise<std_msgs::Float64>("/rear_right_wheel/target_velocity", 1);

            ROS_INFO("No 'leitor_teleop_node' iniciado.");
        }

    private:
    
        ros::Subscriber sub_;
        ros::Publisher pub_front_left_, pub_front_right_, pub_back_left_, pub_back_right_;

        double wheelSeparation_, wheelRadius_;
        double step_, maxSpeed_, minSpeed_;
        double currentSpeed_;

        
        void teleopCallback(const geometry_msgs::Twist::ConstPtr& msg) {
            
            std_msgs::Float64 out;

            if (msg->linear.x > 0.0) {
                currentSpeed_ += step_;
                if (currentSpeed_ > maxSpeed_){
                    currentSpeed_ = maxSpeed_;
                }
            } 
            else if (msg->linear.x < 0.0) {
                currentSpeed_ -= step_;
                if (currentSpeed_ < minSpeed_){

               currentSpeed_ = minSpeed_; } 
            }else {
            // desacelera gradualmente até parar
            if (currentSpeed_ > step_)
                currentSpeed_ -= step_;
            else if (currentSpeed_ < -step_)
                currentSpeed_ += step_;
            else
                currentSpeed_ = 0.0;
            }

            
            double w = msg->angular.z;
            double v = currentSpeed_;
            

            // ROS_INFO("Velocidade linear: %.2f", currentSpeed_);
            // ROS_INFO("Velocidade angular: %.2f", w);
            
            double w_right = ((2*v) + (w*wheelSeparation_))/(2*wheelRadius_);
            double w_left  = ((2*v) - (w*wheelSeparation_))/(2*wheelRadius_);

            // ROS_INFO("Roda direita: %.2f", w_right);
            // ROS_INFO("Roda esquerda: %.2f", w_left);

            // ROS_INFO("---------------");

            std_msgs::Float64 left_msg, right_msg;
            left_msg.data  = w_right;
            right_msg.data = w_left ;


            // Publish values in ROS
            pub_front_left_.publish(left_msg);
            pub_back_left_.publish(left_msg);

            pub_front_right_.publish(right_msg);
            pub_back_right_.publish(right_msg);

        }
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "leitor_teleop_node");
    ros::NodeHandle nh;

    LeitorTeleopNode leitor(nh);

    ros::spin();
    return 0;
}

//olhar na aula
// double v_right = linear + (angular * wheel_separation_ / 2.0);
// double v_left  = linear - (angular * wheel_separation_ / 2.0);
// ddsm115::ddsm115_drive_response response = ddsm115_communicator->getWheelRPM(wheel_ids_list[wheel_index]);
// current_speed_ = 0.0;
// parametros fisicos
// nh.param("wheel_separation", wheel_separation_, 0.265); // distância entre rodas (m)
// nh.param("wheel_radius", wheel_radius_, 0.05);         // raio da roda (m)
// nh.param("step", step_, 5.0);                          // incremento de velocidade
// nh.param("max_speed", max_speed_, 100.0);
// nh.param("min_speed", min_speed_, -100.0);
