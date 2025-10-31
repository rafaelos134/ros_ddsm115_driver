#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/Float64.h"

const double step = 5.0; 
const double max_speed = 100.0;
const double min_speed = -100.0;



class LeitorTeleopNode {
public:
    LeitorTeleopNode(ros::NodeHandle& nh) {

        current_speed_ = 0.0;
        //parametros fisicos
        nh.param("wheel_separation", wheel_separation_, 0.265); // distância entre rodas (m)
        nh.param("wheel_radius", wheel_radius_, 0.05);         // raio da roda (m)
        nh.param("step", step_, 5.0);                          // incremento de velocidade
        nh.param("max_speed", max_speed_, 100.0);
        nh.param("min_speed", min_speed_, -100.0);


        // Configura subscriber e publisher
        sub_ = nh.subscribe("/cmd_vel", 10, &LeitorTeleopNode::teleopCallback, this);

        pub_front_left_  = nh.advertise<std_msgs::Float64>("/front_left_wheel/target_velocity", 1);
        pub_front_right_ = nh.advertise<std_msgs::Float64>("/front_right_wheel/target_velocity", 1);
        pub_back_left_   = nh.advertise<std_msgs::Float64>("/rear_left_wheel/target_velocity", 1);
        pub_back_right_  = nh.advertise<std_msgs::Float64>("/rear_right_wheel/target_velocity", 1);

        ROS_INFO("No 'leitor_teleop_node' iniciado.");
    }

private:
    double wheel_separation_, wheel_radius_;
    double step_, max_speed_, min_speed_;
    double current_speed_;

    void teleopCallback(const geometry_msgs::Twist::ConstPtr& msg) {
        std_msgs::Float64 out;

        if (msg->linear.x > 0.0) {
            current_speed_ += step;
            if (current_speed_ > max_speed) current_speed_ = max_speed;
        } 
        else if (msg->linear.x < 0.0) {
            current_speed_ -= step;
            if (current_speed_ < min_speed) current_speed_ = min_speed;
        } 
        else {
            // Desaceleração suave quando nenhuma tecla é pressionada
            if (current_speed_ > 0)
                current_speed_ -= step;
            else if (current_speed_ < 0)
                current_speed_ += step;

            // Impede que oscile próximo de 0
            if (fabs(current_speed_) < step)
                current_speed_ = 0.0;
        }

        double linear = current_speed_;
        double angular = msg->angular.z;


        //olhar na aula
        double v_right = linear + (angular * wheel_separation_ / 2.0);
        double v_left  = linear - (angular * wheel_separation_ / 2.0);

        std_msgs::Float64 left_msg, right_msg;
        left_msg.data  = v_left;
        right_msg.data = v_right;

        pub_front_left_.publish(left_msg);
        pub_back_left_.publish(left_msg);

        pub_front_right_.publish(right_msg);
        pub_back_right_.publish(right_msg);

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
