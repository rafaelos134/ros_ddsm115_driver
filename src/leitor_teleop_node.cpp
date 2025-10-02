#include "leitor_teleop_node.hpp"

class LeitorTeleopNode {
public:
    LeitorTeleopNode(ros::NodeHandle& nh) {
        

        sub_ = nh.subscribe("/cmd_vel", 10, &LeitorTeleopNode::teleopCallback, this);

        // Aqui você pode publicar para cada roda, se tiver tópicos separados
        pub_front_left_  = nh.advertise<std_msgs::Float64>("wheel/front_left/cmd_vel", 1);
        pub_front_right_ = nh.advertise<std_msgs::Float64>("wheel/front_right/cmd_vel", 1);
        pub_back_left_   = nh.advertise<std_msgs::Float64>("wheel/back_left/cmd_vel", 1);
        pub_back_right_  = nh.advertise<std_msgs::Float64>("wheel/back_right/cmd_vel", 1);

        ROS_INFO("No 'leitor_teleop_node' iniciado.");
    }

private:
    void teleopCallback(const geometry_msgs::Twist::ConstPtr& msg) {
        std_msgs::Float64 out;

        if (msg->linear.x > 0.0) {  
            out.data = 10.0;
            pub_front_left_.publish(out);
            pub_front_right_.publish(out);
            pub_back_left_.publish(out);
            pub_back_right_.publish(out);
            ROS_INFO("Frente pressionada -> rodas ligadas em 10");
        } else {
            out.data = 0.0;
            pub_front_left_.publish(out);
            pub_front_right_.publish(out);
            pub_back_left_.publish(out);
            pub_back_right_.publish(out);
        }
    }

    ros::Subscriber sub_;
    ros::Publisher pub_front_left_, pub_front_right_, pub_back_left_, pub_back_right_;
};

int main(int argc, char **argv) {

    ros::init(argc, argv, "leitor_teleop_node");

    ros::NodeHandle nh;


    ROS_INFO("teclado inciado");
    LeitorTeleopNode leitor(nh);

    ros::spin();
    return 0;
}
