// #include "leitor_teleop_node.hpp"

// class LeitorTeleopNode {
// public:
//     LeitorTeleopNode(ros::NodeHandle& nh) {
        

//         sub_ = nh.subscribe("/cmd_vel", 10, &LeitorTeleopNode::teleopCallback, this);

//         // Aqui você pode publicar para cada roda, se tiver tópicos separados
//         pub_front_left_  = nh.advertise<std_msgs::Float64>("wheel/front_left/cmd_vel", 1);
//         pub_front_right_ = nh.advertise<std_msgs::Float64>("wheel/front_right/cmd_vel", 1);
//         pub_back_left_   = nh.advertise<std_msgs::Float64>("wheel/back_left/cmd_vel", 1);
//         pub_back_right_  = nh.advertise<std_msgs::Float64>("wheel/back_right/cmd_vel", 1);

//         ROS_INFO("No 'leitor_teleop_node' iniciado.");
//     }

// private:
//     void teleopCallback(const geometry_msgs::Twist::ConstPtr& msg) {
//         std_msgs::Float64 out;

//         if (msg->linear.x > 0.0) {  
//             out.data = 10.0;
//             pub_front_left_.publish(out);
//             pub_front_right_.publish(out);
//             pub_back_left_.publish(out);
//             pub_back_right_.publish(out);
//             ROS_INFO("Frente pressionada -> rodas ligadas em 10");
//         } else {
//             out.data = 0.0;
//             pub_front_left_.publish(out);
//             pub_front_right_.publish(out);
//             pub_back_left_.publish(out);
//             pub_back_right_.publish(out);
//         }
//     }

//     ros::Subscriber sub_;
//     ros::Publisher pub_front_left_, pub_front_right_, pub_back_left_, pub_back_right_;
// };

// int main(int argc, char **argv) {

//     ros::init(argc, argv, "leitor_teleop_node");

//     ros::NodeHandle nh;


//     ROS_INFO("teclado inciado");
//     LeitorTeleopNode leitor(nh);

//     ros::spin();
//     return 0;
// }



#include "leitor_teleop_node.hpp"


class LeitorTeleopNode {
    public:
        LeitorTeleopNode(ros::NodeHandle& nh, ros::NodeHandle& pnh) {
            // Lê parâmetros (ou usa padrão)
            pnh.param<std::string>("input_topic", input_topic_, "/cmd_vel");
            pnh.param<std::string>("output_topic", output_topic_, "leitura/cmd_vel");
            pnh.param<int>("queue_size", queue_size_, 10);

            // Configura subscriber e publisher
            sub_ = nh.subscribe(input_topic_, queue_size_, &LeitorTeleopNode::teleopCallback, this);
            pub_ = nh.advertise<std_msgs::Float64>(output_topic_, 1);

            ROS_INFO_STREAM("No 'leitor_teleop_node' iniciado.");
            ROS_INFO_STREAM("Escutando em: " << input_topic_);
            ROS_INFO_STREAM("Publicando em: " << output_topic_);
        }

    private:
        void teleopCallback(const geometry_msgs::Twist::ConstPtr& msg) {
            ROS_INFO("Recebi do %s -> Linear X: %.2f | Angular Z: %.2f",
                    input_topic_.c_str(), msg->linear.x, msg->angular.z);

            std_msgs::Float64 out;
            out.data = msg->linear.x;  // só envia Linear X
            pub_.publish(out);
        }

        ros::Subscriber sub_;
        ros::Publisher pub_;
        std::string input_topic_;
        std::string output_topic_;
        int queue_size_;
    };

int main(int argc, char **argv) {

    //melhorar e simplificar o codigo
    ros::init(argc, argv, "leitor_teleop_node");
    ros::NodeHandle node;
    

    ROS_INFO("INCIADO");

    ros::Rate loop_rate(1); 

    while (ros::ok()) {
        ROS_INFO("No leitor_teleop_node rodando...");
        ros::spinOnce();
        loop_rate.sleep();
    }



    // LeitorTeleopNode node(node, pnh);
    // ros::NodeHandle pnh("~");

    ros::spin();
    return 0;
}
