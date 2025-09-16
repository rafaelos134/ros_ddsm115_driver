#include "ddsm115_communicator.hpp"

namespace ddsm115{

    /**
     * @brief Construct a new DDSM115Communicator::DDSM115Communicator object
     * 
     * @param port_name 
     */

    DDSM115Communicator::DDSM115Communicator(std::string port_name){
    
        struct termios tty; // comunication parameters

        this->port_name_ = port_name;
        this->ddsm115_state_ = DDSM115State::STATE_NORMAL;
        pthread_mutex_init(&port_mutex_, NULL); // inicializate the mutex
        ROS_INFO("Opening serial port %s", port_name_.c_str());
        this->port_fd_ = open(port_name_.c_str(), O_RDWR);

        if (port_fd_ <= 0){
            ROS_ERROR("Unable to open port %s", port_name_.c_str());
            ddsm115_state_ = DDSM115State::STATE_FAILED;
            return;
        }

        ROS_INFO("Getting tty attributes");

        if (tcgetattr(port_fd_, &tty) != 0){
            ROS_ERROR("Unable to get attributes for port %s", port_name_.c_str());
            ddsm115_state_ = DDSM115State::STATE_FAILED;
            return;
        }

        tty.c_cflag &= ~PARENB; // No parity
        tty.c_cflag &= ~CSTOPB; // 1 stop bit
        tty.c_cflag &= ~CSIZE; // Clear byte size bits
        tty.c_cflag |= CS8; // 8 bits per byte
        tty.c_cflag &= ~CRTSCTS; // Disable CTS/RTS
        tty.c_lflag = 0; // Make tty raw
        tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Turn off s/w flow ctrl
        tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL); // Disable any special handling of received bytes
        tty.c_oflag &= ~OPOST; // Prevent special interpretation of output bytes
        tty.c_oflag &= ~ONLCR; // Prevent conversion of newline to carriage return/line feed
        tty.c_cc[VTIME] = 1;  // Read timeout
        tty.c_cc[VMIN] = 0;
        // Set baud to 115200
        cfsetispeed(&tty, B115200);
        cfsetospeed(&tty, B115200);
        ROS_INFO("Setting tty attributes");

        if (tcsetattr(port_fd_, TCSANOW, &tty) != 0){
            ROS_ERROR("Error %i setting port attributes: %s\n", errno, strerror(errno));
            ddsm115_state_ = DDSM115State::STATE_FAILED;
            return;
        }
        }











}