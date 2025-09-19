  struct ddsm115_drive_response{
    uint8_t mode;
    double current;
    double velocity;
    double position;
    uint8_t error;
    DDSM115State result;
  };

  /**
   * @brief A class providing a communication interface to DDSM115 wheels
   * 
   */
  class DDSM115Communicator{
    public:
      DDSM115Communicator(std::string port_name);
      void disconnect();
      void setWheelMode(int wheel_id, DDSM115Mode mode);
      ddsm115_drive_response setWheelRPM(int wheel_id, double rpm);
      DDSM115State getState();
      ddsm115_drive_response getWheelRPM(int wheel_id);

    private:
      std::map<int, ddsm115_drive_response> last_responses;
      std::string port_name_;
      int port_fd_;
      pthread_mutex_t port_mutex_;
      DDSM115State ddsm115_state_;
      void lockPort();
      void unlockPort();
      uint8_t maximCrc8(uint8_t* data, const unsigned int size);
  };





/**
 * @brief Drive DDSM115 wheel and get it's feedback
 * 
 * @param wheel_id 
 * @param rpm 
 * @return ddsm115_drive_response        "as mudancas devem ser aq" novo
 */
ddsm115_drive_response DDSM115Communicator::setWheelRPM(int wheel_id, double rpm){
  ddsm115_drive_response result;
  int16_t rpm_value = (int16_t)rpm;
  
  // TODO: this implementation of data encoding is not endian safe
  uint8_t drive_cmd[] = { (uint8_t)wheel_id,
                          DDSM115Command::COMMAND_DRIVE_MOTOR,
                          (uint8_t)((rpm_value >> 8) & 0xFF),
                          (uint8_t)(rpm_value & 0xFF),
                          0x00,
                          0x00,
                          0x00,
                          0x00,
                          0x00,
                          0x00 };
  
  uint8_t drive_response[] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
  
  drive_cmd[9] = maximCrc8(drive_cmd, 9);
  lockPort();
  write(port_fd_, &drive_cmd, sizeof(drive_cmd));
  // tcdrain(port_fd_);
  int total_num_bytes = 0;
  int num_bytes = 0;
  for (int i = 0; i < sizeof(drive_response); i++) {
    num_bytes = read(port_fd_, &drive_response[i], 1);
    if (num_bytes <= 0) {
      break;
    }
    total_num_bytes += num_bytes;
  }
  
  unlockPort();
  
  // processing posible error
  if (num_bytes < 0){
    ROS_ERROR("Error reading DDSM115 response for wheel id %d", wheel_id);
    result.result = DDSM115State::STATE_FAILED;
    return result;
  }
  if (total_num_bytes < 10){
    // ROS_WARN("Timeout reading DDSM115 response for wheel id %d", wheel_id);
    // ROS_INFO("Received %d bytes", total_num_bytes);
    // for (int i = 0; i < 10; i++) {
    //   ROS_INFO("%02x", drive_response[i]);
    // }
    result.result = DDSM115State::STATE_FAILED;
    return result;
  }
  if (drive_response[0] != wheel_id){
    ROS_INFO("Received response for wheel %d instead of %d", drive_response[0], wheel_id);
    result.result = DDSM115State::STATE_FAILED;
    return result;
  }
  if (drive_response[9] != maximCrc8(drive_response, 9)){
    ROS_ERROR("CRC error in response from wheel id %d", wheel_id);
    result.result = DDSM115State::STATE_FAILED;
    return result;
  }
  
  // TODO: this implementation of data decoding is not endian safe
  int16_t drive_current = 0;
  int16_t drive_velocity = 0;
  uint16_t drive_position = 0;
  uint8_t swap;
  swap = drive_response[4];
  drive_response[4] = drive_response[5];
  drive_response[5] = swap;
  drive_current = (drive_response[2] << 8) + drive_response[3];
  drive_velocity = (drive_response[4] << 8) + drive_response[5];
  drive_position = (drive_response[6] << 8) + drive_response[7];
  
  ROS_INFO("drive %d : velocity = %d, position = %d", drive_response[0], drive_velocity, drive_position);
  result.velocity = (double)drive_velocity;
  result.position = (double)drive_position * (360.0 / 32767.0);
  result.current = (double)drive_current * (8.0 / 32767.0);
  result.result = DDSM115State::STATE_NORMAL;

  last_responses[wheel_id] = result;
  
  return result;
}

// new
/**
 * @brief get wheel sppeds
 * 
 * @param wheel_id 
 * @return velocity whell
 */
ddsm115_drive_response DDSM115Communicator::getWheelRPM(int wheel_id){

  ddsm115_drive_response response = last_responses[wheel_id];

  // std::cout << response.velocity << std::endl;

  return response;

}


/**
 * @brief Call back for all wheels target velocity topics
 * 
 * @param target_velocity_msg 
 * @param wheel_name 
 */
void wheelTargetVelocityCallback(const std_msgs::Float64::ConstPtr& target_velocity_msg, std::string wheel_name){
  std_msgs::Float64 velocity_msg;
  std_msgs::Float64 angle_msg;
  std_msgs::Float64 current_msg;
  ROS_INFO("I heard: [%f] for wheel \"%s\"", target_velocity_msg->data, wheel_name.c_str());
  int wheel_index = wheelIndexByName(wheel_name);

  // std::cout << wheel_index << std::endl;
  
  if (wheel_index < 0){
    ROS_WARN("Received target velocity for unknown wheel %s", wheel_name.c_str());
    return;
  }

  ddsm115::ddsm115_drive_response response = ddsm115_communicator->setWheelRPM(
      wheel_ids_list[wheel_index], vel2Rpm(target_velocity_msg->data) * wheel_directions[wheel_index]);

}


/**
 * @brief Main of the node
 * 
 * @param argc 
 * @param argv 
 * @return int 
 */
int main(int argc, char** argv){
  std::string port_name;
  ros::init(argc, argv, "ros_ddsm115_driver_node");
  ros::NodeHandle node("~");
  ROS_INFO("DDSM115 starting up");
  // Check if we have all required parameters
  if (!node.hasParam("wheel_names"))
  {
    ROS_ERROR("Can't run without wheel_names parameter");
    return -1;
  }
  if (!node.hasParam("wheel_ids"))
  {
    ROS_ERROR("Can't run without wheel_ids parameter");
    return -1;
  }
  if (!node.hasParam("wheel_directions"))
  {
    ROS_ERROR("Can't run without wheel_directions parameter");
    return -1;
  }
  // Get all parameters
  node.param<std::string>("port_name", port_name, DEFAULT_SERIAL_PORT);
  node.getParam("wheel_names", wheel_names_list);
  node.getParam("wheel_ids", wheel_ids_list);
  node.getParam("wheel_directions", wheel_directions_list);

  // Check that wheel array parameters have equial length (specify all wheels)
  if (!(wheel_names_list.size() == wheel_ids_list.size() && wheel_ids_list.size() == wheel_directions_list.size())){
    ROS_ERROR("wheel_names, wheel_ids and wheel_directions must be of an equal size");
    return -1;
  }
  // Check that wheel names and ids are distinct (can't run with 2 wheels with the same name or id)
  if (!stringsAreDistinct(wheel_names_list)){
    ROS_ERROR("wheel names must be distinct");
    return -1;
  }
  if (!intsAreDistinct(wheel_ids_list)){
    ROS_ERROR("wheel ids must be distinct");
    return -1;
  }

  // Open DDSM115 communication interface
  ddsm115_communicator = new ddsm115::DDSM115Communicator(port_name);
  if (ddsm115_communicator->getState() != ddsm115::DDSM115State::STATE_NORMAL){
    ROS_ERROR("Failed to initialize DDSM115 communication");
    return -1;
  }

  // Do all setup for every wheel
  for (int i = 0; i < (int)wheel_names_list.size(); i++){
    // Check that parameters are valid
    if (wheel_names_list[i].length() == 0){
      ROS_ERROR("wheel name can't be empty");
      return -1;
    }
    if (wheel_ids_list[i] <= 0){
      ROS_ERROR("wheel id must be >0");
      return -1;
    }

    ROS_INFO("Adding wheel %s id %d and direction %s", wheel_names_list[i].c_str(), wheel_ids_list[i],
             wheel_directions_list[i].c_str());
    
    // Create wheel direction multiplyers array
    if (wheel_directions_list[i] == "forward"){
      wheel_directions.push_back(1);
    }
    else{
      wheel_directions.push_back(-1);
    }


    /*
      Create wheel target velocity subscriber, bind with wheel name as additional parameter
      to handle all wheels with single callback
    */
    ros::Subscriber velocity_sub = node.subscribe<std_msgs::Float64>("/" + wheel_names_list[i] + "/target_velocity", 1,
                                          boost::bind(&wheelTargetVelocityCallback, _1, wheel_names_list[i]));
    wheel_velocity_subs.push_back(velocity_sub);

    // Create publisher for wheel velocity
    ros::Publisher velocity_pub = node.advertise<std_msgs::Float64>("/" + wheel_names_list[i] + "/current_velocity", 1);
    wheel_velocity_pubs.push_back(velocity_pub);
    
    // Create publisher for wheel angle
    ros::Publisher angle_pub = node.advertise<std_msgs::Float64>("/" + wheel_names_list[i] + "/angle", 1);
    wheel_angle_pubs.push_back(angle_pub);
    
    // Create publisher for wheel current consumption
    ros::Publisher current_pub = node.advertise<std_msgs::Float64>("/" + wheel_names_list[i] + "/current", 1);
    wheel_current_pubs.push_back(current_pub);
    
    // Set wheel mode to velocity loop
    ddsm115_communicator->setWheelMode(wheel_ids_list[i], ddsm115::DDSM115Mode::VELOCITY_LOOP);
  }



  // get and pub velocities at 1 sec
  ros::Rate rate(1); 

  while (ros::ok()) {
    for (int i = 0; i < (int)wheel_names_list.size(); i++) {
      ddsm115::ddsm115_drive_response response = ddsm115_communicator->getWheelRPM(wheel_ids_list[i]);

      // std::cout << wheel_ids_list[i] << std::endl;


      std_msgs::Float64 vel_msg;
      vel_msg.data = rpm2Vel(response.velocity) * wheel_directions[wheel_ids_list[i]];
      // ROS_INFO("Received velocity command: %d", i);

      // wheel_velocity_pubs[wheel_ids_list[i]].publish(vel_msg);
      wheel_velocity_pubs[i].publish(vel_msg); // erro aq
        
    }

    ros::spinOnce();
    rate.sleep();
          
  }
}






std::map<int, double> last_sent_rpm;

void wheelTargetVelocityCallback(const std_msgs::Float64::ConstPtr& target_velocity_msg, std::string wheel_name){
  // ... (código do callback como antes) ...
  
  double target_rpm = vel2Rpm(target_velocity_msg->data) * wheel_directions[wheel_index];
  
  // Salva o último RPM comandado
  last_sent_rpm[wheel_ids_list[wheel_index]] = target_rpm;

  // Envia o comando (opcional, pois o loop principal também enviará)
  ddsm115_communicator->setWheelRPM(wheel_ids_list[wheel_index], target_rpm);
}

 // Inicializa o último RPM enviado para 0 para todas as rodas
  for (int wheel_id : wheel_ids_list) {
      last_sent_rpm[wheel_id] = 0.0;
  }





// ... (código da classe DDSM115Communicator e outras funções) ...

// **Pequeno ajuste:** Você precisa de uma forma de saber qual a última velocidade enviada.
// Vamos armazenar isso em um mapa.
std::map<int, double> last_sent_rpm;

void wheelTargetVelocityCallback(const std_msgs::Float64::ConstPtr& target_velocity_msg, std::string wheel_name){
  // ... (código do callback como antes) ...
  
  double target_rpm = vel2Rpm(target_velocity_msg->data) * wheel_directions[wheel_index];
  
  // Salva o último RPM comandado
  last_sent_rpm[wheel_ids_list[wheel_index]] = target_rpm;

  // Envia o comando (opcional, pois o loop principal também enviará)
  ddsm115_communicator->setWheelRPM(wheel_ids_list[wheel_index], target_rpm);
}


int main(int argc, char** argv){
  // ... (todo o seu código de setup) ...

  // Inicializa o último RPM enviado para 0 para todas as rodas
  for (int wheel_id : wheel_ids_list) {
      last_sent_rpm[wheel_id] = 0.0;
  }

  ros::Rate rate(10); // Aumentar a frequência para uma leitura mais útil (ex: 10 Hz)

  while (ros::ok()) {
    for (int i = 0; i < (int)wheel_names_list.size(); i++) {
      int wheel_id = wheel_ids_list[i];

      // Pede ativamente uma atualização, reenviando o último comando de RPM
      ddsm115::ddsm115_drive_response response = ddsm115_communicator->setWheelRPM(wheel_id, last_sent_rpm[wheel_id]);

      if (response.result == ddsm115::DDSM115State::STATE_NORMAL) {
        std_msgs::Float64 vel_msg;
        // CORREÇÃO DE BUG: O índice para wheel_directions deve ser `i`, não `wheel_id`.
        vel_msg.data = rpm2Vel(response.velocity) * wheel_directions[i]; 
        wheel_velocity_pubs[i].publish(vel_msg);
        
        // Publique também os outros dados se desejar
        // std_msgs::Float64 angle_msg;
        // angle_msg.data = response.position;
        // wheel_angle_pubs[i].publish(angle_msg);
      }
    }

    ros::spinOnce();
    rate.sleep();
  }
}