#include "ddsm115_communicator.hpp"

namespace ddsm115{

/**
 * @brief Construct a new DDSM115Communicator::DDSM115Communicator object
 * 
 * @param port_name 
 */
DDSM115Communicator::DDSM115Communicator(std::string port_name){
  struct termios tty;

  port_name_ = port_name;
  ddsm115_state_ = DDSM115State::STATE_NORMAL;
  pthread_mutex_init(&port_mutex_, NULL);
  ROS_INFO("Opening serial port %s", port_name_.c_str());
  port_fd_ = open(port_name_.c_str(), O_RDWR);

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


/**
 * @brief Disconnect from DDSM115
 * 
 */
void DDSM115Communicator::disconnect()
{
  close(port_fd_);
}

/**
 * @brief Set wheel mode
 * 
 * @param wheel_id 
 * @param mode 
 */
void DDSM115Communicator::setWheelMode(int wheel_id, DDSM115Mode mode)
{
  uint8_t mode_cmd[] = {
    (uint8_t)wheel_id, DDSM115Command::COMMAND_SWITCH_MODE, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, mode
  };
  lockPort();
  write(port_fd_, &mode_cmd, sizeof(mode_cmd));
  unlockPort();
}

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
                          0x00, // Acceleration time
                          0x00,
                          0x00,
                          0x00 };
  
  // uint8_t drive_response[] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };



  // drive_cmd[9] = maximCrc8(drive_cmd, 9);
  // lockPort();
  
  // write(port_fd_, &drive_cmd, sizeof(drive_cmd));
  // int num_bytes = read(port_fd_, &drive_response[1], 1);

  // ROS_INFO("Read %d bytes", num_bytes);







  // Array para armazenar a resposta completa da roda
  uint8_t drive_response[10] = {0}; // Inicializado com zeros

  // 1. Calcula e insere o CRC no último byte do comando
  drive_cmd[9] = maximCrc8(drive_cmd, 9);

  lockPort(); // Presumindo que esta função gerencia o acesso concorrente à porta

  // 2. Escreve o comando e VERIFICA o retorno
  ssize_t bytes_written = write(port_fd_, drive_cmd, sizeof(drive_cmd));

  if (bytes_written != sizeof(drive_cmd)) {
      perror("Erro ao escrever na porta serial");
      // Lidar com o erro aqui...
      unlockPort();
  }

  // 3. (MUITO IMPORTANTE) Aguarda um pequeno tempo para a roda processar e responder
  usleep(10000); // 10.000 microssegundos = 10 milissegundos. Ajuste se necessário.

  // 4. Lê a resposta COMPLETA (10 bytes)
  //    - Passe o ponteiro para o início do array: drive_response
  //    - Peça para ler o tamanho total do array: sizeof(drive_response)
  ssize_t num_bytes = read(port_fd_, drive_response, sizeof(drive_response)); //meu problema está exatemente no read

  if (num_bytes < 0) {
    perror("Erro ao ler da porta serial");
  } else if (num_bytes == 0) {
      printf("Nenhum dado recebido. Timeout?\n");
  } else {
      printf("Recebidos %zd bytes:\n", num_bytes);
      for (int i = 0; i < sizeof(drive_response); ++i) {
          printf("Byte %d: 0x%02X\n", i, drive_response[i]);
      }

      // Aqui você pode adicionar a verificação do CRC da resposta para garantir
      // que os dados recebidos são válidos.
  }





    // original
  // tcdrain(port_fd_);
  // int total_num_bytes = 0;
  // int num_bytes = 0;

  // for (int i = 0; i < sizeof(drive_response); i++) {
  //   num_bytes = read(port_fd_, &drive_response[i], 1);
  //   if (num_bytes <= 0) {
  //     break;
  //   }
  //   total_num_bytes += num_bytes;
  // }



  unlockPort();
  
  // processing posible error
  // if (num_bytes < 0){
  //   ROS_ERROR("Error reading DDSM115 response for wheel id %d", wheel_id);
  //   result.result = DDSM115State::STATE_FAILED;
  //   return result;
  // }
  // if (total_num_bytes < 10){
  //   // ROS_WARN("Timeout reading DDSM115 response for wheel id %d", wheel_id);
  //   // ROS_INFO("Received %d bytes", total_num_bytes);
  //   // for (int i = 0; i < 10; i++) {
  //   //   ROS_INFO("%02x", drive_response[i]);
  //   // }
  //   result.result = DDSM115State::STATE_FAILED;
  //   return result;
  // }
  // if (drive_response[0] != wheel_id){
  //   ROS_INFO("Received response for wheel %d instead of %d", drive_response[0], wheel_id);
  //   result.result = DDSM115State::STATE_FAILED;
  //   return result;
  // }
  // if (drive_response[9] != maximCrc8(drive_response, 9)){
  //   ROS_ERROR("CRC error in response from wheel id %d", wheel_id);
  //   result.result = DDSM115State::STATE_FAILED;
  //   return result;
  // }
  
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

  ROS_INFO("velocity = %f", (double)drive_velocity);

  // result.velocity = (double)20;


  //minha ideia ta certa mas o problema são o calculo passado

  last_responses[wheel_id] = result;












  
  return result;
}

// // Esta função agora envia um comando de velocidade e lê a resposta com o estado real.
// ddsm115_drive_response DDSM115Communicator::getWheelRPM(int wheel_id, int16_t target_rpm) {

//   ddsm115_drive_response result;
  
//   // Protocolo 1: drive motor to rotate (Comando 0x64)
//   uint8_t command_frame[10] = {
//       (uint8_t)wheel_id,
//       0x64, // <-- MUDANÇA CRUCIAL: Comando para mover e obter feedback
//       0x00,
//       0x00,
//       (uint8_t)(target_rpm >> 8),   // Velocity high 8 bits
//       (uint8_t)(target_rpm & 0xFF), // Velocity low 8 bits
//       0x00, // Acceleration time (0x00 para usar o padrão)
//       0x00, // Brake (0x00 para não frear)
//       0x00, // Reservado
//       0x00  // Espaço para o CRC
//   };

//   // Calcula o CRC sobre os 9 primeiros bytes
//   command_frame[9] = maximCrc8(command_frame, 9);

//   uint8_t response_frame[10] = {0};

//   lockPort();
//   tcflush(port_fd_, TCIFLUSH); 
  
//   write(port_fd_, command_frame, sizeof(command_frame));

//   // Lê a resposta do motor
//   int num_bytes = read(port_fd_, response_frame, sizeof(response_frame));
  
//   unlockPort();

//   // --- Verificações de erro (essenciais) ---
//   if (num_bytes != 10) {
//     ROS_WARN("Falha na leitura para roda %d: esperava 10 bytes, recebeu %d", wheel_id, num_bytes);
//     result.result = DDSM115State::STATE_FAILED;
//     return result;
//   }
//   if (response_frame[9] != maximCrc8(response_frame, 9)) {
//     ROS_WARN("Erro de CRC na resposta da roda %d", wheel_id);
//     result.result = DDSM115State::STATE_FAILED;
//     return result;
//   }

//   // --- Decodificação da RESPOSTA (Motor Feedback) ---
//   // A estrutura da resposta é a mesma, mas agora os dados são válidos!
//   int16_t feedback_current  = (response_frame[2] << 8) | response_frame[3];
//   int16_t feedback_velocity = (response_frame[4] << 8) | response_frame[5];
//   uint16_t feedback_position = (response_frame[6] << 8) | response_frame[7];
//   uint8_t error_code        = response_frame[8];

//   // --- Conversão com as UNIDADES CORRETAS da documentação ---
//   // O valor da velocidade já é RPM!
//   result.velocity = (double)feedback_velocity;
//   // Posição: 0~32767 -> 0~360 graus
//   result.position = (double)feedback_position * (360.0 / 32767.0);
//   // Corrente: -32767~32767 -> -8A~8A
//   result.current  = (double)feedback_current * (8.0 / 32767.0);
//   result.result   = DDSM115State::STATE_NORMAL;

//   if (error_code != 0) {
//       ROS_ERROR("Motor da roda %d reportou erro: 0x%02X", wheel_id, error_code);
//   }
  
//   ROS_INFO("Roda %d: Cmd RPM=%d, Feedback RPM=%.2f", wheel_id, target_rpm, result.velocity);

//   return result;
// }










































// ddsm115_drive_response DDSM115Communicator::getWheelRPM(int wheel_id) {

//   ddsm115_drive_response result;

//   // Monta o pacote de request de feedback
//   uint8_t read_cmd[] = {
//       (uint8_t)wheel_id,
//       DDSM115Command::COMMAND_GET_OTHER_FEEDBACK, // <-- aqui usamos 0x74
//       0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
//   };

//   uint8_t read_response[10] = {0};

//   // // Calcula CRC
//   read_cmd[9] = maximCrc8(read_cmd, 9);

//   lockPort();
//   write(port_fd_, &read_cmd, sizeof(read_cmd));

//   int total_num_bytes = 0;
//   int num_bytes = 0;
//   for (int i = 0; i < sizeof(read_response); i++) {
//     num_bytes = read(port_fd_, &read_response[i], 1);
//     if (num_bytes <= 0) break;
//     total_num_bytes += num_bytes;
//   }
//   unlockPort();

//   // Os dois primeiros matam o codigo
//   // if (num_bytes < 0 || total_num_bytes < 10) {
//   //   result.result = DDSM115State::STATE_FAILED;
//   //   return result;
//   // }
//   // if (read_response[0] != wheel_id) {
//   //   result.result = DDSM115State::STATE_FAILED;
//   //   return result;
//   // }


//   if (read_response[9] != maximCrc8(read_response, 9)) {
//     result.result = DDSM115State::STATE_FAILED;
//     return result;
//   }

//   // Decodificação igual ao setWheelRPM
//   int16_t drive_current  = (read_response[2] << 8) | read_response[3];
//   int16_t drive_velocity = (read_response[4] << 8) | read_response[5];
//   uint16_t drive_position = (read_response[6] << 8) | read_response[7];





//   result.velocity = (double)drive_velocity;
//   result.position = (double)drive_position * (360.0 / 32767.0);
//   result.current  = (double)drive_current * (8.0 / 32767.0);
//   result.result   = DDSM115State::STATE_NORMAL;



  
//   ROS_INFO("Wheel %d velocity = %f", wheel_id, result.velocity);

//   return result;
// }



//   // result.velocity = (double)20.0;
//   // result.position = (double)20.0;
//   // result.current  = (double)20.0;
//   // result.result   = DDSM115State::STATE_NORMAL;







// new
/**
 * @brief get wheel sppeds
 * 
 * @param wheel_id 
 * @return velocity whell
 */
ddsm115_drive_response DDSM115Communicator::getWheelRPM(int wheel_id){

  ddsm115_drive_response response = last_responses[wheel_id];
  // ROS_INFO("velocity = %f", response.velocity);

  // std::cout << response.velocity << std::endl;

  return response;

}


/**
 * @brief Lock communication mutex
 * 
 */
void DDSM115Communicator::lockPort()
{
  pthread_mutex_lock(&port_mutex_);
}

/**
 * @brief Unlock communication mutex
 * 
 */
void DDSM115Communicator::unlockPort()
{
  pthread_mutex_unlock(&port_mutex_);
}

/**
 * @brief Get DDSM115 communication state
 * 
 * @return DDSM115State 
 */
DDSM115State DDSM115Communicator::getState()
{
  return ddsm115_state_;
}

/**
 * @brief Maxim CRC8 calculator
 * 
 * @param data 
 * @param size 
 * @return uint8_t 
 */
uint8_t DDSM115Communicator::maximCrc8(uint8_t* data, const unsigned int size)
{
  uint8_t crc = 0;
  for (unsigned int i = 0; i < size; ++i)
  {
    uint8_t inbyte = data[i];
    for (unsigned char j = 0; j < 8; ++j)
    {
      uint8_t mix = (crc ^ inbyte) & 0x01;
      crc >>= 1;
      if (mix)
        crc ^= 0x8C;
      inbyte >>= 1;
    }
  }
  return crc;
}

}  // namespace ddsm115
