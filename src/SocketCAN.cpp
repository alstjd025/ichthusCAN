/**
 * @file
 * This file implements functions to receive
 * and transmit CAN frames via SocketCAN.
 */

#ifndef MINGW

#include <SocketCAN.h>
#include <stdio.h>
// strncpy
#include <string.h>
// close
#include <unistd.h>
// socket
#include <sys/socket.h>
// SIOCGIFINDEX
#include <sys/ioctl.h>

/*
00000001 00100000 00000000 01010001 01111110 00100111 00111111 00111101
* https://github.com/JCube001/socketcan-demo
* http://blog.mbedded.ninja/programming/operating-systems/linux/how-to-use-socketcan-with-c-in-linux
* https://github.com/linux-can/can-utils/blob/master/candump.c
*/

/*
#include <endian.h>
#include <sys/socket.h>
#include <sys/types.h>
*/


// Constructer For SocketCAN
// 
SocketCAN::SocketCAN(DeviceType type)
  :CANAdapter(),
  sockfd(-1),
  receiver_thread_id(0)
{
  if(type == DeviceType::KIACAN){
    velocity = new std::queue<CanMessage::WHL_SPD>;
    mcm_data = nullptr;
    crc_checker = CRC8(0x07, 0x00, false, false, 0x00, false);
    adapter_type = ADAPTER_SOCKETCAN;
    devicetype = type;
    printf("KIACAN adapter created.\n");
  }
  else if(type == DeviceType::MCM){
    velocity = nullptr;
    mcm_data = new std::queue<CanMessage::MCM_DATA>;
    crc_checker = CRC8(0x07, 0x00, false, false, 0x00, false);
    adapter_type = ADAPTER_SOCKETCAN;
    devicetype = type;
    printf("MCM adapter created.\n");
  }
}

//  Not Implemented
SocketCAN::SocketCAN()
{
  velocity = nullptr;
  mcm_data = new std::queue<CanMessage::MCM_DATA>;
  crc_checker = CRC8(0x07, 0x00, false, false, 0x00, false);
  adapter_type = ADAPTER_SOCKETCAN;
  devicetype = DeviceType::DEFAULT;
  printf("Default adapter created.\n");
}

SocketCAN::~SocketCAN()
{
  printf("Destroying SocketCAN adapter...\n");
  if (this->is_open())
  {
    //this->close();
  }
}


void SocketCAN::open(char* interface)
{
  // Request a socket
  sockfd = socket(PF_CAN, SOCK_RAW, CAN_RAW);
  if (sockfd == -1)
  {
    printf("Error: Unable to create a CAN socket\n");
    return;
  }
  printf("Created CAN socket with descriptor %d.\n", sockfd);

  // Get the index of the network interface
  strncpy(if_request.ifr_name, interface, IFNAMSIZ);
  //if (ioctl(sockfd, SIOCGIFINDEX, &if_request) == -1)
  if (ioctl(sockfd, SIOCGIFINDEX, &if_request) == -1)
  {
    printf("Unable to select CAN interface %s: I/O control error\n", interface);

    // Invalidate unusable socket
    close();
    return;
  }
  printf("Found: %s has interface index %d.\n", interface, if_request.ifr_ifindex);

  // Bind the socket to the network interface
  addr.can_family = AF_CAN;
  addr.can_ifindex = if_request.ifr_ifindex;
  int rc = bind(
    sockfd,
    reinterpret_cast<struct sockaddr*>(&addr),
    sizeof(addr)
  );
  if (rc == -1)
  {
    printf("Failed to bind socket to network interface\n");

    // Invalidate unusable socket
    close();
    return;
  }
  printf("Successfully bound socket to interface %d.\n", if_request.ifr_ifindex);

}


void SocketCAN::close()
{
  terminate_receiver_thread = true;

  if (!is_open())
    return;

  // Close the file descriptor for our socket
  ::close(sockfd);
  sockfd = -1;

  printf("CAN socket destroyed.\n");
}


bool SocketCAN::is_open()
{
  return (sockfd != -1);
}


void SocketCAN::transmit(can_frame_t& frame)
{
  int nbytes;
  //CANAdapter::transmit(frame);
  if (!is_open())
  {
    printf("Unable to transmit: Socket not open\n");
    return;
  }
  nbytes = write(sockfd, &frame, sizeof(can_frame_t));
  std::cout << " write : " << frame.data[0] << frame.data[1] << "\n";
  if(nbytes < 0){
    printf("Write Failed %d bytes \n", nbytes);
    return;
  }
  printf("Write succeed %d bytes\n", nbytes);
}


static void* socketcan_receiver_thread(void* argv)
{
  /*
    * The first and only argument to this function
    * is the pointer to the object, which started the thread.
    */
  SocketCAN* sock = (SocketCAN*) argv;

  // Holds the set of descriptors, that 'select' shall monitor
  fd_set descriptors;

  // Highest file descriptor in set
  int maxfd = sock->sockfd;

  // How long 'select' shall wait before returning with timeout
  struct timeval timeout;

  // Buffer to store incoming frame
  can_frame_t rx_frame;

  // Run until termination signal received
  while (!sock->terminate_receiver_thread)
  {
    // Clear descriptor set
    FD_ZERO(&descriptors);
    // Add socket descriptor
    FD_SET(sock->sockfd, &descriptors);
    
    // Set timeout
    timeout.tv_sec  = 10;
    timeout.tv_usec = 0;
    // Wait until timeout or activity on any descriptor
    if (select(maxfd+1, &descriptors, NULL, NULL, NULL) == 1)
    {
      int len = read(sock->sockfd, &rx_frame, CAN_MTU);
      if (len < 0)
        continue;
      if (sock->reception_handler != NULL)
      {
        sock->reception_handler(&rx_frame);
      }
      else if(sock->pid_reception_handler != NULL)
      {
        if(sock->devicetype == DeviceType::KIACAN)
          sock->pid_reception_handler(&rx_frame, sock->velocity, sock->KIA_Queue_lock);
        else if(sock->devicetype == DeviceType::MCM)
          sock->mcm_reception_handler(&rx_frame, sock->mcm_data, sock->MCM_Queue_lock);
      }
    }
    else
    {
      printf("[RX] Nothing to Receieve.\n");
      sock->terminate_receiver_thread = true;
    }
  }

  printf("[RX] Thread terminated.\n");

  // Thread terminates
  return NULL;
}



void SocketCAN::start_receiver_thread()
{
  /*
    * Frame reception is accomplished in a separate, event-driven thread.
    *
    * See also: https://www.thegeekstuff.com/2012/04/create-threads-in-linux/
    */
  terminate_receiver_thread = false;
  int rc = pthread_create(&receiver_thread_id, NULL, &socketcan_receiver_thread, this);
  if (rc != 0)
  {
    printf("Unable to start receiver thread.\n");
    return;
  }
  printf("Successfully started receiver thread with ID %lu.\n", receiver_thread_id);
  // Wait 6 sec for termination of receiver thread if there's nothing to receive.
  // Can only transmitt can_frame after this wait
  
}


void SocketCAN::pid_control(float obj){
  float velocity_error;
  float integral = 0;
  float velocity_error_last = 0;
  float output_last = 0;

  float Kp = 0;
  float Ki = 0;
  float Kd = 0;
  float Tf = 0; // 시간필터

  float max_output = 0.1; // 속도 신호의 최댓값
  KIA_Queue_lock.lock();
  while(!velocity->empty())
    velocity->pop();
  KIA_Queue_lock.unlock();
  while(true){
    KIA_Queue_lock.lock();
    if(velocity->empty()){ 
      KIA_Queue_lock.unlock();
    }
    else{
      can_frame_t send_data;
      float currenct_velocity = 0;
      currenct_velocity = velocity->front().FL; //Front Left
      currenct_velocity = velocity->front().FR; //Front Right
      currenct_velocity = velocity->front().RL; //Rear Left
      currenct_velocity = velocity->front().RR; //Rear Right
      currenct_velocity /= 4;
      velocity->pop();
      KIA_Queue_lock.unlock();
      velocity_error = obj - currenct_velocity;
      
      float p_term = Kp * velocity_error;
      float i_term = Ki * integral;
      float d_term = Kd * (velocity_error - velocity_error_last)*0.02;

      float output = p_term + i_term + d_term; // pid 계산값

      if(output >= max_output){
          output = max_output;
      }else if( output <= -max_output){
          output = 0;
      }else{
          integral += (velocity_error * 0.02);
      }
      float_hex_convert temp;
      temp.val = output;
      make_can_frame(COMMAND_ID, temp, send_data);
      transmit(send_data);
      output_last = output;
      velocity_error_last = velocity_error;
    }
  }
}

bool SocketCAN::mcm_state_update(){
  MCM_Queue_lock.lock();
  while(!mcm_data->empty()){
    CanMessage::MCM_DATA data = mcm_data->front();
    mcm_data->pop();
    MCM_Queue_lock.unlock();
    if(data.type == MCM_MESSAGE_TYPE::CONTROL_RESPONSE){
      std::cout << "got control response msg \n";
      switch (data.int_id)
      {
      case BRAKE_ID:
        if(MCM_State.Brake_Control_State != data.bool_data){
          MCM_State.Brake_Control_State = data.bool_data;
          std::cout << "MCM [BRAKE] Control state have changed to " << MCM_State.Brake_Control_State\
                    << " \n";
        }
        break;
      case ACCEL_ID:
        if(MCM_State.Accel_Control_State != data.bool_data){
          MCM_State.Accel_Control_State = data.bool_data;
          std::cout << "MCM [ACCEL] Control state have changed to " << MCM_State.Accel_Control_State\
                    << " \n";
        }
        break;
      case STEER_ID:
        if(MCM_State.Steer_Control_State != data.bool_data){
          MCM_State.Steer_Control_State = data.bool_data;
          std::cout << "MCM [STEER] Control state have changed to " << MCM_State.Steer_Control_State\
                    << " \n";
        }
        break;
      default:
        break;
      }
    }
  }
  if(MCM_State.Brake_Control_State == 1 && MCM_State.Accel_Control_State == 1 &&\
      MCM_State.Steer_Control_State == 1){
        return true;
      }
  return false;
}

void SocketCAN::send_control_request(uint8_t interface, bool enable){
  can_frame_t send_data;
  make_can_frame(CONTROL_ID, interface, enable, send_data);
  transmit(send_data);
}

void SocketCAN::make_can_frame(unsigned int id_hex, uint8_t interface_id, bool value\
                                                                 ,can_frame_t& data){
  float_hex_convert converter;
  memset(converter.data, 0, 4); 
  converter.data[0] = interface_id;
  converter.data[1] = value;
  make_can_frame(id_hex, converter, data);
}

void SocketCAN::make_can_frame(unsigned int id_hex, float_hex_convert converter\
                                                            , can_frame_t& data){
  switch (id_hex){
  //Accel Command
  case 0x160: {
    data.can_dlc = 8;
    data.can_id = id_hex;
    data.data[0] = DEFAULT_BUS;
    data.data[1] = ACCEL_ID;
    data.data[2] = NONE;
    memcpy(data.data+3, converter.data, 4);
    break;
  }
  case 0x60: {
    data.can_dlc = 8;
    data.can_id = id_hex;
    memcpy(data.data+3, converter.data, 4);
    break;
  }
  default:
    break;
  }
  crc_8(data.data, 7, data.data+7);
}

void SocketCAN::crc_8(uint8_t* data, int data_length_bytes, uint8_t* crc){
  *crc = crc_checker.calculate(data, data_length_bytes);
}


#endif
