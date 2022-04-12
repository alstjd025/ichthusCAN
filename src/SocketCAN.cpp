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
    initialize_mcm_State();
    printf("MCM adapter created.\n");
  }
}

SocketCAN::SocketCAN(DeviceType type, std::queue<CanMessage::WHL_SPD>& KIA_queue,\
                std::mutex& KIA_lock):CANAdapter(), sockfd(-1), receiver_thread_id(0){
  if(type == DeviceType::KIACAN){
    velocity = &KIA_queue;
    KIA_Queue_lock = &KIA_lock;
    mcm_data = nullptr;
    crc_checker = CRC8(0x07, 0x00, false, false, 0x00, false);
    adapter_type = ADAPTER_SOCKETCAN;
    devicetype = type;
    printf("KIACAN adapter created.\n");
  }
  else if(type == DeviceType::MCM){
    velocity = &KIA_queue;
    KIA_Queue_lock = &KIA_lock;
    mcm_data = new std::queue<CanMessage::MCM_DATA>;
    crc_checker = CRC8(0x07, 0x00, false, false, 0x00, false);
    adapter_type = ADAPTER_SOCKETCAN;
    devicetype = type;
    initialize_mcm_State();
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

void SocketCAN::initialize_mcm_State(){
  MCM_State_lock.lock();
  MCM_State_subsys1.Accel_Control_State = false;
  MCM_State_subsys1.Brake_Control_State = false;
  MCM_State_subsys1.Steer_Control_State = false;
  MCM_State_subsys2.Accel_Control_State = false;
  MCM_State_subsys2.Brake_Control_State = false;
  MCM_State_subsys2.Steer_Control_State = false;
  MCM_State_lock.unlock();
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
  if(nbytes < 0){
    printf("Write Failed %d bytes \n", nbytes);
    return;
  }
  return;
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
  #ifndef PIDTEST
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
        sock->reception_handler(&rx_frame);
      else if(sock->pid_reception_handler != NULL)
        sock->pid_reception_handler(&rx_frame, sock->velocity, *sock->KIA_Queue_lock);
      else if(sock->mcm_reception_handler != NULL)
        sock->mcm_reception_handler(&rx_frame, sock->mcm_data, sock->MCM_Queue_lock);
    }
    else
    {
      printf("[RX] Nothing to Receieve.\n");
      sock->terminate_receiver_thread = true;
    }
  }
  #endif
  #ifdef PIDTEST  //Send dummy data to queueu
  CanMessage::WHL_SPD temp_spd;
  temp_spd.FL = 10;
  temp_spd.FR = 10;
  temp_spd.RL = 10;
  temp_spd.RR = 10;
  while(1){
    sock->KIA_Queue_lock->lock();
    sock->velocity->push(temp_spd);  
    sock->KIA_Queue_lock->unlock();
    sleep(1);
  }
  #endif

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
void SocketCAN::pid_decision(float ref){

  float currenct_velocity = 0;
  KIA_Queue_lock->lock();
  if(!velocity->empty()){
    currenct_velocity += velocity->front().FL; //Front Left
    currenct_velocity += velocity->front().FR; //Front Right
    currenct_velocity += velocity->front().RL; //Rear Left
    currenct_velocity += velocity->front().RR; //Rear Right
    currenct_velocity /= 4;
    velocity->pop();
    KIA_Queue_lock->unlock();
    std::cout << "Cur Vel : " << currenct_velocity << "\n";
    float velocity_err = obj - currenct_velocity;

    if(velocity_err > 0){ //buffer 만들기 
      throttle_pid_control(velocity_err);
    }else{
      brake_pid_control(velocity_err);
    }
    return;
  }
  else
    KIA_Queue_lock->unlock();  
  return;
}

void SocketCAN::throttle_pid_control(float err){
  float velocity_error;
  float integral = 0;

  float max_output = 0.3; // 속도 신호의 최댓값
  can_frame_t send_data;
  velocity_error = err;
  
  float p_term = thr_Kp * velocity_error;
  float i_term = thr_Ki * integral;
  float d_term = thr_Kd * (velocity_error - velocity_error_last);

  float output = p_term; /*+ i_term + d_term*/ // pid 계산값 ,, p_term만 가지고 제어한 후 d_term과 I_term으로 추가제어
  std::cout << "output : " << output << "\n";

  if(output >= max_output){
      output = max_output;
  }else if( output <= 0){
      output = 0;
  }

  integral += (velocity_error); // iterm을 사용하고자 할때 error값을 조건부로 받으면 안됨.

  float_hex_convert temp;
  temp.val = output;
  send_data.data[1] = ACCEL_ID;
  make_can_frame(COMMAND_ID, temp, send_data);
  transmit(send_data);
  thr_output_last = output;
  thr_velocity_error_last = velocity_error;
    
  
}

void SocketCAN::brake_pid_control(float err){
  float velocity_error;
  float integral = 0;

  float max_output = 0.1; // 속도 신호의 최댓값

  can_frame_t send_data;

  velocity_error = err;
  
  float p_term = br_Kp * velocity_error;
  float i_term = br_Ki * integral;
  float d_term = br_Kd * (velocity_error - velocity_error_last);

  float output = p_term + i_term + d_term; // pid 계산값

  if(output >= max_output){
      output = max_output;
  }else if( output <= -max_output){
      output = 0;
  }
  
  
  integral += (velocity_error);
  float_hex_convert temp;
  temp.val = output;
  send_data.data[1] = BRAKE_ID;
  make_can_frame(COMMAND_ID, temp, send_data);
  transmit(send_data);

  brk_output_last = output;
  brk_velocity_error_last = velocity_error;

}

void SocketCAN::steer_pid_control(float err){
  float velocity_error;
  float max_output = 0.1; 

  can_frame_t send_data;

  velocity_error = err;
  
  float p_term = ste_Kp * velocity_error;
  float i_term = ste_Ki * integral;
  float d_term = ste_Kd * (velocity_error - ste_velocity_error_last);

  float output = p_term + i_term + d_term; // pid 계산값
  
  
  integral += (velocity_error);
  float_hex_convert temp;
  temp.val = output;
  send_data.data[1] = STE_ID;
  make_can_frame(COMMAND_ID, temp, send_data);
  transmit(send_data);

  ste_output_last = output;
  ste_velocity_error_last = velocity_error;

}

bool SocketCAN::mcm_state_update(){
  MCM_Queue_lock.lock();
  MCM_State_lock.lock();
  while(!mcm_data->empty()){
    CanMessage::MCM_DATA data = mcm_data->front();
    mcm_data->pop();
    MCM_Queue_lock.unlock();
    if(data.type == MCM_MESSAGE_TYPE::CONTROL_RESPONSE){
      switch (data.hex_id)
      {
      case BRAKE_ID:
        if(data.subsys_id == 0){
          if(MCM_State_subsys1.Brake_Control_State != data.bool_data)
            MCM_State_subsys1.Brake_Control_State = data.bool_data;
        }
        else if(data.subsys_id == 1){
          if(MCM_State_subsys2.Brake_Control_State != data.bool_data)
            MCM_State_subsys2.Brake_Control_State = data.bool_data;      
        }
        printf("MCM [BRAKE] Control state have changed to [SUBSYS1] %d [SUBSYS2] %d \n"\
              , MCM_State_subsys1.Brake_Control_State, MCM_State_subsys2.Brake_Control_State);
        break;
      case ACCEL_ID:
        if(data.subsys_id == 0){
          if(MCM_State_subsys1.Accel_Control_State != data.bool_data)
            MCM_State_subsys1.Accel_Control_State = data.bool_data;
        }
        else if(data.subsys_id == 1){
          if(MCM_State_subsys2.Accel_Control_State != data.bool_data)
            MCM_State_subsys2.Accel_Control_State = data.bool_data;      
        }
        printf("MCM [ACEEL] Control state have changed to [SUBSYS1] %d [SUBSYS2] %d \n"\
              , MCM_State_subsys1.Accel_Control_State, MCM_State_subsys2.Accel_Control_State);
        break;
      case STEER_ID:
        if(data.subsys_id == 0){
          if(MCM_State_subsys1.Steer_Control_State != data.bool_data)
            MCM_State_subsys1.Steer_Control_State = data.bool_data;
        }
        else if(data.subsys_id == 1){
          if(MCM_State_subsys2.Steer_Control_State != data.bool_data)
            MCM_State_subsys2.Steer_Control_State = data.bool_data;      
        }
        printf("MCM [Steer] Control state have changed to [SUBSYS1] %d [SUBSYS2] %d \n"\
              , MCM_State_subsys1.Steer_Control_State, MCM_State_subsys2.Steer_Control_State);
        break;
      default:
        break;
      }
    }
  }
  /*
  MCM_Queue_lock.unlock();
  if(MCM_State_subsys1.Brake_Control_State == 1 && MCM_State_subsys1.Accel_Control_State == 1 &&\
      MCM_State_subsys1.Steer_Control_State == 1 && MCM_State_subsys2.Brake_Control_State == 1 &&\
      MCM_State_subsys2.Accel_Control_State == 1 && MCM_State_subsys2.Steer_Control_State == 1){
      MCM_State_lock.unlock();
    return true;
  }
  */
  MCM_Queue_lock.unlock();
  if(MCM_State_subsys1.Accel_Control_State == 1 && MCM_State_subsys2.Accel_Control_State == 1){
      MCM_State_lock.unlock();
    return true;
  }
  MCM_State_lock.unlock();
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
    //data.data[1] = ACCEL_ID;
    data.data[2] = NONE;
    std::cout << "data : " << converter.val << "\n";
    memcpy(data.data+3, converter.data, sizeof(float));
    break;
  }
  //Control Request
  case 0x60: {
    data.can_dlc = 8;
    data.can_id = id_hex;
    data.data[0] = DEFAULT_BUS;
    memcpy(data.data+1, converter.data, sizeof(float));
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
