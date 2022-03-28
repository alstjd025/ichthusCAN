/**
 * @file
 * This file declares an interface to SocketCAN,
 * to facilitates frame transmission and reception.
 */

#if (!defined(SOCKETCAN_H)) && (!defined(MINGW))
#define SOCKETCAN_H

#include "CRC.h"

#include <CANAdapter.h>
#include <CANFrame.h>
#include <stdbool.h>
// IFNAMSIZ, ifreq
#include <net/if.h>
#include <netinet/in.h>
// Multi-threading
#include <pthread.h>
#include <iostream>

#include <queue>
#include <mutex>


/**
 * Interface request structure used for socket ioctl's
 */
typedef struct ifreq interface_request_t;

/**
 * Socket address type for CAN sockets
 */
typedef struct sockaddr_can can_socket_address_t;


/**
 * Facilitates frame transmission and reception via a CAN adapter
 */
class SocketCAN: public CANAdapter
{
  private:
  public:
    interface_request_t if_request;
    can_socket_address_t addr;
    pthread_t receiver_thread_id;
    
    //nullptr when MCM sock
    std::queue<CanMessage::WHL_SPD>* velocity;
    
    //nullptr when KIACAN sock
    std::queue<CanMessage::MCM_DATA>* mcm_data;
    
    std::mutex KIA_Queue_lock;
    std::mutex MCM_Queue_lock;
    std::mutex MCM_State_lock;

    //TYPE (ex. MCM)
    DeviceType devicetype;

    //MCM Status struct
    //Only SockType::MCM can own this
    MCM_STATE MCM_State;

    //CRC
    CRC8 crc_checker;

    /**
     * CAN socket file descriptor
     */
    int sockfd = -1;

    /**
     * Request for the child thread to terminate
     */
    bool terminate_receiver_thread = false;

    /** Constructor */
    SocketCAN();
    SocketCAN(DeviceType type);
    /** Destructor */
    ~SocketCAN();

    /**
     * Open and bind socket
     */
    void open(char*);
    
    /**
     * Close and unbind socket
     */
    void close();

    /**
     * Returns whether the socket is open or closed
     *
     * @retval true     Socket is open
     * @retval false    Socket is closed
     */
    bool is_open();

    /**
     * Sends the referenced frame to the bus
     */
    void transmit(can_frame_t&);

    /**
     * Starts a new thread, that will wait for socket events
     */
    void start_receiver_thread();
    bool mcm_state_update();


    void pid_control(float obj);

    void send_control_request(uint8_t interface, bool enable);

    void make_can_frame(unsigned int id_hex, float_hex_convert converter, can_frame_t& data);
    void make_can_frame(unsigned int id_hex, uint8_t interface_id, bool value, can_frame_t& data);

    void crc_8(uint8_t* data, int data_length, uint8_t* crc);
};

#endif
