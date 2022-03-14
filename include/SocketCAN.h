/**
 * @file
 * This file declares an interface to SocketCAN,
 * to facilitates frame transmission and reception.
 */

#if (!defined(SOCKETCAN_H)) && (!defined(MINGW))
#define SOCKETCAN_H

#include <CANAdapter.h>
#include <CANFrame.h>
#include <stdbool.h>
// IFNAMSIZ, ifreq
#include <net/if.h>
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
    std::queue<float>* velocity;
    std::mutex qlock;
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
  
    void pid_control(float obj);

    void make_can_frame(int id, float value, can_frame_t* data);
};

#endif
