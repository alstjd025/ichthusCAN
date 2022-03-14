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
 * https://github.com/JCube001/socketcan-demo
 * http://blog.mbedded.ninja/programming/operating-systems/linux/how-to-use-socketcan-with-c-in-linux
 * https://github.com/linux-can/can-utils/blob/master/candump.c
 */

/*
#include <endian.h>
#include <sys/socket.h>
#include <sys/types.h>
*/


SocketCAN::SocketCAN()
   :CANAdapter(),
    sockfd(-1),
    receiver_thread_id(0)
{
    velocity = new std::queue<float>;
    adapter_type = ADAPTER_SOCKETCAN;
    printf("SocketCAN adapter created.\n");
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
                sock->reception_handler(&rx_frame, sock->velocity, sock->qlock);
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
    while(true){
        qlock.lock();
        if(velocity->empty()){ // :(
            qlock.unlock();
        }
        else{
            float currenct_velocity = velocity->front();
            velocity->pop();
            std::cout << "pid :: " << currenct_velocity << "\n";
            qlock.unlock();
        }
    }
}

void SocketCAN::make_can_frame(int id, float value, can_frame_t* data){
    data->can_id = id;
    
}

#endif
