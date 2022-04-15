
//vector find
#include <algorithm>
// sleep, socket
#include <unistd.h>
// printf
#include <stdio.h>
// uintxx_t
#include <stdint.h>
#include <SocketCAN.h>
#include <SLCAN.h>

//dbcppp headers
#include <fstream>
#include <unordered_map>
#include "dbcppp/CApi.h"
#include "dbcppp/Network.h"

//for queue
#include <queue>

//#define PIDTEST
#define THROTTLETEST


/*
// How To Test in VCAN0 (Virtual Can)
// 
// 'sudo apt install net-tools'
// 'sudo apt install can-utils'
// 'sudo modprobe vcan'
// 'sudo ip link add dev vcan0 type vcan'
// 'sudo ip link set up vcan0'
// 'candump vcan0'
// 
// After this, just open another terminal and command
// 'cansend vcan0 386#11.11.11.11.11.11.11.11'
// And you can see the message you've sent shows right up in candump terminal.
// 
// In libcan directory where libcan.so and other so files exist, command 'make'   
// CD into test directory and 'make'
// Now you can run test programm with './test hyundai_kia_generic.dbc'
//
//
// TEST PROGRAMM USAGE 
// ./test <MCM CAN BUS> <KIA CAN BUS> 
// ex) ./test can0 can1
//
10000001 00000000 00000001 00001000 00000000 00000000 00000000
00000001 00000000 00000001 00001000 00000000 00000000 00000000

00000001 00100000 00000001 00000000 10000110 10000111 00000000
10000001 00100000 00000001 00000000 10000110 00000000 00000000


 TODO (20220408) MinsungKim
   1. Fix Switching logic between ThrottlePID & BrakePID (Reference legacy ichthus_controller)
   2. Implement SteeringPID simple as ThrottlePID (use reference Steering Wheel Position from KIA CAN)


//  [COMMENT FOR ROS2 BRINGUP]
//  Some logics which used for sharing data between threads(locks, queue, etc..)
//  are no more needed because in ROS2 we will use ROS2 msg publish & Subscribe logic.
//  
//  rx_pid_ichthus_handler()           -> needed for KIA driver node (already implemented)
//  rx_mcm_ichthus_handler()           -> needed for MCM driver node 
//  pid_test()                         -> needed for PID node  
//   ㄴ Recieving reference speed from user using CLI is an Experimental Logic, subject to change.              
//  test_interface()                   -> needed for PID node(need more discussion)
//
//
//
*/

std::unique_ptr<dbcppp::INetwork> net;
std::unordered_map<uint64_t, const dbcppp::IMessage*> messages;
std::queue<CanMessage::WHL_SPD> KIA_QUEUE;
std::mutex KIA_LOCK;


void Clear()
{
    std::cout << "\x1B[2J\x1B[H";
}

//  DBC LOADER FUNCTION
//  ADD All Messages in DBC file to parser object
void load_dbc(){
    std::cout << "Loading DBC \n";

    {
        std::ifstream idbc("hyundai_kia_generic.dbc");
        net = dbcppp::INetwork::LoadDBCFromIs(idbc);
    }
    
    for (const dbcppp::IMessage& msg : net->Messages())
    {
        messages.insert(std::make_pair(msg.Id(), &msg));
    }
}

//  DBC LOADER FUNCTION
//  ADD Unique Message(vector) in DBC file to parser object
void load_dbc(std::vector<uint64_t>& id){
    std::cout << "Loading DBC \n";
    
    {
        std::ifstream idbc("hyundai_kia_generic.dbc");
        net = dbcppp::INetwork::LoadDBCFromIs(idbc);
    }

    
    for (const dbcppp::IMessage& msg : net->Messages())
    {
        auto iter = find(id.begin(), id.end(), msg.Id());
        if(iter != id.end()){    
            messages.insert(std::make_pair(msg.Id(), &msg));
        }
    }
}

void rx_pid_ichthus_handler(can_frame_t* frame, std::queue<CanMessage::WHL_SPD>* velocity,\
                                                         std::mutex& KIA_Queue_lock)
{
    auto iter = messages.find(frame->can_id);
    if (iter != messages.end())
    {
        const dbcppp::IMessage* msg = iter->second;
        int counter = 0;
        KIA_Queue_lock.lock();   
        for (const dbcppp::ISignal& sig : msg->Signals())
        {
            const dbcppp::ISignal* mux_sig = msg->MuxSignal();
            if (sig.MultiplexerIndicator() != dbcppp::ISignal::EMultiplexer::MuxValue ||
                (mux_sig && mux_sig->Decode(frame->data) == sig.MultiplexerSwitchValue()))
            {
                counter++;
                CanMessage::WHL_SPD data;
                switch (counter)
                {
                case 1:
                        data.FL = sig.RawToPhys(sig.Decode(frame->data));
                    break;
                case 2:
                        data.FR = sig.RawToPhys(sig.Decode(frame->data));
                    break;
                case 3:
                        data.RL = sig.RawToPhys(sig.Decode(frame->data));
                    break;
                case 4:
                        data.RR = sig.RawToPhys(sig.Decode(frame->data));
                        velocity->push(data);
                        counter = 0;
                        KIA_Queue_lock.unlock();
                default:
                    return;
                }
            }
        }
    }
}

void rx_mcm_ichthus_handler(can_frame_t* Frame, std::queue<CanMessage::MCM_DATA>* MCM_Data,\
                                                         std::mutex& MCM_Queue_Lock)
{
    CanMessage::MCM_DATA data_;
    switch (Frame->can_id)
    {
    case 0x061: //Control Enable Response From MCM
        data_.subsys_id = (Frame->data[0] & SUBSYS_MASK) >> 7;
        data_.hex_id = Frame->data[1];
        data_.bool_data = Frame->data[2];
        data_.type = MCM_MESSAGE_TYPE::CONTROL_RESPONSE;
        MCM_Queue_Lock.lock();
        MCM_Data->push(data_);
        MCM_Queue_Lock.unlock();
        break;
    case 0x161:
        break;
    default:
        break;
    }
}

void rx_handler(can_frame_t* frame)
{
    printf("Received bytes: Frame from 0x%0X, DLC=%d\n", frame->can_id, frame->can_dlc);
    std::cout << std::flush;
}

void pid_test(char* mcm, char* kia){
    if(mcm == NULL || kia == NULL){
        std::cout << "CAN Argument Error \n";
        exit(-1);
    }
    int temp_flag = 0;
    int temp_flag_ = 0;
    char temp_char = '\0';
    float obj = 0;
    printf("\n Starts PID Control Test \n");
    printf("Create MCM Adapter with [%s] KIA Adapter with [%s] \n", mcm, kia);
    printf("#############################\n");
    std::vector<uint64_t> id;
    id.push_back(902);  //WHL_SPD11     WHL_SPD_FL, FR, RL, RR
    load_dbc(id);
    
    SocketCAN* KIAadapter = new SocketCAN(DeviceType::KIACAN, KIA_QUEUE, KIA_LOCK);
    KIAadapter->pid_reception_handler = &rx_pid_ichthus_handler;
    KIAadapter->open(kia);
    KIAadapter->start_receiver_thread();
    
    SocketCAN* MCMadapter = new SocketCAN(DeviceType::MCM, KIA_QUEUE, KIA_LOCK);
    MCMadapter->mcm_reception_handler = &rx_mcm_ichthus_handler;
    MCMadapter->open(mcm);
    MCMadapter->start_receiver_thread();
    sleep(3);
    
    std::cout << "==================================================" << "\n";
    while(temp_flag_ == 0){
        #ifdef PIDTEST
        std::cout << "PID TEST,, SKIP Control Request \n";
        #endif
        #ifndef PIDTEST
        while(temp_flag == 0){
            MCMadapter->send_control_request(ACCEL_ID, true);
            #ifndef THROTTLETEST
            MCMadapter->send_control_request(BRAKE_ID, true);
            MCMadapter->send_control_request(STEER_ID, true);
            #endif
            std::cout << "Send Control Request, Waiting for Response (3s)\n";
            sleep(5);
            if(MCMadapter->mcm_state_update() != true){
                std::cout << "Control Enable Failed \n";
                std::cout << "Try Again? [y/n] \n";
                std::cin >> temp_char;
                if(temp_char == 'n'){
                    std::cout << "Control Request Stopped \n";
                    exit(0);
                }
                else if(temp_char == 'y'){
                    temp_char = '\0';
                    temp_flag = 0;
                }
            }
            else{
                temp_flag = 1;
            }
        }
        #endif
        sleep(5);
        std::cout << "==================================================" << "\n";
        std::cout << "Control Enabled \n";
        std::cout << "==================================================" << "\n";
        std::cout << "Object Value (km/h?) : ";
        std::cin >> obj;
        while(1){
            MCMadapter->pid_decision(obj);
            #ifndef PIDTEST
            if(!MCMadapter->mcm_state_update()){
                std::cout << "Control Diabled\n";
                std::cout << "Do again? 0:yes 1:no" << "\n";
                std::cin >> temp_flag_;
                if(temp_flag_ == 1)
                    exit(0);
            }
            #endif
        }
    }
    pthread_join(KIAadapter->receiver_thread_id, NULL);
    pthread_join(MCMadapter->receiver_thread_id, NULL);
    delete KIAadapter;
    delete MCMadapter;
    sleep(1.1);
}

void can_recieve_test(){
    std::cout << "==================================================" << "\n";
    std::cout << "CAN Recieve Test \n";
    std::cout << "Select CAN Interface for test\n";
    std::cout << "MCM 1 \n";
    std::cout << "KIA 2 \n";
    int choice = 0;
    char sock[5];
    std::cin >> choice;
    std::cout << "Socket Interface name : ";
    std::cin >> sock;
    switch (choice)
    {
    case 1:{
        printf("\nStarts MCM CAN Reciever %s \n", sock);
        printf("#############################\n");
        std::vector<uint64_t> id;
        SocketCAN* adapter = new SocketCAN(DeviceType::MCM);
        adapter->mcm_reception_handler = &rx_mcm_ichthus_handler;
        adapter->open(sock);
        adapter->start_receiver_thread();
        pthread_join(adapter->receiver_thread_id, NULL);
        delete adapter;
        sleep(1.1);
        break;
    }
    case 2:{
        printf("\nStarts KIA CAN Reciever %s \n", sock);
        printf("#############################\n");
        std::vector<uint64_t> id;
        id.push_back(902);  //WHL_SPD11     WHL_SPD_FL, FR, RL, RR
        load_dbc(id);   
        SocketCAN* adapter = new SocketCAN(DeviceType::KIACAN);
        adapter->pid_reception_handler = &rx_pid_ichthus_handler;
        adapter->open(sock);
        adapter->start_receiver_thread();
        pthread_join(adapter->receiver_thread_id, NULL);
        delete adapter;
        sleep(1.1);
        break;
    }
        break;
    
    default:
        break;
    }
}


void test_interface(char* mcm, char* kia){
    int command = -1;
    std::cout << "==================================================" << "\n";
    std::cout << "============= ICHTHUS ACTUATION TEST =============" << "\n";
    std::cout << "==================================================" << "\n";
    std::cout << "= 0. MCM Control Status                          =" << "\n";
    std::cout << "= 1. PID CONTROL TEST                            =" << "\n";
    std::cout << "= 2. CAN RECIEVE TEST                            =" << "\n";
    std::cout << "= 3. HELP                                        =" << "\n";
    std::cout << "= 4. EXIT                                        =" << "\n";
    std::cout << "==================================================" << "\n";
    std::cout << "==================================================" << "\n";
    std::cout << ":";
    std::cin >> command;
    Clear();
    switch (command)
    {
    case 0:
        std::cout << "==================================================" << "\n";
        std::cout << "=============== MCM Control Status ==============" << "\n";
        std::cout << "==================================================" << "\n";
        std::cout << "==================================================" << "\n";
        std::cout << "==================================================" << "\n";
        std::cout << "==================================================" << "\n";
        std::cout << "Not Implemented \n";
        break;
    case 1:
        std::cout << "==================================================" << "\n";
        std::cout << "================ PID CONTROL TEST ================" << "\n";
        std::cout << "==================================================" << "\n"; 
        pid_test(mcm, kia); 
        break;
    case 2:
        can_recieve_test();  
        break;
    case 3:
        std::cout << "==================================================" << "\n";
        std::cout << "======================= HELP =====================" << "\n";
        std::cout << "==================================================" << "\n";  
        std::cout << "= 1.                                             =" << "\n";
        std::cout << "= 2. MCM STATUS CHECK                            =" << "\n";
        std::cout << "= 3. CAN RECIEVE TEST                            =" << "\n";
        std::cout << "= 4. EXIT                                        =" << "\n";
        std::cout << "==================================================" << "\n";
        std::cout << "==================================================" << "\n";
        break;
    case 4:
        std::cout << "==================================================" << "\n";
        std::cout << "================ CAN RECIEVE TEST ================" << "\n";
        std::cout << "==================================================" << "\n";
        can_recieve_test(); 
        exit(0);

    default:
        std::cout << "QUIT TEST \n";
        exit(0);
    }
    
}

int main(int argc, char* argv[])
{
    test_interface(argv[1], argv[2]);
    return 0;
}



/*
void test_socketcan(char* mcm, char* kia)
{
    printf("\nStarts KIA CAN Reciever\n");
    printf("#############################\n");
    
    std::vector<uint64_t> id;
    id.push_back(688);  //SAS11         SAS_ANGLE, SAS_SPEED
    id.push_back(902);  //WHL_SPD11     WHL_SPD_FL, FR, RL, RR
    id.push_back(881);  //E_EMS11       ACC PEDAL POS, BEK PEDAL POS
    id.push_back(544);  //ESP12         LONG_ACCEL, LAT_ACCEL, YAW_RATE
    
    load_dbc(id);
    SocketCAN* adapter = new SocketCAN();
    adapter->reception_handler = &rx_ichthus_handler;
    adapter->open("vcan0");
    adapter->start_receiver_thread();


    printf("Starts Transmit test \n");

    can_frame_t frame;
    frame.can_id = 0x386;
    frame.can_dlc = 7;
    frame.data[0] = 10.1;
    frame.data[1] = 10.1;
    frame.data[2] = 100;
    frame.data[3] = 100;
    frame.data[4] = 100;
    frame.data[5] = 100;
    frame.data[6] = 100;
    adapter->transmit(frame);

    pthread_join(adapter->receiver_thread_id, NULL);
    delete adapter;
    sleep(1.1);
}
*/