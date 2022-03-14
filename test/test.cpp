
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
*/



std::unique_ptr<dbcppp::INetwork> net;
std::unordered_map<uint64_t, const dbcppp::IMessage*> messages;

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

void rx_ichthus_handler(can_frame_t* frame, std::queue<float>* velocity, std::mutex& qlock)
{
    auto iter = messages.find(frame->can_id);
    if (iter != messages.end())
    {
        const dbcppp::IMessage* msg = iter->second;
        float velocity_temp = 0;
        int counter = 0;
        for (const dbcppp::ISignal& sig : msg->Signals())
        {
            const dbcppp::ISignal* mux_sig = msg->MuxSignal();
            if (sig.MultiplexerIndicator() != dbcppp::ISignal::EMultiplexer::MuxValue ||
                (mux_sig && mux_sig->Decode(frame->data) == sig.MultiplexerSwitchValue()))
            {
                velocity_temp += sig.RawToPhys(sig.Decode(frame->data));
                counter++;
                if(counter == 4){
                    qlock.lock();
                    velocity->push(velocity_temp/4);
                    counter = 0;
                    velocity_temp = 0;
                    qlock.unlock();
                    return;
                }
            }
        }
    }

}

void rx_handler(can_frame_t* frame)
{
    printf("Received bytes: Frame from 0x%0X, DLC=%d\n", frame->can_id, frame->can_dlc);
    
    auto iter = messages.find(frame->can_id);
    if (iter != messages.end())
    {
        const dbcppp::IMessage* msg = iter->second;
        std::cout << "Received Message: " << msg->Name() << "\n";
        for (const dbcppp::ISignal& sig : msg->Signals())
        {
            const dbcppp::ISignal* mux_sig = msg->MuxSignal();
            if (sig.MultiplexerIndicator() != dbcppp::ISignal::EMultiplexer::MuxValue ||
                (mux_sig && mux_sig->Decode(frame->data) == sig.MultiplexerSwitchValue()))
            {
                std::cout << "\t" << sig.Name() << "=" << sig.RawToPhys(sig.Decode(frame->data)) << sig.Unit() << "\n";
            }
        }
    }

    std::cout << std::flush;
    // TODO: Do something here with the received frame
}

void open_kiacan_start_pid(){
    float obj = 0;
    printf("\nStarts KIA CAN Reciever (can0)\n");
    printf("#############################\n");
    std::vector<uint64_t> id;
    id.push_back(902);  //WHL_SPD11     WHL_SPD_FL, FR, RL, RR
    load_dbc(id);
    SocketCAN* adapter = new SocketCAN();
    adapter->reception_handler = &rx_ichthus_handler;
    adapter->open("can0");
    adapter->start_receiver_thread();
    std::cout << "Object Value (km/h?) : ";
    std::cin >> obj;
    adapter->pid_control(obj);
    pthread_join(adapter->receiver_thread_id, NULL);
    delete adapter;
    sleep(1.1);
}


void test_interface(){
    int command = -1;
    std::cout << "==================================================" << "\n";
    std::cout << "============== STARTS TEST INTERFACE =============" << "\n";
    std::cout << "==================================================" << "\n";
    std::cout << "= 0. PID CONTROL TEST                            =" << "\n";
    std::cout << "= 1. MCM STATUS CHECK                            =" << "\n";
    std::cout << "= 2. CAN RECIEVE TEST                            =" << "\n";
    std::cout << "= 3. EXIT                                        =" << "\n";
    std::cout << "==================================================" << "\n";
    std::cout << "==================================================" << "\n";
    std::cout << ":";
    std::cin >> command;
    switch (command)
    {
    case 0:
        std::cout << "==================================================" << "\n";
        std::cout << "================ PID CONTROL TEST ================" << "\n";
        std::cout << "==================================================" << "\n"; 
        open_kiacan_start_pid(); 
        break;
    case 1:
        std::cout << "==================================================" << "\n";
        std::cout << "================ MCM STATUS CHECK ================" << "\n";
        std::cout << "==================================================" << "\n";  
        break;
    case 2:
        std::cout << "==================================================" << "\n";
        std::cout << "================ CAN RECIEVE TEST ================" << "\n";
        std::cout << "==================================================" << "\n"; 
        break;
    case 3:
        break;
    case -1:
        std::cout << "QUIT TEST \n";
        exit(0);

    default:
        std::cout << "QUIT TEST \n";
        exit(0);
    }
    
}

int main(int argc, char* argv[])
{
    test_interface();
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