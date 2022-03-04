
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
*/



std::unique_ptr<dbcppp::INetwork> net;
std::unordered_map<uint64_t, const dbcppp::IMessage*> messages;


//  ADD All Messages in DBC file
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


//  ADD Unique Message(vector) in DBC file
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

void rx_ichthus_handler(can_frame_t* frame)
{
    auto iter = messages.find(frame->can_id);
    if (iter != messages.end())
    {
        const dbcppp::IMessage* msg = iter->second;
        for (const dbcppp::ISignal& sig : msg->Signals())
        {
            const dbcppp::ISignal* mux_sig = msg->MuxSignal();
            if (sig.MultiplexerIndicator() != dbcppp::ISignal::EMultiplexer::MuxValue ||
                (mux_sig && mux_sig->Decode(frame->data) == sig.MultiplexerSwitchValue()))
            {
                std::cout << sig.RawToPhys(sig.Decode(frame->data)) << sig.Unit() << "\n";
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


void test_socketcan()
{
    printf("\nTesting SocketCAN adapter\n");
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

/*
    printf("Starts Transmit test \n");

    can_frame_t frame;
    frame.can_id = 0x386;
    frame.can_dlc = 7;
    frame.data[0] = 1;
    frame.data[1] = 1;
    frame.data[2] = 1;
    frame.data[3] = 1;
    frame.data[4] = 1;
    frame.data[5] = 1;
    frame.data[6] = 1;
    adapter->transmit(frame);
*/
    pthread_join(adapter->receiver_thread_id, NULL);
    delete adapter;
    sleep(1.1);
}



int main()
{
    test_socketcan();
    return 0;
}
