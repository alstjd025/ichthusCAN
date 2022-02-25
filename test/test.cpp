
// sleep
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


std::unique_ptr<dbcppp::INetwork> net;
std::unordered_map<uint64_t, const dbcppp::IMessage*> messages;

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
    load_dbc();

    SocketCAN* adapter = new SocketCAN();
    adapter->reception_handler = &rx_handler;
    
    adapter->open("vcan0");
    
    adapter->start_receiver_thread();
    


    sleep(3);
    printf("Starts Transmit test \n");
/*
    can_frame_t frame;
    frame.can_id = 0x123;
    frame.can_dlc = 3;
    frame.data[0] = 1;
    frame.data[1] = 2;
    frame.data[2] = 3;
    adapter->transmit(&frame);
*/
    delete adapter;
    sleep(1.1);
}



int main()
{
    test_socketcan();
    return 0;
}
