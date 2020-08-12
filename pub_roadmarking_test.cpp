#include "MessageManager.h"

class Msger
{
public:
    Msger() : tunnel{""} {};

    void pub()
    {
        tunnel.publish("ROADMARKING", &roadmarkinglist);
        std::this_thread::sleep_for(1s);
    }

public:
    zcm::ZCM tunnel;
    MsgRoadMarkingList roadmarkinglist;
};

int main()
{
    // zcm::ZCM tunnel{""};
    // MsgRoadMarkingList roadmarking;
    // for (size_t i = 0; i < 50; ++i)
    // {
    //     tunnel.publish("ROADMARKING", &roadmarking);
    //     std::this_thread::sleep_for(1s);
    // }

    MessageManager messageManager{"ipc"};
    while (true)
    {
        messageManager.publish_roadmarking();
    }
}