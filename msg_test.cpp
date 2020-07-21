#include "MessageManager.h"
#include "PIDController.h"
#include <random>

static const string HOST = "127.0.0.1"; // sercer host.
static const uint16_t PORT = 2000;      // server post.
static const size_t WORKER_THREADS = 0ULL;
static const string ZCM_URL = "udpm://239.255.76.67:7667?ttl=1";
static const string PID_PARAMETER_FILEPATH = "../cfg/pid_parameters.json";
static const string TOWN_NAME = "Town03";
static std::mt19937_64 rng((std::random_device())());
static volatile bool keyboardIrruption = false;

int main()
{
    auto client = cc::Client(HOST, PORT, WORKER_THREADS);
    client.SetTimeout(10s);
    std::cout << "[INFO] Client API version : " << client.GetClientVersion() << '\n';
    std::cout << "[INFO] Server API version : " << client.GetServerVersion() << '\n';

    // init LCM messageManager and PIDController
    MessageManager msgManager(ZCM_URL);
    msgManager.TUNNEL.subscribe("CANCONTROL", &MessageManager::control_handler, &msgManager);
    msgManager.subscribe_all();
    PIDController pidController(PID_PARAMETER_FILEPATH);

    SharedPtr<cc::World> carlaWorld;
    SharedPtr<cc::Vehicle> player;
    SharedPtr<cc::BlueprintLibrary> bpLib;
    SharedPtr<cc::Map> map;
    cc::Vehicle::Control control;
    vector<SharedPtr<cc::Sensor>> sensorList;
    vector<SharedPtr<cc::Vehicle>> actorList;
    MessageManager *msgManagerPtr;
    //SharedPtr<MessageManager> msgManagerPtr(&msgManager);
}