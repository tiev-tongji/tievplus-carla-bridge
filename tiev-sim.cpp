#include <random>

#include "MessageManager.h"
#include "PIDController.h"

#define ASYNC_MODE
//#define SYNC_MODE
//#define OPT_TIME_TEST
//#define HIL_MODE
#define AUTOPILOT_MODE

static const string HOST = "127.0.0.1"; // sercer host.
static const uint16_t PORT = 2000;      // server post.
static const size_t WORKER_THREADS = 0ULL;
static const string ZCM_URL = "udpm://239.255.76.67:7667?ttl=1";
static const string PID_PARAMETER_FILEPATH = "../cfg/pid_parameters.json";
static const string TOWN_NAME = "Town04";
static std::mt19937_64 rng((std::random_device())());

// to raise a runtime error.
#define EXPECT_TRUE(pred)                \
    if (!(pred))                         \
    {                                    \
        throw std::runtime_error(#pred); \
    }

// /// Pick a random element from @a range.
template <typename RangeT, typename RNG>
static auto &RandomChoice(const RangeT &range, RNG &&generator)
{
    EXPECT_TRUE(range.size() > 0u);
    std::uniform_int_distribution<size_t> dist{0u, range.size() - 1u};
    return range[dist(std::forward<RNG>(generator))];
}

inline cg::Transform makeTransform(double x, double y, double z, double pitch, double yaw, double roll)
{
    cg::Location loc(x, y, z);
    cg::Rotation rot(pitch, yaw, roll);
    cg::Transform trans(loc, rot);
    return trans;
}

class MyWorld
{
public:
    MyWorld(cc::World &carlaWorld, MessageManager &msgManager)
        : carlaWorld(&carlaWorld), msgManager(&msgManager){};
    ~MyWorld()
    {
        if (player.get() != nullptr)
        {
            printf("destroying player");
            player->Destroy();
        }
        for (auto s : sensorList)
        {
            s->Destroy();
        }
    };

    void setup()
    {
        bpLib = carlaWorld->GetBlueprintLibrary();
        map = carlaWorld->GetMap();
    }

    SharedPtr<cc::Vehicle> spawnPlayer(const cg::Transform &initTransform)
    {
        auto vehicleBp = bpLib->Filter("vehicle");
        auto bpPlayer = RandomChoice(*vehicleBp, rng);
        auto veh = carlaWorld->TrySpawnActor(bpPlayer, initTransform);
        player = static_pointer_cast<cc::Vehicle>(veh);
        return player;
    }

    SharedPtr<cc::Sensor> spawnSensor(const string &bpName, const cg::Transform &transfrom,
                                      SharedPtr<cc::Actor> parent)
    {
        auto sensorBp = bpLib->Find(bpName);
        EXPECT_TRUE(sensorBp != nullptr);
        auto sensorActor = carlaWorld->SpawnActor(*sensorBp, transfrom, parent.get());
        std::cout << "Spawned sensor  " << sensorActor->GetDisplayId() << '\n';
        auto sensor = static_pointer_cast<cc::Sensor>(sensorActor);
        sensorList.push_back(sensor);
        return sensor;
    }

    void init()
    {
        msgManager->vehState = player; // msgManager need ego car's state to pack tiev msgs.

        // rigister sensors' callback functions.
        for (auto s : sensorList)
        {
            if (start_with(s->GetTypeId(), "sensor.other.imu"))
            {
                s->Listen([this](auto data) {
                    auto imuMsg = static_pointer_cast<csd::IMUMeasurement>(data);
                    this->msgManager->pack_caninfo(*imuMsg);
                });
            }
            else if (start_with(s->GetTypeId(), "sensor.other.gnss"))
            {
                s->Listen([this](auto data) {
                    auto gnssMsg = static_pointer_cast<csd::GnssMeasurement>(data);
                    this->msgManager->pack_navinfo(*gnssMsg);
                });
            }
            else if (start_with(s->GetTypeId(), "sensor.lidar.ray_cast"))
            {
                s->Listen([this](auto data) {
                    auto lidarMsg = static_pointer_cast<csd::LidarMeasurement>(data);
                    this->msgManager->pack_fusionmap_lidar(*lidarMsg);
                });
            }
            else if (start_with(s->GetTypeId(), "sensor.camera.rgb"))
            {
                ;
            }
            else if (start_with(s->GetTypeId(), "sensor.camera.semantic_segmentation"))
            {
                ;
            }
            else if (start_with(s->GetTypeId(), "sensor.camera.depth"))
            {
                ;
            }
        }

        // set carla inner autopilot mode.
#ifdef AUTOPILOT_MODE
        player->SetAutopilot();
#endif
    }

    void tick(PIDController &pidController)
    {
#ifdef HIL_MODE
        double aimAcc = msgManager->CONTROL.longitudinal_acceleration_command;
        double vehAcc = msgManager->CANINFO.acceleration_x;
        double aimSteer = msgManager->CONTROL.steer_wheel_angle_command;
        double vehSteer = msgManager->CANINFO.steer_wheel_angle;
        pidController.tick(aimAcc, aimSteer, vehAcc, vehSteer, true);
        control.brake = pidController.control.brake;
        control.throttle = pidController.control.throttle;
        control.steer = pidController.control.steer;
        player->ApplyControl(control);
#endif

        msgManager->pack_objectlist(*carlaWorld->GetActors().get());
        msgManager->pack_fusionmap_raster();

#ifdef SYNC_MODE
        msgManager->publish_all();
#endif

#ifdef OPT_TIME_TEST
        auto t1 = std::chrono::steady_clock::now();
        msgManager->pack_objectlist(*(carlaWorld->GetActors().get()));
        auto t2 = std::chrono::steady_clock::now();
        double dr_ms_pack_objectlist = std::chrono::duration<double, std::milli>(t2 - t1).count();

        auto t3 = std::chrono::steady_clock::now();
        msgManager->pack_fusionmap_raster();
        auto t4 = std::chrono::steady_clock::now();
        double dr_ms_pack_fusionmap = std::chrono::duration<double, std::milli>(t4 - t3).count();

        auto t5 = std::chrono::steady_clock::now();
        msgManager->publish_objectlist();
        auto t6 = std::chrono::steady_clock::now();
        double dr_ms_pub_objectlist = std::chrono::duration<double, std::milli>(t6 - t5).count();

        auto t7 = std::chrono::steady_clock::now();
        msgManager->publish_fusionmap();
        auto t8 = std::chrono::steady_clock::now();
        double dr_ms_pub_fusionmap = std::chrono::duration<double, std::milli>(t8 - t7).count();

        auto t9 = std::chrono::steady_clock::now();
        msgManager->publish_navinfo();
        auto t10 = std::chrono::steady_clock::now();
        double dr_ms_pub_navinfo = std::chrono::duration<double, std::milli>(t10 - t9).count();

        auto t11 = std::chrono::steady_clock::now();
        msgManager->publish_caninfo();
        auto t12 = std::chrono::steady_clock::now();
        double dr_ms_pub_caninfo = std::chrono::duration<double, std::milli>(t12 - t11).count();

        std::cout << "opt time:\n"
                  << "pack objectlist:    " << dr_ms_pack_objectlist << " ms\n"
                  << "pack fusionmap:     " << dr_ms_pack_fusionmap << " ms\n"
                  << "publish objectlist: " << dr_ms_pub_objectlist << " ms\n"
                  << "publish fusionmap:  " << dr_ms_pub_fusionmap << " ms\n"
                  << "publish navinfo:    " << dr_ms_pub_navinfo << " ms\n"
                  << "publish cainfo:     " << dr_ms_pub_caninfo << " ms\n";
#endif

        // Move spectator so we can see the vehicle from the simulator window.
        auto transform = player->GetTransform();
        auto spectator = carlaWorld->GetSpectator();
        transform.location -= 16.0f * transform.GetForwardVector();
        transform.location.z += 10.0f;
        transform.rotation.yaw += 0.0f;
        transform.rotation.pitch = -25.0f;
        cg::Vector3D vec(0.0f, 0.0f, 0.0f);
        spectator->SetAngularVelocity(vec);
        spectator->SetVelocity(vec);
        spectator->SetTransform(transform);
    }

public:
    SharedPtr<cc::World> carlaWorld;
    SharedPtr<cc::Vehicle> player;
    SharedPtr<cc::BlueprintLibrary> bpLib;
    SharedPtr<cc::Map> map;
    cc::Vehicle::Control control;
    vector<SharedPtr<cc::Sensor>> sensorList;
    SharedPtr<MessageManager> msgManager;
};

void gameLoop(int16_t freq)
{
    // connect to server
    auto client = cc::Client(HOST, PORT, WORKER_THREADS);
    client.SetTimeout(10s);
    std::cout << "[INFO] Client API version : " << client.GetClientVersion() << '\n';
    std::cout << "[INFO] Server API version : " << client.GetServerVersion() << '\n';

    // init LCM messageManager and PIDController
    MessageManager msgManager(ZCM_URL);
    msgManager.TUNNEL.subscribe("CANCONTROL", &MessageManager::control_handler, &msgManager);
    msgManager.subscribe_all();
    PIDController pidController(PID_PARAMETER_FILEPATH);
#ifdef ASYNC_MODE
    msgManager.publish_all_async();
#endif
    // // get world, blueprints and map
    cc::World carlaWorld = client.LoadWorld(TOWN_NAME);
    MyWorld world(carlaWorld, msgManager);
    world.setup();

    // spawn actors
    // auto egoInitTransform = makeTransform(0, 0, 0, 0, 0, 0);
    auto egoInitTransform = RandomChoice(world.map->GetRecommendedSpawnPoints(), rng);
    world.spawnPlayer(egoInitTransform);
    //std::cout << "Spawned " << world.player->GetDisplayId() << '\n';
    std::cout << "Spawned ego car " << world.player->GetDisplayId() << '\n';
    auto gnssTransform = makeTransform(0, 0, 3, 0, 0, 0);
    auto gnss = world.spawnSensor("sensor.other.gnss", gnssTransform, world.player);
    auto imuTransform = makeTransform(0, 0, 3, 0, 0, 0);
    auto imu = world.spawnSensor("sensor.other.imu", imuTransform, world.player);
    // auto lidarTransform = makeTransform(0, 0, 3, 0, 0, 0);
    // auto lidar = world.spawnSensor("sensor.lidar.ray_cast", lidarTransform, world.player);

    world.init();
    // game loop
    while (true)
    {
        auto time_point = std::chrono::steady_clock::now() + std::chrono::milliseconds(1000 / freq);
        world.tick(pidController);
        std::this_thread::sleep_until(time_point);
    }
}

int main()
{
    gameLoop(60);
}