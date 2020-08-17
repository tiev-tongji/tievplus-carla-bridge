#include <random>

#include "MessageManager.h"
#include "PIDController.h"
#include <csignal>

#define ASYNC_MODE
//#define SYNC_MODE
//#define OPT_TIME_TEST

//#define HIL_MODE
//#define AUTOPILOT_MODE

static const string HOST = "127.0.0.1"; // sercer host.
static const uint16_t PORT = 2000;      // server post.
static const size_t WORKER_THREADS = 0ULL;
static const string ZCM_URL = "udpm://239.255.76.67:7667?ttl=1";
static const string PID_PARAMETER_FILEPATH = "../cfg/pid_parameters.json";
static const string TOWN_NAME = "Town06";
static const double SIM_FREQ = 50;
static std::mt19937_64 rng((std::random_device())());
static volatile bool keyboardIrruption = false;

// to raise a runtime error.
#define EXPECT_TRUE(pred)                \
    if (!(pred))                         \
    {                                    \
        throw std::runtime_error(#pred); \
    }

// Pick a random element from @a range.
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

inline cg::Transform makeTransformRelative(cg::Transform egoT, double xr, double yr, double zr)
{
    float yaw = deg2rad(egoT.rotation.yaw);
    float x = egoT.location.x + cos(yaw) * xr + sin(yaw) * yr;
    float y = egoT.location.y + sin(yaw) * xr - cos(yaw) * yr;
    float z = egoT.location.z + zr;
    return makeTransform(x, y, z, egoT.rotation.pitch, egoT.rotation.yaw, egoT.rotation.roll);
}

void sig_handler(int sig)
{
    if (sig == SIGINT)
    {
        keyboardIrruption = true;
    }
}

class MyWorld
{
public:
    MyWorld(cc::World &carlaWorld, cc::TrafficManager &tm)
        : carlaWorld(carlaWorld), tm(tm), debugHelper(carlaWorld.MakeDebugHelper()),
          msgManager(ZCM_URL), pidController(PID_PARAMETER_FILEPATH){};
    ~MyWorld()
    {
        for (auto s : sensorList)
        {
            if (s->IsAlive())
            {
                std::cout << "[INFO] Destroying sensor " << s->GetDisplayId() << std::endl;
                s->Stop();
                s->Destroy();
            }
        }
        for (auto npc : npcList)
        {
            if (npc->IsAlive())
            {
                std::cout << "[INFO] Destroying npc    " << npc->GetDisplayId() << std::endl;
                npc->Destroy();
            }
        }
        for (auto prop : propList)
        {
            if (prop->IsAlive())
            {
                std::cout << "[INFO] Destroying prop   " << prop->GetDisplayId() << std::endl;
                prop->Destroy();
            }
        }
        if (player.get() != nullptr)
        {
            if (player->IsAlive())
            {
                std::cout << "[INFO] Destroying player " << player->GetDisplayId() << std::endl;
                player->Destroy();
            }
        }
    };

    void printBlueprintLib()
    {
        for (auto bp : *bpLib)
        {
            std::cout << "blueprint ID: " << bp.GetId() << std::endl;
            for (auto it = bp.begin(); it != bp.end(); ++it)
            {
                std::cout << "-----[attribute] " << it->GetId() << std::endl;
            }
        }
    }

    void printLandmarks()
    {
        auto landmarks = map->GetAllLandmarks();
        std::cout << "-------------" << landmarks.size() << " Landmarks in Map---------------" << std::endl;
        for (auto lm : landmarks)
        {
            std::cout << ">>name: " << lm->GetName() << " type: " << lm->GetType() << std::endl;
        }
    }

    void setup()
    {
        bpLib = carlaWorld.GetBlueprintLibrary();
        map = carlaWorld.GetMap();
    }

    void initMessager()
    {
        msgManager.vehState = player; // msgManager need ego car's state to pack tiev msgs.
        msgManager.TUNNEL.subscribe("MsgChassisCommandSignal", &MessageManager::control_handler, &msgManager);
        msgManager.subscribe_all();
#ifdef ASYNC_MODE
        msgManager.publish_all_async(100, 100, 20, 20, 20, 20);
#endif
    }

    void rigisterSensorCallback()
    {
        for (auto s : sensorList)
        {
            if (start_with(s->GetTypeId(), "sensor.other.imu"))
            {
                ;
            }
            else if (start_with(s->GetTypeId(), "sensor.other.gnss"))
            {
                s->Listen([this](auto data) {
                    auto gnssMsg = static_pointer_cast<csd::GnssMeasurement>(data);
                    this->msgManager.pack_navinfo(*gnssMsg);
                });
            }
            else if (start_with(s->GetTypeId(), "sensor.lidar.ray_cast"))
            {
                s->Listen([this](auto data) {
                    auto lidarMsg = static_pointer_cast<csd::LidarMeasurement>(data);
                    this->msgManager.pack_fusionmap_lidar(*lidarMsg);
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
    }

    void setServerSyncMode()
    {
        // more precision of simulation: async mode, fixed delta_time
        auto worldSettings = carlaWorld.GetSettings();
        worldSettings.synchronous_mode = true;
        worldSettings.fixed_delta_seconds = 1.0 / SIM_FREQ;
        // worldSettings.fixed_delta_seconds = 0.02;
        carlaWorld.ApplySettings(worldSettings);
    }

    void init()
    {
        spectator = carlaWorld.GetSpectator();
        initMessager();
        rigisterSensorCallback();
        //setServerSyncMode();
#ifdef AUTOPILOT_MODE
        // set carla inner autopilot mode
        player->SetAutopilot();
#endif
    }

    void relocateSpectator2egoCar()
    {
        // Move spectator so we can see the vehicle from the simulator window.
        auto transform = player->GetTransform();
        transform.location -= 16.0f * transform.GetForwardVector();
        transform.location.z += 10.0f;
        transform.rotation.yaw += 0.0f;
        transform.rotation.pitch = -25.0f;
        cg::Vector3D vec(0.0f, 0.0f, 0.0f);
        spectator->SetAngularVelocity(vec);
        spectator->SetVelocity(vec);
        spectator->SetTransform(transform);
    }

    void forceLaneChange(SharedPtr<cc::Vehicle> npc, double dist, bool toLeft)
    {
        static bool changed = false;
        auto loc = unreal2vehframe(player->GetLocation(), npc->GetLocation(), player->GetTransform().rotation.yaw);
        std::cout << "lanechange control, loc.x: " << loc.x << " dist: " << dist << std::endl;
        if (loc.x > dist && !changed)
        {
            tm.SetForceLaneChange(npc, toLeft);
            std::cout << "!!!!!!!!!!!!!!!HAVE CHANGED!!!!!!!!!!!!" << std::endl;
            changed = true;
        }
    }

    void tick()
    {
        //carlaWorld.Tick(10s);
        carlaWorld.WaitForTick(1s);
        relocateSpectator2egoCar();

#ifdef HIL_MODE
        double aimAcc = msgManager.CONTROL.longitudinal_acceleration_command;
        double vehAcc = msgManager.CANINFO.acceleration_x;
        double aimSteer = -msgManager.CONTROL.steer_wheel_angle_command;
        double vehSteer = -msgManager.CANINFO.steer_wheel_angle;
        pidController.tick(aimAcc, aimSteer, vehAcc, vehSteer, true);
        control.brake = pidController.control.brake;
        control.throttle = pidController.control.throttle;
        control.steer = pidController.control.steer;

        control.reverse = msgManager.CONTROL.car_gear_command == 2;
        control.hand_brake = msgManager.CONTROL.car_gear_command == 1;
        player->ApplyControl(control);
#endif

        msgManager.pack_caninfo();
        msgManager.pack_objectlist(*carlaWorld.GetActors());
        msgManager.pack_fusionmap_raster();
        msgManager.pack_roadmarking(map->GetWaypoint(player->GetLocation()), debugHelper, true);

#ifdef SYNC_MODE
        msgManager.publish_all();
#endif

#ifdef OPT_TIME_TEST
        auto t1 = std::chrono::steady_clock::now();
        msgManager.pack_objectlist(*carlaWorld.GetActors());
        auto t2 = std::chrono::steady_clock::now();
        double dr_ms_pack_objectlist = std::chrono::duration<double, std::milli>(t2 - t1).count();

        auto t3 = std::chrono::steady_clock::now();
        msgManager.pack_fusionmap_raster();
        auto t4 = std::chrono::steady_clock::now();
        double dr_ms_pack_fusionmap = std::chrono::duration<double, std::milli>(t4 - t3).count();

        auto t5 = std::chrono::steady_clock::now();
        msgManager.publish_objectlist();
        auto t6 = std::chrono::steady_clock::now();
        double dr_ms_pub_objectlist = std::chrono::duration<double, std::milli>(t6 - t5).count();

        auto t7 = std::chrono::steady_clock::now();
        msgManager.publish_fusionmap();
        auto t8 = std::chrono::steady_clock::now();
        double dr_ms_pub_fusionmap = std::chrono::duration<double, std::milli>(t8 - t7).count();

        auto t9 = std::chrono::steady_clock::now();
        msgManager.publish_navinfo();
        auto t10 = std::chrono::steady_clock::now();
        double dr_ms_pub_navinfo = std::chrono::duration<double, std::milli>(t10 - t9).count();

        auto t11 = std::chrono::steady_clock::now();
        msgManager.publish_caninfo();
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
    }

    SharedPtr<cc::Vehicle> spawnPlayer(const cg::Transform &transform, const string &blueprintName)
    {

        auto bpPlayer = *bpLib->Find(blueprintName);
        auto vehEgo = carlaWorld.TrySpawnActor(bpPlayer, transform);
        std::cout << "[INFO] Spawned player  " << vehEgo->GetDisplayId() << '\n';
        player = static_pointer_cast<cc::Vehicle>(vehEgo);
        cg::Vector3D vec(0, 0, 0);
        player->SetVelocity(vec);
        player->SetAngularVelocity(vec);
        auto egoPhysics = player->GetPhysicsControl();
        // std::cout << "wheel radius: " << egoPhysics.wheels[0].radius << std::endl;
        // std::cout << "mass center: (x,y,z) = (" << egoPhysics.center_of_mass.x << ", "
        //           << egoPhysics.center_of_mass.y << ", "
        //           << egoPhysics.center_of_mass.z << std::endl;
        // std::cout << "use auto gear-box: " << egoPhysics.use_gear_autobox << std::endl;
        egoPhysics.mass = 1854;
        auto massCenter = egoPhysics.center_of_mass;
        massCenter.x = 1.436;
        massCenter.y = 0;
        massCenter.z = -0.526;
        egoPhysics.wheels[0].radius = 33;
        egoPhysics.wheels[0].max_steer_angle = 54;
        egoPhysics.wheels[1].radius = 33;
        egoPhysics.wheels[1].max_steer_angle = 54;
        egoPhysics.wheels[2].radius = 33;
        egoPhysics.wheels[3].radius = 33;
        player->ApplyPhysicsControl(egoPhysics);
        auto playerInitControl = player->GetControl();
        playerInitControl.hand_brake = true;
        player->ApplyControl(playerInitControl);
        return player;
    }

    SharedPtr<cc::Vehicle> spawnNpc(const cg::Transform &transform, const string &blueprintName = "random")
    {
        SharedPtr<cc::BlueprintLibrary> vehicleBp;
        if (blueprintName == "" || blueprintName == "random")
        {
            vehicleBp = bpLib->Filter("vehicle");
        }
        else
        {
            vehicleBp = bpLib->Filter(blueprintName);
        }
        auto bpTarget = RandomChoice(*vehicleBp, rng);
        auto target = carlaWorld.TrySpawnActor(bpTarget, transform);
        std::cout << "[INFO] Spawned npc     " << target->GetDisplayId() << '\n';
        auto vehTarget = static_pointer_cast<cc::Vehicle>(target);
        npcList.push_back(vehTarget);
        vehTarget->SetVelocity(cg::Vector3D{0, 0, 0});
        vehTarget->SetAngularVelocity(cg::Vector3D{0, 0, 0});
        //auto targetInitControl = vehTarget->GetControl();
        //targetInitControl.hand_brake = true;
        //vehTarget->ApplyControl(targetInitControl);
        return vehTarget;
    }

    SharedPtr<cc::Actor> spawnStatic(const cg::Transform &transform, const string &blueprintName)
    {
        auto propBpLib = bpLib->Filter(blueprintName);
        auto bpProp = RandomChoice(*propBpLib, rng);
        auto prop = carlaWorld.TrySpawnActor(bpProp, transform);
        std::cout << "[INFO] Spawned prop    " << prop->GetDisplayId() << '\n';
        prop->SetSimulatePhysics(true);
        propList.push_back(prop);
        return prop;
    }

    SharedPtr<cc::Sensor> spawnGnss(cg::Transform &relativeTransform, cc::Actor *parent = 0)
    {
        auto gnssBp = *bpLib->Find("sensor.other.gnss");
        cc::Actor *attachTo = parent;
        if (!parent)
        {
            attachTo = player.get();
        }
        auto actorGnss = carlaWorld.SpawnActor(gnssBp, relativeTransform, attachTo);
        std::cout << "[INFO] Spawned sensor  " << actorGnss->GetDisplayId() << '\n';
        auto sensorGnss = static_pointer_cast<cc::Sensor>(actorGnss);
        sensorList.push_back(sensorGnss);
        return sensorGnss;
    }

    SharedPtr<cc::Sensor> spawnLidar(cg::Transform &relativeTransform, cc::Actor *parent = 0)
    {
        auto lidarBp = *bpLib->Find("sensor.lidar.ray_cast");
        lidarBp.SetAttribute("channels", "64");              // Number of lasers
        lidarBp.SetAttribute("range", "150.0");              // Maximum distance to measure/raycast in meters
        lidarBp.SetAttribute("points_per_second", "128000"); // Points generated by all lasers per second
        lidarBp.SetAttribute("rotation_frequency", "20");    // Lidar rotation frequency
        lidarBp.SetAttribute("upper_fov", "10.0");           // Angle in degrees of the highest laser
        lidarBp.SetAttribute("lower_fov", "-30.0");          // Angle in degrees of the lowest laser
        lidarBp.SetAttribute("sensor_tick", "0.0");          // Simulation seconds between sensor captures
        cc::Actor *attachTo = parent;
        if (!parent)
        {
            attachTo = player.get();
        }
        auto actorLidar = carlaWorld.TrySpawnActor(lidarBp, relativeTransform, attachTo);
        std::cout << "[INFO] Spawned sensor  " << actorLidar->GetDisplayId() << "\n";
        auto sensorLidar = static_pointer_cast<cc::Sensor>(actorLidar);
        sensorList.push_back(sensorLidar);
        return sensorLidar;
    }

    SharedPtr<cc::Sensor> spawnCamera(cg::Transform &relativeTransform, cc::Actor *parent = 0)
    {
        cc::Actor *attachTo = parent;
        if (!parent)
        {
            attachTo = player.get();
        }
        auto cameraBp = *bpLib->Find("sensor.camera.rgb");
        auto actorCamera = carlaWorld.SpawnActor(cameraBp, relativeTransform, attachTo);
        std::cout << "[INFO] Spawned sensor " << actorCamera->GetDisplayId() << '\n';
        auto sensorCamera = static_pointer_cast<cc::Sensor>(actorCamera);
        sensorList.push_back(sensorCamera);
        return sensorCamera;
    }

    SharedPtr<cc::Sensor> spawnImu(cg::Transform &relativeTransform, cc::Actor *parent = 0)
    {
        cc::Actor *attachTo = parent;
        if (!parent)
        {
            attachTo = player.get();
        }
        auto imuBp = *bpLib->Find("sensor.other.imu");
        auto actorImu = carlaWorld.SpawnActor(imuBp, relativeTransform, attachTo);
        std::cout << "[INFO] Spawned sensor  " << actorImu->GetDisplayId() << '\n';
        auto sensorImu = static_pointer_cast<cc::Sensor>(actorImu);
        sensorList.push_back(sensorImu);
        return sensorImu;
    }

public:
    cc::World &carlaWorld;
    cc::TrafficManager &tm;
    SharedPtr<cc::BlueprintLibrary> bpLib;
    SharedPtr<cc::Map> map;
    cc::DebugHelper debugHelper;

    SharedPtr<cc::Vehicle> player;
    SharedPtr<cc::Actor> spectator;
    vector<SharedPtr<cc::Sensor>> sensorList;
    vector<SharedPtr<cc::Vehicle>> npcList;
    vector<SharedPtr<cc::Actor>> propList;

    cc::Vehicle::Control control;
    MessageManager msgManager;
    PIDController pidController;
};

void gameLoop(int16_t freq)
{
    // connect to server
    auto client = cc::Client(HOST, PORT, WORKER_THREADS);
    client.SetTimeout(3s);
    std::cout << "[INFO] Client API version : " << client.GetClientVersion() << '\n';
    std::cout << "[INFO] Server API version : " << client.GetServerVersion() << '\n';

    // get world, blueprints and map
    //cc::World carlaWorld = client.LoadWorld(TOWN_NAME);
    cc::World carlaWorld = client.GetWorld();
    cc::TrafficManager tm = client.GetInstanceTM();
    MyWorld world(carlaWorld, tm);
    world.setup();

    //cg::Transform egoT = RandomChoice(world.map->GetRecommendedSpawnPoints(), rng);
    cg::Transform egoT = makeTransform(25.527479, 146.448837, 3.5, 0, 0.5, 0); // TOWN06 straight road with 5 lanes
    cg::Transform egoT = makeTransform(, , 3, 0, 0, , 0);                      // TOWN03 0817
    world.spawnPlayer(egoT, "vehicle.tesla.model3");
    cg::Transform sensorOffset = makeTransform(0, 0, 3.5, 0, 0, 0);
    world.spawnGnss(sensorOffset);
    //world.spawnLidar(sensorOffset);
    //cg::Transform target1T = makeTransformRelative(egoT, -5, 3.5, 0);
    //auto npc = world.spawnNpc(target1T, "vehicle.audi.tt");
    //cg::Transform target2T = makeTransformRelative(egoT, 60, 0, 0);

    world.init();

    //npc->SetAutopilot(true);
    //world.spawnNpc(target2T)->SetAutopilot(true);

    // game loop
    while (!keyboardIrruption)
    {
        //auto time_point = std::chrono::steady_clock::now() + std::chrono::milliseconds(1000 / freq);
        world.tick();
        //world.forceLaneChange(npc, 2, true);
        //std::this_thread::sleep_until(time_point);
    }
}

int main()
{
    signal(SIGINT, sig_handler);
    gameLoop(SIM_FREQ);
}