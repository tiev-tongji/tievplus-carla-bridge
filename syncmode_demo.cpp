#include "MessageManager.hpp"
#include <random>

static const string HOST = "127.0.0.1"; // sercer host.
static const uint16_t PORT = 2000;      // server post.
static const size_t WORKER_THREADS = 0ULL;
static const string TOWN_NAME = "Town06";
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

int main()
{
    // connect to server
    auto client = cc::Client(HOST, PORT, WORKER_THREADS);
    client.SetTimeout(3s);
    std::cout << "[INFO] Client API version : " << client.GetClientVersion() << '\n';
    std::cout << "[INFO] Server API version : " << client.GetServerVersion() << '\n';

    auto world = client.GetWorld();
    auto settings = world.GetSettings();
    settings.fixed_delta_seconds = 0.02;
    settings.synchronous_mode = true;
    world.ApplySettings(settings);

    auto map = world.GetMap();
    auto bplib = world.GetBlueprintLibrary();

    auto transform = RandomChoice(map->GetRecommendedSpawnPoints(), rng);
    auto bpVeh = *bplib->Find("vehicle.tesla.model3");
    auto actor = world.TrySpawnActor(bpVeh, transform);
    auto pActor = static_pointer_cast<cc::Vehicle>(actor);
    pActor->SetAutopilot();

    auto spectator = world.GetSpectator();

    double lastframetime = 0;

    while (true)
    {
        auto time_point = std::chrono::steady_clock::now() + std::chrono::milliseconds(1000 / 50);
        world.Tick(1s);
        auto snapshot = world.GetSnapshot();
        auto delta_seconds = snapshot.GetTimestamp().delta_seconds;
        auto now = snapshot.GetTimestamp().platform_timestamp;
        auto plattimespend = now - lastframetime;
        std::cout << "simulation time step: " << delta_seconds << "s" << std::endl;
        std::cout << "real time step: " << plattimespend << " s" << std::endl;
        lastframetime = now;
        auto trans = pActor->GetTransform();
        trans.location -= 16.0f * trans.GetForwardVector();
        trans.location.z += 10.0f;
        trans.rotation.yaw += 0.0f;
        trans.rotation.pitch = -25.0f;
        spectator->SetTransform(trans);
        std::this_thread::sleep_until(time_point);
    }
}