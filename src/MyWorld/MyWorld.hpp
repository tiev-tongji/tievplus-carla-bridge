#pragma once

#include <vector>
#include <string>
#include <map>

#include "carla/client/Client.h"
#include "carla/client/World.h"
#include "rapidjson/filereadstream.h"
#include "rapidjson/document.h"

namespace tievsim
{
    namespace cc = carla::client;
    namespace cg = carla::geom;

    class MyWorld
    {
    public:
        MyWorld() = default;
        explicit MyWorld(SharedPtr<cc::World> world);
        virtual ~MyWorld();

        void SpawnPlayer();
        void SpawnNpc();

    private:
        void PullParameter(const std::string &parmeter_filepath);

        carla::SharedPtr<cc::World> world_;
        carla::SharedPtr<cc::Vehicle> player_;
        std::map<std::string, carla::SharedPtr<cc::Sensor>> sensors_;
    };
} // namespace tievsim