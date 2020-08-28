#pragma once

#include <vector>
#include <string>

#include "carla/client/Client.h"
#include "carla/client/World.h"

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

    private:
        carla::SharedPtr<cc::World> world_;
        carla::SharedPtr<cc::Vehicle> player_;
        vector<carla::SharedPtr<cc::Actor>> npclist_;
    };
} // namespace tievsim