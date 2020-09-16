#pragma once
#include <map>

#include "MessageManager/MessageManager.hpp"
#include "VehicleController/TievplusController.hpp"

#include "carla/client/Vehicle.h"
#include "carla/client/Map.h"

namespace tievsim
{
    namespace cc = carla::client;
    namespace cg = carla::geom;

    class MyPlayer
    {
    public:
        MyPlayer();
        virtual ~MyPlayer() = default;

        carla::SharedPtr<cc::Vehicle> get_veh();

        void SetAutoPilot();
        void SetManualControl();
        void SetTievControl();

        void ControlTick();
        void ClientSideSensorTick(
            int frame_duration_objectlist, carla::SharedPtr<cc::ActorList> actor_list,
            int frame_duration_roadmarking, carla::SharedPtr<cc::Map> map);

        void AddImu();
        void AddGnss();
        void AddLidar();
        void AddCameraRgb();
        void AddCameraDepth();

        void Listen();
        void StopListen(const string &name = "all");

    private:
        void ManualControlTick();
        void NoneControlTick();
        void TievControlTick();

        carla::SharedPtr<cc::Vehicle> veh_;
        carla::SharedPtr<MessageManager> msg_manager_;
        carla::SharedPtr<TievplusController> pid_controller_;
        std::map<string, carla::SharedPtr<cc::Sensor>> sensors_;

        std::string driver_;

        int cnt_objectlist_;
        int cnt_roadmarking_;
    };
} // namespace tievsim