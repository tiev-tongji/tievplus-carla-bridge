#pragma once
#include "MessageManager/MessageManager.h"
#include "carla/client/Vehicle.h"

namespace tievsim
{
    namespace cc = carla::client;
    namespace cg = carla::geom;

    class MyPlayer
    {
    public:
        MyPlayer(carla::SharedPtr<cc::World> world);
        virtual ~MyPlayer() = default;

        carla::SharedPtr<cc::Vehicle> get_veh();
        carla::SharedPtr<cc::World> get_world();

        void SetAutoPilot();
        void SetManualControl();
        void SetTievControl();
        void Tick();

        void AddImu();
        void AddGnss();
        void AddLidar();
        void AddCameraRgb();
        void AddCameraDepth();

        void Listen();
        void StopListen();

    private:
        void ControlTick();
        void ManualControlTick();
        void NoneControlTick();
        void TievControlTick();
        void ClientSideSensorTick();

        carla::SharedPtr<cc::World> world_;
        carla::SharedPtr<cc::Vehicle> veh_;
        carla::SharedPtr<MessageManager> msg_manager_;
        carla::SharedPtr<PidController> pid_controller_;
        std::vector<carla::SharedPtr<cc::Sensor>> sensors_;

        std::string driver_;
    }
} // namespace tievsim