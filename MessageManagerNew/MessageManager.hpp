#pragma once
#include "MessageManagerBase.hpp"
#include "utils.hpp"
#include "GeographicLib/GeoCoords.hpp"

#include "carla/client/Client.h"
#include "carla/client/World.h"
#include "carla/client/Map.h"
#include "carla/client/Sensor.h"
#include "carla/client/Actor.h"
#include "carla/client/ActorList.h"
#include "carla/client/detail/ActorState.h"
#include "carla/client/Vehicle.h"
#include "carla/client/Walker.h"
#include "carla/client/BlueprintLibrary.h"
#include "carla/sensor/data/GnssMeasurement.h"
#include "carla/sensor/data/IMUMeasurement.h"
#include "carla/sensor/data/LidarMeasurement.h"
#include "carla/sensor/data/Image.h"
#include "carla/client/DebugHelper.h"
#include "carla/trafficmanager/TrackTraffic.h"

namespace tievsim
{
    namespace cc = carla::client;
    namespace cg = carla::geom;
    namespace csd = carla::sensor::data;
    namespace cs = carla::sensor;
    using carla::SharedPtr;

    class MessageManager : public MessageManagerBase
    {
    public:
        MessageManager(const string &url);
        ~MessageManager() = default;

        void PackCaninfo(const csd::IMUMeasurement &imu_msg);
        void PackNavinfo(const csd::GnssMeasurement &gnss_msg);
        void PackObjectlist();
        void PackFusionmap();
        void PackRoadmarking();
        void PackTrafficlight();

    private:
        SharedPtr<cc::Vehicle> ego_car_;
        SharedPtr<cc::Sensor> lidar_;

        PredictedObject PackOneObject();
    };

}; // namespace tievsim