#pragma once
#include <list>
#include "MessageManagerBase.hpp"
#include "utils.hpp"

#include "GeographicLib/GeoCoords.hpp"
#include "rapidjson/filereadstream.h"
#include "rapidjson/document.h"

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
    using std::list;

    class MessageManager : public MessageManagerBase
    {
    public:
        MessageManager(const string &url, const string &parameter_filepath);
        ~MessageManager() = default;

        void PackCaninfo(const csd::IMUMeasurement &imu_msg);
        void PackNavinfo(const csd::GnssMeasurement &gnss_msg);
        void PackObjectlist(const cc::ActorList &actors);
        void PackFusionmap(const csd::LidarMeasurement &lidar_msg);
        void PackRoadmarking(SharedPtr<cc::Map> map);
        void PackTrafficlight();

    private:
        SharedPtr<cc::Vehicle> ego_car_;
        SharedPtr<cc::Sensor> lidar_;

        PredictedObject PackOneObject(const SharedPtr<cc::Actor> actor);
        void PullParameter(const string &filepath);
        void RasterFusionmap();
        Lane PackLane(SharedPtr<cc::Waypoint> waypoint, SharedPtr<cc::Waypoint> current);
        list<SharedPtr<cc::Waypoint>> GetSlice(SharedPtr<cc::Waypoint> current,
                                               int *left_lane_num, int *right_lane_num);

        // 参数表
    private:
        float kWheelbase;
        float kMass;
        cg::Location kMassCenter;
        float kWheelRadius;
        float kMaxSteer;
        float kMaxSteerWheel;
        int kPathPointNum;
        float kPathPointTimestep;
        float kMapResolution;
        int kMapRowNum;
        int kMapColNum;
        int kMapRowCenter;
        int kMapColCenter;
        int kLanelinePointNum;
        float kLanelinePointDistance;
    };

}; // namespace tievsim