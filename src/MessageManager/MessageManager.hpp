#pragma once
#include <list>
#include "MessageManager/MessageManagerBase.hpp"
#include "simutils.hpp"

#include "GeographicLib/GeoCoords.hpp"
#include "rapidjson/filereadstream.h"
#include "rapidjson/document.h"

#include "carla/client/Map.h"
#include "carla/client/Sensor.h"
#include "carla/client/Actor.h"
#include "carla/client/ActorList.h"
#include "carla/client/Vehicle.h"
#include "carla/sensor/data/GnssMeasurement.h"
#include "carla/sensor/data/IMUMeasurement.h"
#include "carla/sensor/data/LidarMeasurement.h"
#include "carla/sensor/data/Image.h"

namespace tievsim
{
    namespace cc = carla::client;
    namespace cg = carla::geom;
    namespace csd = carla::sensor::data;
    namespace cs = carla::sensor;

    class MessageManager : public MessageManagerBase
    {
    public:
        MessageManager(const std::string &url, const std::string &parameter_filepath);
        ~MessageManager() = default;

        void PackCaninfo(const csd::IMUMeasurement &imu_msg);
        void PackNavinfo(const csd::GnssMeasurement &gnss_msg);
        void PackObjectlist(const cc::ActorList &actors);
        void PackFusionmap(const csd::LidarMeasurement &lidar_msg);
        void PackRoadmarking(carla::SharedPtr<cc::Map> map);
        void PackTrafficlight();

    private:
        carla::SharedPtr<cc::Vehicle> ego_car_;
        carla::SharedPtr<cc::Sensor> lidar_;

        void PullParameter(const string &filepath);

        PredictedObject PackOneObject(const carla::SharedPtr<cc::Actor> actor);

        void RasterFusionmap();

        Lane PackLane(carla::SharedPtr<cc::Waypoint> waypoint, carla::SharedPtr<cc::Waypoint> current);

        std::list<carla::SharedPtr<cc::Waypoint>>
        GetSlice(carla::SharedPtr<cc::Waypoint> current, int *left_lane_num, int *right_lane_num);

        void ResetFusionmap();
        void ResetRoadmarking();
        void ResetObjectlist();

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