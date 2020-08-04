/*
ZeroCM Message Manager

2020.2.22 by leoherz
*/
#pragma once

#include <vector>
#include <string>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <chrono>
#include <cmath>

#define USE_ZCM

#ifdef USE_ZCM
#include <zcm/zcm-cpp.hpp>
#include "MsgCanInfoSignal.hpp"
#include "MsgFusionMap.hpp"
#include "MsgNavInfoSignal.hpp"
#include "MsgRainDetectionSignal.hpp"
#include "MsgPredictedObjectTrajectoryList.hpp"
#include "MsgRoadMarkingList.hpp"
#include "MsgTrafficLightSignal.hpp"
#include "MsgTrafficSignSignal.hpp"
#include "MsgParkingSlotList.hpp"
#include "MsgChassisCommandSignal.hpp"
#endif

#ifdef USE_LCM
#include <lcm/lcm-cpp.hpp>
#include "MsgCanInfoSignal.hpp"
#include "MsgFusionMap.hpp"
#include "MsgNavInfoSignal.hpp"
#include "MsgRainDetectionSignal.hpp"
#include "MsgPredictedObjectTrajectoryList.hpp"
#include "MsgRoadMarkingList.hpp"
#include "MsgTrafficLightSignal.hpp"
#include "MsgTrafficSignSignal.hpp"
#include "MsgParkingSlotList.hpp"
#include "MsgChassisCommandSignal.hpp"
#endif

#include <GeographicLib/GeoCoords.hpp>
#include <eigen3/Eigen/Eigen>

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

namespace cc = carla::client;
namespace cg = carla::geom;
namespace csd = carla::sensor::data;
namespace cs = carla::sensor;
using boost::static_pointer_cast;
using carla::SharedPtr;
using namespace std::chrono_literals;
using std::string;
using std::vector;

const static double PI = 3.14159265358979323846;
// OBJECTLIST parameters
const static size_t POINTS_NUM_OBJECTLIST_PREDICT = 10; // including current point
const static double TIMESTEP_OBJECTLIST_PREDICT = 0.5;	// unit: second
// tiev-plus vehicle parameters
const static double MAX_STEERINGWHEEL = 500;
// FUSIONMAP parameters
const static float FUSIONMAP_RESOLUTION = 0.2;
const static int16_t MAP_ROW_NUM = 1001;
const static int16_t MAP_COLUMN_NUM = 401;
const static int16_t MAP_CENTER_ROW = 850;
const static int16_t MAP_CENTER_COLUMN = 200;
const static vector<uint8_t> MAP_ROW_0(MAP_COLUMN_NUM);
const static double LIDAR_ROTATE_FREQUENCY = 20;

inline double norm2(double x, double y, double z = 0) { return sqrt(x * x + y * y + z * z); }
inline double rad2deg(double rad) { return rad / PI * 180.0; }
inline double deg2rad(double deg) { return deg / 180.0 * PI; }
inline double kph2mps(double kph) { return kph / 3.6; }
inline double mps2kph(double mps) { return mps * 3.6; }
inline bool start_with(const std::string &s1, const std::string &s2)
{
	return s2.size() <= s1.size() && s1.compare(0, s2.size(), s2) == 0;
}
inline string get_attr(const vector<cc::ActorAttributeValue> &attrs, const string &attr_name)
{
	for (auto attr : attrs)
	{
		if (attr.GetId() == attr_name)
			return attr.GetValue();
	}
	throw "[ERROR] specified attribute not in the attributes list!";
}
inline void unreal2vehframe(const cg::Location &ego_location, const cg::Location &obj_location, double theta, cg::Location &res)
{
	double xr = obj_location.x - ego_location.x;
	double yr = obj_location.y - ego_location.y;
	double zr = obj_location.z - ego_location.z;
	double theta_rad = deg2rad(theta);
	res.x = cos(theta_rad) * xr + sin(theta_rad) * yr;
	res.y = sin(theta_rad) * xr - cos(theta_rad) * yr;
	res.z = zr;
}
inline bool in_vehframe_test(const cc::Vehicle &vehState, const cc::ActorPtr pActor)
{
	cg::Location locEgo = vehState.GetTransform().location;
	cg::Location locObj = pActor->GetTransform().location;
	float yaw = vehState.GetTransform().rotation.yaw;
	cg::Location locVehframe;
	unreal2vehframe(locEgo, locObj, yaw, locVehframe);
	float deltaZ = locObj.z - locEgo.z;
	return (locVehframe.x < (MAP_CENTER_ROW * FUSIONMAP_RESOLUTION + 3)) &&
		   (locVehframe.x > -((MAP_ROW_NUM - MAP_CENTER_ROW - 1) * FUSIONMAP_RESOLUTION + 3)) &&
		   (abs(locVehframe.y) < (MAP_CENTER_COLUMN * FUSIONMAP_RESOLUTION + 3)) && (abs(deltaZ) < 3);
}
inline bool in_area_test(double x, double y, const PredictedObject &obj)
{
	// only need to calculate the z dimension of vectors' cross products.
	// if all cross products have the same symbol, the point is located within the area.
	double z1 = (obj.bounding_box[0][1] - obj.bounding_box[0][0]) * (y - obj.bounding_box[1][0]) -
				(obj.bounding_box[1][1] - obj.bounding_box[1][0]) * (x - obj.bounding_box[0][0]);
	bool isNeg = z1 < 0;
	//printf("z1: %f\n", z1);
	double z2 = (obj.bounding_box[0][3] - obj.bounding_box[0][1]) * (y - obj.bounding_box[1][1]) -
				(obj.bounding_box[1][3] - obj.bounding_box[1][1]) * (x - obj.bounding_box[0][1]);
	//printf("z2: %f\n", z2);
	if ((isNeg && z2 >= 0) || (!isNeg && z2 < 0))
		return false; // once result's symbol is different from previous ones, return false.
	double z3 = (obj.bounding_box[0][2] - obj.bounding_box[0][3]) * (y - obj.bounding_box[1][3]) -
				(obj.bounding_box[1][2] - obj.bounding_box[1][3]) * (x - obj.bounding_box[0][3]);
	//printf("z3: %f\n", z3);
	if ((isNeg && z3 >= 0) || (!isNeg && z3 < 0))
		return false;
	double z4 = (obj.bounding_box[0][0] - obj.bounding_box[0][2]) * (y - obj.bounding_box[1][2]) -
				(obj.bounding_box[1][0] - obj.bounding_box[1][2]) * (x - obj.bounding_box[0][2]);
	//printf("z4: %f\n", z4);
	if ((isNeg && z4 >= 0) || (!isNeg && z4 < 0))
		return false;
	return true;
}

class MessageManager
{
public:
	MessageManager(std::string url) : TUNNEL(url), _need_stop(false) { CONTROL.car_gear_command = 1; };
	MessageManager() : _need_stop(false){};
	MessageManager(MessageManager const &) = delete;
	MessageManager &operator=(MessageManager const &) = delete;
	~MessageManager()
	{
		_mutex.unlock();
		_need_stop = true;
		for (auto &t : _pub_threads)
		{
			if (t.joinable())
				t.join();
		}
#ifdef USE_LCM
		for (auto &t : _sub_threads)
		{
			if (t.joinable())
				t.join();
		}
#endif
#ifdef USE_ZCM
		TUNNEL.stop();
#endif
	};

#ifdef USE_ZCM
	void control_handler(const zcm::ReceiveBuffer *rbuf, const std::string &chan,
						 const MsgChassisCommandSignal *msg);
#endif
#ifdef USE_LCM
	void control_handler(const lcm::ReceiveBuffer *rbuf, const std::string &chan,
						 const MsgChassisCommandSignal *msg);
	void sub_loop();
#endif

	void clear();

	void publish_caninfo();
	void publish_navinfo();
	void publish_fusionmap();
	void publish_objectlist();
	void publish_all();
	void publish_all_async(int freq_caninfo = 150, int freq_navinfo = 150, int freq_fusionmap = 60,
						   int freq_objectlist = 60, int freq_lanes = 60);
	void subscribe_all();

	void pack_caninfo();
	void pack_navinfo(const csd::GnssMeasurement &gnssMsg);
	void pack_objectlist(const cc::ActorList &actors);
	void pack_fusionmap_lidar(const csd::LidarMeasurement &lidarMsg);
	void pack_fusionmap_raster();

	PredictedObject pack_one_object(cc::ActorPtr pActor);

private:
	void pub_caninfo_loop(int freq);
	void pub_navinfo_loop(int freq);
	void pub_fusionmap_loop(int freq);
	void pub_objectlist_loop(int freq);
	void sub_control_loop();

public:
// all ZCM messages were made public, in order to directly access them.
#ifdef USE_ZCM
	zcm::ZCM TUNNEL;
#endif
#ifdef USE_LCM
	lcm::LCM TUNNEL;
#endif
	MsgChassisCommandSignal CONTROL;
	MsgCanInfoSignal CANINFO;
	MsgNavInfoSignal NAVINFO;
	MsgFusionMap FUSIONMAP;
	MsgPredictedObjectTrajectoryList OBJECTLIST;
	GeographicLib::GeoCoords coord;
	vector<vector<uint8_t>> MAP_HISTORY_CELLS; // to cache fusionmap cells of the previous frame.
	SharedPtr<cc::Vehicle> vehState;		   // to cache ego car's state from carla, designed for sensor callback in carla.
	vector<vector<float>> pcdRawData;

private:
	std::vector<std::thread>
		_pub_threads;
#ifdef USE_LCM
	std::vector<std::thread> _sub_threads;
#endif
	std::mutex _mutex;
	bool _need_stop;
};
