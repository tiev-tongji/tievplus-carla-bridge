#include "MessageManager.h"

#ifdef USE_ZCM
void MessageManager::control_handler(const zcm::ReceiveBuffer *rbuf, const std::string &chan,
									 const MsgChassisCommandSignal *msg)
{
	CONTROL.timestamp = msg->timestamp;
	CONTROL.autonomous_mode_control_command = msg->autonomous_mode_control_command;
	CONTROL.car_gear_command = msg->car_gear_command;
	CONTROL.steer_wheel_angle_command = msg->steer_wheel_angle_command;
	CONTROL.longitudinal_acceleration_command = msg->longitudinal_acceleration_command;
}

void MessageManager::subscribe_all()
{
	TUNNEL.start();
}
#endif

#ifdef USE_LCM
void MessageManager::control_handler(const lcm::ReceiveBuffer *rbuf, const std::string &chan,
									 const MsgChassisCommandSignal *msg)
{
	CONTROL.timestamp = msg->timestamp;
	CONTROL.autonomous_mode_control_command = msg->autonomous_mode_control_command;
	CONTROL.car_gear_command = msg->car_gear_command;
	CONTROL.steer_wheel_angle_command = msg->steer_wheel_angle_command;
	CONTROL.longitudinal_acceleration_command = msg->longitudinal_acceleration_command;
}

void MessageManager::sub_loop()
{
	while (!_need_stop)
	{
		//_mutex.lock();
		TUNNEL.handle();
		//_mutex.unlock();
	}
}

void MessageManager::subscribe_all()
{
	_sub_threads.push_back(std::thread(&MessageManager::sub_loop, this));
	for (auto &t : _sub_threads)
	{
		t.detach();
	}
}
#endif

void MessageManager::clear()
{
	_mutex.lock();
	// clear objects in the list every epoch
	OBJECTLIST.predicted_object.clear();
	OBJECTLIST.object_count = 0;
	// cache and clear map cells every epoch
	MAP_HISTORY_CELLS = FUSIONMAP.map_cells;
	FUSIONMAP.map_cells.assign(FUSIONMAP.map_cells.size(), MAP_ROW_0);
	_mutex.unlock();
}

void MessageManager::publish_caninfo()
{
	TUNNEL.publish("CANINFO", &CANINFO);
}

void MessageManager::publish_navinfo()
{
	TUNNEL.publish("NAVINFO", &NAVINFO);
}

void MessageManager::publish_fusionmap()
{
	TUNNEL.publish("FUSIONMAP", &FUSIONMAP);
}

void MessageManager::publish_objectlist()
{
	TUNNEL.publish("PREDICTEDOBJECT", &OBJECTLIST);
}

void MessageManager::publish_all()
{
	publish_caninfo();
	publish_navinfo();
	publish_fusionmap();
	publish_objectlist();
}

void MessageManager::pub_caninfo_loop(int freq)
{
	while (!_need_stop)
	{
		auto time_point = std::chrono::steady_clock::now() + std::chrono::milliseconds(1000 / freq);
		_mutex.lock();
		TUNNEL.publish("CANINFO", &CANINFO);
		//printf("async mode: publish CANINFO\n");
		_mutex.unlock();
		std::this_thread::sleep_until(time_point);
	}
}

void MessageManager::pub_navinfo_loop(int freq)
{
	while (!_need_stop)
	{
		auto time_point = std::chrono::steady_clock::now() + std::chrono::milliseconds(1000 / freq);
		_mutex.lock();
		TUNNEL.publish("NAVINFO", &NAVINFO);
		//printf("async mode: publish NAVINFO\n");
		_mutex.unlock();
		std::this_thread::sleep_until(time_point);
	}
}

void MessageManager::pub_fusionmap_loop(int freq)
{
	while (!_need_stop)
	{
		auto time_point = std::chrono::steady_clock::now() + std::chrono::milliseconds(1000 / freq);
		_mutex.lock();
		TUNNEL.publish("FUSIONMAP", &FUSIONMAP);
		//print("async mode: publish FUSIONMAP");
		_mutex.unlock();
		std::this_thread::sleep_until(time_point);
	}
}

void MessageManager::pub_objectlist_loop(int freq)
{
	while (!_need_stop)
	{
		auto time_point = std::chrono::steady_clock::now() + std::chrono::milliseconds(1000 / freq);
		_mutex.lock();
		TUNNEL.publish("PREDICTEDOBJECT", &OBJECTLIST);
		//print("async mode: publish OBJECTLIST");
		_mutex.unlock();
		std::this_thread::sleep_until(time_point);
	}
}

void MessageManager::publish_all_async(int freq_caninfo, int freq_navinfo, int freq_fusionmap,
									   int freq_objectlist, int freq_lanes)
{
	_pub_threads.push_back(std::thread(&MessageManager::pub_caninfo_loop, this, freq_caninfo));
	_pub_threads.push_back(std::thread(&MessageManager::pub_navinfo_loop, this, freq_navinfo));
	_pub_threads.push_back(std::thread(&MessageManager::pub_objectlist_loop, this, freq_objectlist));
	_pub_threads.push_back(std::thread(&MessageManager::pub_fusionmap_loop, this, freq_fusionmap));

	for (auto &t : _pub_threads)
	{
		t.detach();
	}
}

void MessageManager::pack_caninfo(const csd::IMUMeasurement &imuMsg)
{
	_mutex.lock();
	// only need velocity, yaww_rate, steer_wheel_angle, acceleration_x/y
	CANINFO.timestamp = imuMsg.GetTimestamp() * 1000;

	CANINFO.drive_mode = 0;
	CANINFO.epb_mode = 0;
	CANINFO.eps_mode = 0;
	CANINFO.esp_mode = 0;
	CANINFO.gear_mode = 0;
	CANINFO.motor_mode = 0;
	CANINFO.eps_permission = 0;
	CANINFO.esp_permission = 0;

	CANINFO.gear_state = vehState->GetControl().gear; // TODO
	CANINFO.epb_state = 0;
	CANINFO.brake_pedal_state = vehState->GetControl().brake == 0 ? 0 : 1;

	auto vel = vehState->GetVelocity();
	CANINFO.velocity = mps2kph(norm2(vel.x, vel.y, vel.z));
	CANINFO.wheel_speed_fl = 0;
	CANINFO.wheel_speed_fr = 0;
	CANINFO.wheel_speed_rl = 0;
	CANINFO.wheel_speed_rr = 0;

	CANINFO.yaw_rate = deg2rad(-vehState->GetAngularVelocity().z);

	CANINFO.acceleration_x = imuMsg.GetAccelerometer().x;
	CANINFO.acceleration_y = imuMsg.GetAccelerometer().y;

	CANINFO.steer_wheel_angle = -vehState->GetControl().steer * MAX_STEERINGWHEEL;
	CANINFO.steer_angular_speed = 0;

	CANINFO.motor_torque = 0;
	CANINFO.brake_deepness = vehState->GetControl().brake;
	CANINFO.accelerate_deepness = vehState->GetControl().throttle;

	CANINFO.lamp_turn_l = 0;
	CANINFO.lamp_turn_r = 0;
	CANINFO.lamp_brake = 0;

	CANINFO.acceleration_x_desired = 0;
	CANINFO.steer_wheel_angle_desired = 0;
	CANINFO.emergency_control_state = 0;

	_mutex.unlock();
};

void MessageManager::pack_navinfo(const csd::GnssMeasurement &gnssMsg)
{
	_mutex.lock();

	auto loc = vehState->GetTransform().location;
	auto rot = vehState->GetTransform().rotation;

	NAVINFO.timestamp = gnssMsg.GetTimestamp() * 1000;
	// position
	// GeographicLib::GeoCoords coord("121:12:44E 31:16:54N"); // geographic reference point relocated to tongji
	// NAVINFO.utm_x = coord.Easting() + loc.x;
	// NAVINFO.utm_y = coord.Northing() - loc.y;
	// coord.Reset(coord.Zone(), coord.Northp(), NAVINFO.utm_x, NAVINFO.utm_y);
	// NAVINFO.latitude = coord.Latitude();
	// NAVINFO.longitude = coord.Longitude();
	// NAVINFO.altitude = gnssMsg.GetAltitude();

	NAVINFO.latitude = gnssMsg.GetLatitude();
	NAVINFO.longitude = gnssMsg.GetLongitude();
	NAVINFO.altitude = gnssMsg.GetAltitude();
	coord.Reset(NAVINFO.latitude, NAVINFO.longitude);
	NAVINFO.utm_x = coord.Easting();
	NAVINFO.utm_y = coord.Northing();

	//printf("UE4 position: (%f, %f, %f, %f, %f, %f)\n", loc.x, loc.y, loc.z, rot.roll, rot.pitch, rot.yaw);
	//printf("GPS: (%f, %f)\n", NAVINFO.latitude, NAVINFO.longitude);
	//printf("error: (%f, %f)\n", NAVINFO.utm_x - loc.x, NAVINFO.utm_y + loc.y);

	// rotation
	/*
	Carla use Unreal-Engine coordinate system, left-hand
	X, Y, Z binded with East, South, Up
	rotation around Z (yaw): clockwise as positive
	rotation around X and Y (roll and pitch): counter-clockwise as positive
	*/
	float heading = -rot.yaw;
	if (heading < -180)
		heading = heading + 360;
	else if (heading > 180)
		heading = heading - 360;
	NAVINFO.angle_head = deg2rad(heading);
	// printf("UTM: ( E%f, N%f )   heading: %f\n", NAVINFO.utm_x, NAVINFO.utm_y, NAVINFO.angle_head);
	NAVINFO.angle_pitch = rot.pitch;
	NAVINFO.angle_roll = rot.roll;
	auto rot_vel = vehState->GetAngularVelocity();
	NAVINFO.angular_vel_z = deg2rad(-rot_vel.z);
	// speed, velocity, acceleration
	// TODO:rear axle to front axle
	auto vel = vehState->GetVelocity();
	NAVINFO.speed = norm2(vel.x, vel.y);
	NAVINFO.velocity_east = vel.x;
	NAVINFO.velocity_north = -vel.y; // East-South-Up coordinate in Carla.
	NAVINFO.acceleration_x = CANINFO.acceleration_x;
	NAVINFO.acceleration_y = CANINFO.acceleration_y;
	//status
	NAVINFO.curvature = 0;
	NAVINFO.HPOS_accuracy = 0.01;
	NAVINFO.RTK_status = 1;
	NAVINFO.gps_num_satellites = 11;
	NAVINFO.is_reckoning_vaild = 1;

	_mutex.unlock();
};

PredictedObject MessageManager::pack_one_object(cc::ActorPtr pActor)
{
	PredictedObject predObj;
	predObj.id = pActor->GetId();
	auto typeID = pActor->GetTypeId();
	auto attrs = pActor->GetAttributes();
	if (start_with(typeID, "vehicle") && std::stoi(get_attr(attrs, "number_of_wheels")) >= 4)
		predObj.type = 0;
	else if (start_with(typeID, "vehicle") && std::stoi(get_attr(attrs, "number_of_wheels")) <= 2)
		predObj.type = 1;
	else if (start_with(typeID, "walker.pedestrian"))
		predObj.type = 2;
	else
		predObj.type = 127; // for unknown type

	auto vel = pActor->GetVelocity();
	predObj.velocity = norm2(vel.x, vel.y, vel.z);
	auto acc = pActor->GetAcceleration();
	predObj.accelerate = norm2(acc.x, acc.y, acc.z);

	auto rotEgo = vehState->GetTransform().rotation;
	auto rot = pActor->GetTransform().rotation;
	// float heading = 90 + rotEgo.yaw - rot.yaw; // relative
	float heading = -rot.yaw; // relative
	if (heading < -180)
		heading = heading + 360;
	else if (heading > 180)
		heading = heading - 360;
	predObj.heading = deg2rad(heading);

	predObj.trajectory_point_num = POINTS_NUM_OBJECTLIST_PREDICT;
	predObj.trajectory_point.resize(2, std::vector<float>(POINTS_NUM_OBJECTLIST_PREDICT));
	auto locEgo = vehState->GetTransform().location;
	auto loc = pActor->GetTransform().location;
	for (int i = 0; i < POINTS_NUM_OBJECTLIST_PREDICT; ++i)
	{
		cg::Location locPred;
		double t = TIMESTEP_OBJECTLIST_PREDICT;
		locPred.x = loc.x + i * t * vel.x + 0.5 * pow(i * t, 2) * pActor->GetAcceleration().x;
		locPred.y = loc.y + i * t * vel.y + 0.5 * pow(i * t, 2) * pActor->GetAcceleration().y;
		//printf("locEgo (%4f, %4f, %4f)\n", locEgo.x, locEgo.y, locEgo.z);
		//printf("locPred (%4f, %4f, %4f\n", locPred.x, locPred.y, locPred.z);
		cg::Location locVehframe;
		unreal2vehframe(locEgo, locPred, rotEgo.yaw, locVehframe);
		predObj.trajectory_point[0][i] = locVehframe.x;
		predObj.trajectory_point[1][i] = locVehframe.y;
	}

	auto bb = pActor->Serialize().bounding_box;
	predObj.length = pActor->Serialize().bounding_box.extent.x * 2;
	predObj.width = pActor->Serialize().bounding_box.extent.y * 2;
	vector<cg::Location> vertexs(4);
	double obj_yaw = deg2rad(rot.yaw);

	vertexs[0].x = loc.x + (bb.location.x + bb.extent.x) * cos(obj_yaw) - (bb.location.y + bb.extent.y) * sin(obj_yaw);
	vertexs[0].y = loc.y + (bb.location.x + bb.extent.x) * sin(obj_yaw) + (bb.location.y + bb.extent.y) * cos(obj_yaw);

	vertexs[1].x = loc.x + (bb.location.x + bb.extent.x) * cos(obj_yaw) - (bb.location.y - bb.extent.y) * sin(obj_yaw);
	vertexs[1].y = loc.y + (bb.location.x + bb.extent.x) * sin(obj_yaw) + (bb.location.y - bb.extent.y) * cos(obj_yaw);

	vertexs[2].x = loc.x + (bb.location.x - bb.extent.x) * cos(obj_yaw) - (bb.location.y - bb.extent.y) * sin(obj_yaw);
	vertexs[2].y = loc.y + (bb.location.x - bb.extent.x) * sin(obj_yaw) + (bb.location.y - bb.extent.y) * cos(obj_yaw);

	vertexs[3].x = loc.x + (bb.location.x - bb.extent.x) * cos(obj_yaw) - (bb.location.y + bb.extent.y) * sin(obj_yaw);
	vertexs[3].y = loc.y + (bb.location.x - bb.extent.x) * sin(obj_yaw) + (bb.location.y + bb.extent.y) * cos(obj_yaw);

	for (auto &vertex : vertexs)
	{
		unreal2vehframe(locEgo, vertex, rotEgo.yaw, vertex);
	}
	for (int i = 0; i < 4; ++i)
	{
		//printf("vertex 1: (%f, %f)\n", vertexs[i].x, vertexs[i].y);
		if (vertexs[i].y >= predObj.trajectory_point[1][0])
		{
			if (vertexs[i].x <= predObj.trajectory_point[0][0])
			{
				// left-bottom vertex
				predObj.bounding_box[0][1] = vertexs[i].x;
				predObj.bounding_box[1][1] = vertexs[i].y;
			}
			else
			{
				// left-up vertex
				predObj.bounding_box[0][0] = vertexs[i].x;
				predObj.bounding_box[1][0] = vertexs[i].y;
			}
		}
		else
		{
			if (vertexs[i].x <= predObj.trajectory_point[0][0])
			{
				// right-bottom vertex
				predObj.bounding_box[0][3] = vertexs[i].x;
				predObj.bounding_box[1][3] = vertexs[i].y;
			}
			else
			{
				// right-up vertex
				predObj.bounding_box[0][2] = vertexs[i].x;
				predObj.bounding_box[1][2] = vertexs[i].y;
			}
		}
	}

	// for (int i = 0; i < 4; ++i)
	// {
	// 	printf("boundingbox point: (%f, %f)\n", predObj.bounding_box[0][i], predObj.bounding_box[1][i]);
	// }
	return predObj;
};

void MessageManager::pack_objectlist(const cc::ActorList &actors)
{
	_mutex.lock();
	//printf("start packing objectlist\n");
	OBJECTLIST.time_stamp = NAVINFO.timestamp;
	OBJECTLIST.data_source = 1;
	OBJECTLIST.object_count = 0;
	OBJECTLIST.predicted_object.clear();
	//printf("start packing every object\n");
	for (auto actor : actors)
	{
		if (actor->GetId() == vehState->GetId())
		{
			//printf("self, pass\n");
			continue;
		}
		// if (start_with(actor->GetTypeId(), "sensor") || start_with(actor->GetTypeId(), "controller"))
		// {
		// 	printf("sensor or controller actor, pass\n");
		// 	continue;
		// }
		// if (start_with(actor->GetTypeId(), "traffic") || start_with(actor->GetTypeId(), "static"))
		// {
		// 	printf("traffic or static actor, pass\n");
		// 	continue;
		// }
		if (!(start_with(actor->GetTypeId(), "vehicle") || start_with(actor->GetTypeId(), "walker")))
		{
			//printf("not a vehicle or walker, pass\n");
			continue;
		}
		if (!in_vehframe_test(*vehState.get(), actor))
		{
			//printf("not in vehframe, pass\n");
			continue;
		}

		//printf("start pack object: %s\n", actor->GetDisplayId().c_str());
		auto obj = pack_one_object(actor);
		OBJECTLIST.predicted_object.push_back(obj);
		++OBJECTLIST.object_count;
	}

	_mutex.unlock();
};

void MessageManager::pack_fusionmap_raster()
{
	_mutex.lock();

	MAP_HISTORY_CELLS = FUSIONMAP.map_cells;
	FUSIONMAP.map_cells.assign(MAP_ROW_NUM, MAP_ROW_0);

	FUSIONMAP.time_stamp = NAVINFO.timestamp;
	FUSIONMAP.car_utm_position_x = NAVINFO.utm_x;
	FUSIONMAP.car_utm_position_y = NAVINFO.utm_y;
	FUSIONMAP.car_heading = NAVINFO.angle_head;
	FUSIONMAP.map_resolution = FUSIONMAP_RESOLUTION;
	FUSIONMAP.map_row_num = MAP_ROW_NUM;
	FUSIONMAP.map_column_num = MAP_COLUMN_NUM;
	FUSIONMAP.car_center_column = MAP_CENTER_COLUMN;
	FUSIONMAP.car_center_row = MAP_CENTER_ROW;

	for (auto &obj : OBJECTLIST.predicted_object)
	{
		auto box = obj.bounding_box;
		// calculate rasterize zoom
		float left = box[1][0] > box[1][1] ? box[1][0] : box[1][1];
		float right = box[1][2] < box[1][3] ? box[1][2] : box[1][3];
		float bottom = box[0][1] < box[0][3] ? box[0][1] : box[0][3];
		float top = box[0][0] > box[0][2] ? box[0][0] : box[0][2];
		//printf("left: %f, right: %f, bottom: %f, top: %f\n", left, right, bottom, top);
		int16_t leftCell = -floor(left / FUSIONMAP.map_resolution) + FUSIONMAP.car_center_column;
		int16_t rightCell = -ceil(right / FUSIONMAP.map_resolution) + FUSIONMAP.car_center_column;
		int16_t bottomCell = FUSIONMAP.car_center_row - floor(bottom / FUSIONMAP.map_resolution);
		int16_t topCell = FUSIONMAP.car_center_row - ceil(top / FUSIONMAP.map_resolution);
		// front x, right y
		int16_t ymin = leftCell > -1 ? leftCell : 0;
		int16_t ymax = rightCell < FUSIONMAP.map_column_num ? rightCell : (FUSIONMAP.map_column_num - 1);
		int16_t xmin = topCell > -1 ? topCell : 0;
		int16_t xmax = bottomCell < FUSIONMAP.map_row_num ? bottomCell : (FUSIONMAP.map_row_num - 1);
		//printf("suggest rasterize zone: (%d, %d, %d, %d)\n", ymin, ymax, xmin, xmax);

		// rasterize obstacles into map cells
		int16_t x = xmin;
		int16_t y = ymin;

		int rastered_point_num = 0;

		while (x <= xmax)
		{
			y = ymin;
			while (y <= ymax)
			{
				double posY = -(y - FUSIONMAP.car_center_column) * FUSIONMAP.map_resolution;
				double posX = (FUSIONMAP.car_center_row - x) * FUSIONMAP.map_resolution;
				bool isOccupied = in_area_test(posX, posY, obj);
				if (isOccupied)
				{
					// bit-0 history obstacle, bit-1 lidar obstacle, bit-2 moving obstacle
					FUSIONMAP.map_cells[x][y] |= 0b00000010;
					++rastered_point_num;
					bool isHistory = true;
					bool isMoving = (fabs(obj.velocity) > 0.01);
					if (isHistory)
						FUSIONMAP.map_cells[x][y] |= 0b00000001;
					if (isMoving)
						FUSIONMAP.map_cells[x][y] |= 0b00000100;
				}
				++y;
			}
			++x;
		}
		//printf("rastered points count: %d\n", rastered_point_num);
	}
	_mutex.unlock();
};

void MessageManager::pack_fusionmap_lidar(const csd::LidarMeasurement &lidarMsg)
{
	_mutex.lock();

	MAP_HISTORY_CELLS = FUSIONMAP.map_cells;
	FUSIONMAP.map_cells.assign(MAP_ROW_NUM, MAP_ROW_0);

	FUSIONMAP.time_stamp = lidarMsg.GetTimestamp() * 1000;
	FUSIONMAP.car_utm_position_x = NAVINFO.utm_x;
	FUSIONMAP.car_utm_position_y = NAVINFO.utm_y;
	FUSIONMAP.car_heading = NAVINFO.angle_head;
	FUSIONMAP.map_resolution = FUSIONMAP_RESOLUTION;
	FUSIONMAP.map_row_num = MAP_ROW_NUM;
	FUSIONMAP.map_column_num = MAP_COLUMN_NUM;
	FUSIONMAP.car_center_column = MAP_CENTER_COLUMN;
	FUSIONMAP.car_center_row = MAP_CENTER_ROW;

	printf("frame %d\n", lidarMsg.GetFrame());
	printf("timestamp %4f\n", lidarMsg.GetTimestamp());
	printf("lidarMsg channel count: %d\n", lidarMsg.GetChannelCount());
	for (size_t i = 0; i < lidarMsg.GetChannelCount(); ++i)
	{
		printf("lidarMsg point count  : %d\n", lidarMsg.GetPointCount(i));
	}
	float intensity = lidarMsg.begin()->intensity;
	float x = lidarMsg.begin()->point.x;
	float y = lidarMsg.begin()->point.y;
	float z = lidarMsg.begin()->point.z;
	printf("first point :(%4f, %4f, %4f : %4f)\n", x, y, z, intensity);
	printf("total points num: %d\n", lidarMsg.size());

	for (size_t i = 0; i < lidarMsg.size(); ++i)
	{
		auto p = lidarMsg[i].point;
	}
	_mutex.unlock();
}
