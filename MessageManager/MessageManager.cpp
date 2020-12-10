#include "MessageManager.hpp"

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

void MessageManager::publish_roadmarking()
{
	TUNNEL.publish("ROADMARKING", &ROADMARKING);
}

void MessageManager::publish_trafficlight()
{
	TUNNEL.publish("TRAFFICLIGHTSIGNAL", &TRAFFICLIGHT);
}

void MessageManager::publish_all()
{
	publish_caninfo();
	publish_navinfo();
	publish_fusionmap();
	publish_objectlist();
	publish_roadmarking();
	publish_trafficlight();
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

void MessageManager::pub_roadmarking_loop(int freq)
{
	while (!_need_stop)
	{
		auto time_point = std::chrono::steady_clock::now() + std::chrono::milliseconds(1000 / freq);
		_mutex.lock();
		TUNNEL.publish("ROADMARKING", &ROADMARKING);
		//print("async mode: publish ROADMARKINGLIST");
		_mutex.unlock();
		std::this_thread::sleep_until(time_point);
	}
}

void MessageManager::pub_trafficlight_loop(int freq)
{
	while (!_need_stop)
	{
		auto time_point = std::chrono::steady_clock::now() + std::chrono::milliseconds(1000 / freq);
		_mutex.lock();
		TUNNEL.publish("TRAFFICLIGHTSIGNAL", &TRAFFICLIGHT);
		//print("async mode: publish TRAFFICLIGHTSIGNAL");
		_mutex.unlock();
		std::this_thread::sleep_until(time_point);
	}
}

void MessageManager::publish_all_async(int freq_caninfo, int freq_navinfo, int freq_fusionmap,
									   int freq_objectlist, int freq_roadmarking, int freq_trafficlight)
{
	_pub_threads.push_back(std::thread(&MessageManager::pub_caninfo_loop, this, freq_caninfo));
	_pub_threads.push_back(std::thread(&MessageManager::pub_navinfo_loop, this, freq_navinfo));
	_pub_threads.push_back(std::thread(&MessageManager::pub_objectlist_loop, this, freq_objectlist));
	_pub_threads.push_back(std::thread(&MessageManager::pub_fusionmap_loop, this, freq_fusionmap));
	_pub_threads.push_back(std::thread(&MessageManager::pub_roadmarking_loop, this, freq_roadmarking));
	_pub_threads.push_back(std::thread(&MessageManager::pub_trafficlight_loop, this, freq_trafficlight));

	for (auto &t : _pub_threads)
	{
		t.detach();
	}
}

void MessageManager::pack_caninfo()
{
	_mutex.lock();
	CANINFO.timestamp = NAVINFO.timestamp;

	CANINFO.drive_mode = 0;
	CANINFO.epb_mode = 0;
	CANINFO.eps_mode = 0;
	CANINFO.esp_mode = 0;
	CANINFO.gear_mode = 0;
	CANINFO.motor_mode = 0;
	CANINFO.eps_permission = 0;
	CANINFO.esp_permission = 0;
	CANINFO.epb_state = 0;

	if (vehState->GetControl().hand_brake)
		CANINFO.gear_state = 1;
	else if (vehState->GetControl().reverse)
		CANINFO.gear_state = 2;
	else
		CANINFO.gear_state = 4;

	auto vel = vehState->GetVelocity();
	CANINFO.velocity = mps2kph(norm2(vel.x, vel.y));
	CANINFO.wheel_speed_fl = 0;
	CANINFO.wheel_speed_fr = 0;
	CANINFO.wheel_speed_rl = 0;
	CANINFO.wheel_speed_rr = 0;

	CANINFO.yaw_rate = deg2rad(-vehState->GetAngularVelocity().z);

	auto acc = vehState->GetAcceleration();
	double yaw = deg2rad(vehState->GetTransform().rotation.yaw);
	CANINFO.acceleration_x = acc.x * cos(yaw) + acc.y * sin(yaw);
	CANINFO.acceleration_y = acc.x * sin(yaw) - acc.y * cos(yaw);
	//std::cout << "accX: " << CANINFO.acceleration_x << " || accY: " << CANINFO.acceleration_y << std::endl;

	CANINFO.steer_wheel_angle = -vehState->GetControl().steer * MAX_STEERINGWHEEL;
	CANINFO.steer_angular_speed = 0;

	CANINFO.motor_torque = 0;
	CANINFO.brake_deepness = vehState->GetControl().brake;
	CANINFO.accelerate_deepness = vehState->GetControl().throttle;
	CANINFO.brake_pedal_state = vehState->GetControl().brake == 0 ? 0 : 1;

	CANINFO.lamp_turn_l = 0;
	CANINFO.lamp_turn_r = 0;
	CANINFO.lamp_brake = 0;

	CANINFO.acceleration_x_desired = 0;
	CANINFO.steer_wheel_angle_desired = 0;
	CANINFO.emergency_control_state = 0;

	_mutex.unlock();
}

void MessageManager::pack_navinfo(const csd::GnssMeasurement &gnssMsg)
{
	_mutex.lock();

	NAVINFO.timestamp = gnssMsg.GetTimestamp() * 1000;

	auto loc = vehState->GetTransform().location;
	auto rot = vehState->GetTransform().rotation;

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
	// TiEV::LAT lat;
	// TiEV::LON lon;
	// lat.setByDegree(NAVINFO.latitude);
	// lon.setByDegree(NAVINFO.longitude);
	// TiEV::WGS84Coor gps(lat,lon);
	// auto utm = TiEV::latLonToUTMXY(gps);
	// NAVINFO.utm_x = utm.x;
	// NAVINFO.utm_y = utm.y;
	coord.Reset(NAVINFO.latitude, NAVINFO.longitude);
	NAVINFO.utm_x = coord.Easting();
	NAVINFO.utm_y = coord.Northing();

	printf("UE4 position: (%f, %f, %f, %f, %f, %f)\n", loc.x, loc.y, loc.z, rot.roll, rot.pitch, rot.yaw);
	printf("GPS: (%f, %f)\n", NAVINFO.latitude, NAVINFO.longitude);
	printf("UTM: (%f,%f)\n", NAVINFO.utm_x, NAVINFO.utm_y);
	printf("error: (%f, %f)\n", NAVINFO.utm_x - loc.x, NAVINFO.utm_y + loc.y);

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
	float heading = -rot.yaw; // absolute
	if (heading < -180)
		heading = heading + 360;
	else if (heading > 180)
		heading = heading - 360;
	predObj.heading = deg2rad(heading);

	predObj.trajectory_point_num = POINTS_NUM_OBJECTLIST_PREDICT;
	predObj.trajectory_point.resize(2, std::vector<float>(POINTS_NUM_OBJECTLIST_PREDICT));
	auto locEgo = vehState->GetTransform().location;
	locEgo.x = locEgo.x + 2.7 * cos(deg2rad(rotEgo.yaw));
	locEgo.y = locEgo.y + 2.7 * sin(deg2rad(rotEgo.yaw));
	auto loc = pActor->GetTransform().location;
	for (int i = 0; i < POINTS_NUM_OBJECTLIST_PREDICT; ++i)
	{
		cg::Location locPred;
		double t = TIMESTEP_OBJECTLIST_PREDICT;
		locPred.x = loc.x + i * t * vel.x + 0.5 * pow(i * t, 2) * pActor->GetAcceleration().x;
		locPred.y = loc.y + i * t * vel.y + 0.5 * pow(i * t, 2) * pActor->GetAcceleration().y;
		cg::Location locVehframe = unreal2vehframe(locEgo, locPred, rotEgo.yaw);
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
		vertex = unreal2vehframe(locEgo, vertex, rotEgo.yaw);
	}
	std::vector<size_t> indexs_l2r;
	for (int i = 0; i < 4; ++i)
	{
		if (indexs_l2r.empty())
		{
			indexs_l2r.push_back(i);
		}
		else if (vertexs[i].y < vertexs[indexs_l2r.back()].y)
		{
			indexs_l2r.push_back(i);
		}
		else
		{
			indexs_l2r.insert(indexs_l2r.begin(), i);
		}
		// if (vertexs[i].y >= predObj.trajectory_point[1][0])
		// {
		// 	if (vertexs[i].x <= predObj.trajectory_point[0][0])
		// 	{
		// 		// left-bottom vertex
		// 		predObj.bounding_box[0][1] = vertexs[i].x;
		// 		predObj.bounding_box[1][1] = vertexs[i].y;
		// 	}
		// 	else
		// 	{
		// 		// left-up vertex
		// 		predObj.bounding_box[0][0] = vertexs[i].x;
		// 		predObj.bounding_box[1][0] = vertexs[i].y;
		// 	}
		// }
		// else
		// {
		// 	if (vertexs[i].x <= predObj.trajectory_point[0][0])
		// 	{
		// 		// right-bottom vertex
		// 		predObj.bounding_box[0][3] = vertexs[i].x;
		// 		predObj.bounding_box[1][3] = vertexs[i].y;
		// 	}
		// 	else
		// 	{
		// 		// right-up vertex
		// 		predObj.bounding_box[0][2] = vertexs[i].x;
		// 		predObj.bounding_box[1][2] = vertexs[i].y;
		// 	}
		// }
	}
	if (vertexs[indexs_l2r[0]].x < vertexs[indexs_l2r[1]].x)
	{
		auto temp = indexs_l2r[0];
		indexs_l2r[0] = indexs_l2r[1];
		indexs_l2r[1] = temp;
	}
	if (vertexs[indexs_l2r[2]].x < vertexs[indexs_l2r[3]].x)
	{
		auto temp = indexs_l2r[2];
		indexs_l2r[2] = indexs_l2r[3];
		indexs_l2r[3] = temp;
	}
	printf("============================\n");
	for (size_t i = 0; i < 4; ++i)
	{
		predObj.bounding_box[0][i] = vertexs[indexs_l2r[i]].x;
		predObj.bounding_box[1][i] = vertexs[indexs_l2r[i]].y;
		printf("vertexs: (%f, %f)\n", predObj.bounding_box[0][i], predObj.bounding_box[1][i]);
	}
	return predObj;
};

void MessageManager::pack_objectlist(const cc::ActorList &actors)
{
	_mutex.lock();
	OBJECTLIST.time_stamp = NAVINFO.timestamp;
	OBJECTLIST.data_source = 1;
	OBJECTLIST.object_count = 0;
	OBJECTLIST.predicted_object.clear();
	for (auto actor : actors)
	{
		if (actor->GetId() == vehState->GetId())
		{
			continue;
		}
		// if (start_with(actor->GetTypeId(), "sensor") || start_with(actor->GetTypeId(), "controller"))
		// {
		// 	continue;
		// }
		// if (start_with(actor->GetTypeId(), "traffic") || start_with(actor->GetTypeId(), "static"))
		// {
		// 	continue;
		// }
		if (!(start_with(actor->GetTypeId(), "vehicle") || start_with(actor->GetTypeId(), "walker")))
		{
			continue;
		}
		if (!in_vehframe_test(*vehState.get(), actor))
		{
			continue;
		}

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
		int16_t leftCell = -ceil(left / FUSIONMAP.map_resolution) + FUSIONMAP.car_center_column;
		int16_t rightCell = -floor(right / FUSIONMAP.map_resolution) + FUSIONMAP.car_center_column;
		int16_t bottomCell = FUSIONMAP.car_center_row - floor(bottom / FUSIONMAP.map_resolution);
		int16_t topCell = FUSIONMAP.car_center_row - ceil(top / FUSIONMAP.map_resolution);
		// front x, right y
		int16_t ymin = leftCell > -1 ? leftCell : 0;
		int16_t ymax = rightCell < FUSIONMAP.map_column_num ? rightCell : (FUSIONMAP.map_column_num - 1);
		int16_t xmin = topCell > -1 ? topCell : 0;
		int16_t xmax = bottomCell < FUSIONMAP.map_row_num ? bottomCell : (FUSIONMAP.map_row_num - 1);

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
					bool isMoving = (fabs(obj.velocity) > 0.1);
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

	//auto t1 = std::chrono::steady_clock::now();

	double duration = lidarMsg.GetTimestamp() * 1000 - FUSIONMAP.time_stamp;
	if (duration < 1000 / LIDAR_ROTATE_FREQUENCY)
	{

		for (size_t i = 0; i < lidarMsg.size(); ++i)
		{
			vector<float> vec{lidarMsg[i].point.x, lidarMsg[i].point.y, lidarMsg[i].point.z};
			pcdRawData.push_back(vec);
		}
		// std::cout << "-----------------stacked " << lidarMsg.size() << " points" << std::endl;
		// printf("frame %d\n", lidarMsg.GetFrame());
		// printf("timestamp %4f\n", lidarMsg.GetTimestamp() * 1000);
		// printf("lidarMsg channel count: %d\n", lidarMsg.GetChannelCount());
		// printf("total points num: %d\n", lidarMsg.size());
	}
	else
	{
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

		// auto egoT = vehState->GetTransform();
		// auto lidarT = lidarState->GetTransform();
		// printf("ego car transform: (%f,%f,%f,%f)\n", egoT.location.x, egoT.location.y, egoT.location.z, egoT.rotation.yaw);
		// printf("lidar transform: (%f,%f,%f,%f)\n", lidarT.location.x, lidarT.location.y, lidarT.location.z, lidarT.rotation.yaw);
		for (size_t i = 0; i < pcdRawData.size(); ++i)
		{
			float x = pcdRawData[i][0];
			float y = pcdRawData[i][1];
			float z = pcdRawData[i][2];

			//printf("lidar points %d: (%f,%f,%f)\n", i, x, y, z);
		}
		pcdRawData.clear();
	}

	//auto t2 = std::chrono::steady_clock::now();
	//double dr_ms_pack_fusionmap = std::chrono::duration<double, std::milli>(t2 - t1).count();
	//std::cout << "time to pack lidar points: " << dr_ms_pack_fusionmap << std::endl;

	_mutex.unlock();
}

void visRoadmarking(cc::DebugHelper &debugHelper)
{
	;
}

// FIXME: this method is a lit bit resource-consuming, now tested to run at about 100Hz.
void MessageManager::pack_roadmarking(const SharedPtr<cc::Waypoint> current, cc::DebugHelper &debugHelper, bool enableDraw)
{
	pack_trafficlight();

	_mutex.lock();

	auto t1 = std::chrono::steady_clock::now();

	typedef carla::road::element::LaneMarking::LaneChange LaneChange;
	typedef carla::road::element::LaneMarking::Type LaneLineType;

	ROADMARKING.lanes.clear();
	ROADMARKING.num = 0;

	if (current->IsJunction())
	{
		_mutex.unlock();
		return;
	}

	//std::cout << "-----------------------------------------------------------" << std::endl;
	list<SharedPtr<cc::Waypoint>> slice;
	slice.push_back(current);
	//std::cout << "lane id: " << current->GetLaneId() << " lane type: " << laneType2string(current->GetType()) << std::endl;
	//std::cout << ">>>linetype left: " << lineType2string(current->GetLeftLaneMarking()->type)
	//		  << " right: " << lineType2string(current->GetRightLaneMarking()->type) << std::endl;
	//std::cout << "++++++++++++++++++++++++++++++++" << std::endl;
	bool hasLeft = true;
	bool reverseL = false;
	int numLeftLane = 0;
	auto currentL = current;
	while (hasLeft)
	{
		SharedPtr<cc::Waypoint> left;
		if (!reverseL)
			left = currentL->GetLeft();
		else
			left = currentL->GetRight();

		if (!left.get())
		{
			//std::cout << "no left neigher\n";
			hasLeft = false;
		}
		else if (!checkLaneType(left->GetType()))
		{
			//std::cout << "invalid lanetype\n";
			hasLeft = false;
		}
		else if (left->GetLaneId() * current->GetLaneId() < 0)
		{
			slice.push_back(left);
			++numLeftLane;
			currentL = left;
			reverseL = true;
			//std::cout << "lane id: " << left->GetLaneId() << " lane type: " << laneType2string(left->GetType()) << std::endl;
			//std::cout << ">>>linetype left: " << lineType2string(left->GetRightLaneMarking()->type)
			//		  << " right: " << lineType2string(left->GetLeftLaneMarking()->type) << std::endl;
		}
		else
		{
			slice.push_back(left);
			++numLeftLane;
			currentL = left;
			//std::cout << "lane id: " << left->GetLaneId() << " lane type: " << laneType2string(left->GetType()) << std::endl;
			//std::cout << ">>>linetype left: " << lineType2string(left->GetLeftLaneMarking()->type)
			//		  << " right: " << lineType2string(left->GetRightLaneMarking()->type) << std::endl;
		}
	}
	//std::cout << "==================================" << std::endl;
	bool hasRight = true;
	bool reverseR = false;
	int numRightLane = 0;
	auto currentR = current;
	while (hasRight)
	{
		SharedPtr<cc::Waypoint> right;
		if (!reverseR)
			right = currentR->GetRight();
		else
			right = currentR->GetLeft();

		if (!right.get())
		{
			//std::cout << "no right neigher\n";
			hasRight = false;
		}
		else if (!checkLaneType(right->GetType()))
		{
			//std::cout << "invalid lanetype\n";
			hasRight = false;
		}
		else if (right->GetLaneId() * current->GetLaneId() < 0)
		{
			slice.push_front(right);
			++numLeftLane;
			currentL = right;
			reverseR = true;
			//std::cout << "lane id: " << right->GetLaneId() << " lane type: " << laneType2string(right->GetType()) << std::endl;
			//std::cout << ">>>linetype left: " << lineType2string(right->GetRightLaneMarking()->type)
			//		  << " right: " << lineType2string(right->GetLeftLaneMarking()->type) << std::endl;
		}
		else
		{
			slice.push_front(right);
			++numRightLane;
			currentR = right;
			//std::cout << "lane id: " << right->GetLaneId() << " lane type: " << laneType2string(right->GetType()) << std::endl;
			//std::cout << ">>>linetype left: " << lineType2string(right->GetLeftLaneMarking()->type)
			//		  << " right: " << lineType2string(right->GetRightLaneMarking()->type) << std::endl;
		}
	}
	ROADMARKING.current_lane_id = numRightLane;
	ROADMARKING.num = numRightLane + numLeftLane + 1;

	for (auto wp : slice)
	{
		Lane lane;
		lane.width = wp->GetLaneWidth();
		// FIXME: wrong!
		switch (wp->GetLaneChange())
		{
		case LaneChange::None:
			lane.lane_type = lane.kTypeStraight;
			break;
		case LaneChange::Both:
			lane.lane_type = lane.kTypeLeftRight;
			break;
		case LaneChange::Left:
			lane.lane_type = lane.kTypeStraightLeft;
			break;
		case LaneChange::Right:
			lane.lane_type = lane.kTypeStraightRight;
			break;
		default:
			lane.lane_type = lane.kTypeNone;
			break;
		}

		LaneLine lineL;
		lineL.distance = LINE_POINT_DISTANCE;
		auto markingL = wp->GetLeftLaneMarking();
		if (wp->GetLaneId() * current->GetLaneId() < 0)
			markingL = wp->GetRightLaneMarking();
		if (markingL->type == LaneLineType::Solid)
		{
			lineL.line_type = lineL.kTypeSolid;
		}
		else if (markingL->type == LaneLineType::SolidSolid)
		{
			lineL.line_type = lineL.kTypeSolid;
		}
		else if (markingL->type == LaneLineType::SolidBroken)
		{
			lineL.line_type = lineL.kTypeSolid;
		}
		else if (markingL->type == LaneLineType::BrokenSolid)
		{
			lineL.line_type = lineL.kTypeDashed;
		}
		else if (markingL->type == LaneLineType::Broken ||
				 markingL->type == LaneLineType::BrokenBroken)
		{
			lineL.line_type = lineL.kTypeDashed;
		}
		else
		{
			// BottsDots, Grass, Curb, Other, None
			lineL.line_type = lineL.kTypeSolidYellow;
		}

		LaneLine lineR;
		lineR.distance = LINE_POINT_DISTANCE;
		auto markingR = wp->GetRightLaneMarking();
		if (wp->GetLaneId() * current->GetLaneId() < 0)
			markingR = wp->GetLeftLaneMarking();
		if (markingR->type == LaneLineType::Solid)
		{
			lineR.line_type = lineR.kTypeSolid;
		}
		else if (markingR->type == LaneLineType::SolidSolid)
		{
			lineR.line_type = lineR.kTypeSolid;
		}
		else if (markingR->type == LaneLineType::SolidBroken)
		{
			lineR.line_type = lineR.kTypeSolid;
		}
		else if (markingR->type == LaneLineType::BrokenSolid)
		{
			lineR.line_type = lineR.kTypeDashed;
		}
		else if (markingR->type == LaneLineType::Broken ||
				 markingR->type == LaneLineType::BrokenBroken)
		{
			lineR.line_type = lineR.kTypeDashed;
		}
		else
		{
			// BottsDots, Grass, Curb, Other, None
			lineR.line_type = lineR.kTypeSolidYellow;
		}

		vector<SharedPtr<cc::Waypoint>> waypoints;
		cg::Location lastR;
		cg::Location lastL;
		for (size_t i = 0; i < LINE_POINTS_NUM + 1; ++i)
		{
			auto nexts = wp->GetNext(LINE_POINT_DISTANCE * (i + 1));
			if (wp->GetLaneId() * current->GetLaneId() < 0)
			{
				nexts = wp->GetPrevious(LINE_POINT_DISTANCE * (i + 1));
			}

			if (!nexts.empty())
			{
				if (nexts[0]->IsJunction())
				{
					break;
				}

				waypoints.push_back(nexts[0]);
				double width = waypoints[i]->GetLaneWidth();
				float yaw = deg2rad(waypoints[i]->GetTransform().rotation.yaw);
				if (wp->GetLaneId() * current->GetLaneId() < 0)
					yaw = deg2rad(waypoints[i]->GetTransform().rotation.yaw - 180);
				auto loc = waypoints[i]->GetTransform().location;
				float xl = loc.x + width / 2 * sin(yaw);
				float yl = loc.y - width / 2 * cos(yaw);
				float xr = loc.x - width / 2 * sin(yaw);
				float yr = loc.y + width / 2 * cos(yaw);
				if (enableDraw)
				{
					if (i > 0)
					{
						cc::DebugHelper::Color green{20, 200, 90};
						cc::DebugHelper::Color red{240, 30, 40};
						cc::DebugHelper::Color colorL;
						cc::DebugHelper::Color colorR;
						switch (lineL.line_type)
						{
						case LaneLine::kTypeSolid:
							colorL = red;
							break;
						case LaneLine::kTypeSolidYellow:
							colorL = red;
							break;
						case LaneLine::kTypeDashed:
							colorL = green;
							break;
						default:
							break;
						}
						switch (lineR.line_type)
						{
						case LaneLine::kTypeSolid:
							colorR = red;
							break;
						case LaneLine::kTypeSolidYellow:
							colorR = red;
							break;
						case LaneLine::kTypeDashed:
							colorR = green;
							break;
						default:
							break;
						}
						debugHelper.DrawLine(lastL, cg::Location{xl, yl, loc.z}, 0.1, colorL, 0.05, false);
						debugHelper.DrawLine(lastR, cg::Location{xr, yr, loc.z}, 0.1, colorR, 0.05, false);
					}
				}
				lastR.x = xr;
				lastR.y = yr;
				lastR.z = loc.z;
				lastL.x = xl;
				lastL.y = yl;
				lastL.z = loc.z;
				cg::Location resl = unreal2vehframe(vehState->GetLocation(), cg::Location(xl, yl, 0), vehState->GetTransform().rotation.yaw);
				LinePoint pl;
				pl.x = resl.x;
				pl.y = resl.y;
				lineL.points.push_back(pl);
				cg::Location resr = unreal2vehframe(vehState->GetLocation(), cg::Location(xr, yr, 0), vehState->GetTransform().rotation.yaw);
				LinePoint pr;
				pr.x = resr.x;
				pr.y = resr.y;
				lineR.points.push_back(pr);
			}
		}
		lineL.num = waypoints.size();
		lineR.num = waypoints.size();
		lane.left_line = lineL;
		lane.right_line = lineR;
		ROADMARKING.lanes.push_back(lane);
	}

	ROADMARKING.stop_line.exist = 0;
	ROADMARKING.stop_line.num = 1;
	ROADMARKING.stop_line.stop_points.push_back(LinePoint{});
	ROADMARKING.stop_line.distance = -1;

	auto t2 = std::chrono::steady_clock::now();
	double dr_ms_pack_roadmarking = std::chrono::duration<double, std::milli>(t2 - t1).count();
	//std::cout << "time to pack roadmarkinglist: " << dr_ms_pack_roadmarking << std::endl;

	_mutex.unlock();
}

void MessageManager::pack_trafficlight()
{
	TRAFFICLIGHT.left = 1;
	TRAFFICLIGHT.right = 1;
	TRAFFICLIGHT.forward = 1;
}