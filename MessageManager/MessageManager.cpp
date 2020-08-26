#include "MessageManager.hpp"

#ifdef USE_ZCM
void MessageManager::control_handler(const zcm::ReceiveBuffer *rbuf, const std::string &chan,
									 const MsgChassisCommandSignal *msg)
{
	std::lock_guard<std::mutex> lock(control_mutex, std::adopt_lock);
	chassis_command_.timestamp = msg->timestamp;
	chassis_command_.autonomous_mode_control_command = msg->autonomous_mode_control_command;
	chassis_command_.car_gear_command = msg->car_gear_command;
	chassis_command_.steer_wheel_angle_command = msg->steer_wheel_angle_command;
	chassis_command_.longitudinal_acceleration_command = msg->longitudinal_acceleration_command;
}

void MessageManager::subscribe_all()
{
	tunnel_.start();
}
#endif

#ifdef USE_LCM
void MessageManager::control_handler(const lcm::ReceiveBuffer *rbuf, const std::string &chan,
									 const MsgChassisCommandSignal *msg)
{
	std::lock_guard<std::mutex> lock(control_mutex, std::adopt_lock);
	chassis_command_.timestamp = msg->timestamp;
	chassis_command_.autonomous_mode_control_command = msg->autonomous_mode_control_command;
	chassis_command_.car_gear_command = msg->car_gear_command;
	chassis_command_.steer_wheel_angle_command = msg->steer_wheel_angle_command;
	chassis_command_.longitudinal_acceleration_command = msg->longitudinal_acceleration_command;
}

void MessageManager::sub_loop()
{
	while (!need_stop_)
	{
		TUNNEL.handle();
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

void MessageManager::publish_caninfo()
{
	tunnel_.publish("CANINFO", &caninfo_);
}

void MessageManager::publish_navinfo()
{
	tunnel_.publish("NAVINFO", &navinfo_);
}

void MessageManager::publish_fusionmap()
{
	tunnel_.publish("FUSIONMAP", &fusionmap_);
}

void MessageManager::publish_objectlist()
{
	tunnel_.publish("PREDICTEDOBJECT", &object_list_);
}

void MessageManager::publish_roadmarking()
{
	tunnel_.publish("ROADMARKINGLIST", &roadmarking_list_);
}

void MessageManager::publish_trafficlight()
{
	tunnel_.publish("TRAFFICLIGHTSIGNAL", &trafficlight_);
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
	while (!need_stop_)
	{
		auto time_point = std::chrono::steady_clock::now() + std::chrono::milliseconds(1000 / freq);
		tunnel_.publish("CANINFO", &caninfo_);
		//printf("async mode: publish CANINFO\n");
		std::this_thread::sleep_until(time_point);
	}
}

void MessageManager::pub_navinfo_loop(int freq)
{
	while (!need_stop_)
	{
		auto time_point = std::chrono::steady_clock::now() + std::chrono::milliseconds(1000 / freq);
		tunnel_.publish("NAVINFO", &navinfo_);
		//printf("async mode: publish NAVINFO\n");
		std::this_thread::sleep_until(time_point);
	}
}

void MessageManager::pub_fusionmap_loop(int freq)
{
	while (!need_stop_)
	{
		auto time_point = std::chrono::steady_clock::now() + std::chrono::milliseconds(1000 / freq);
		tunnel_.publish("FUSIONMAP", &fusionmap_);
		//print("async mode: publish FUSIONMAP");
		std::this_thread::sleep_until(time_point);
	}
}

void MessageManager::pub_objectlist_loop(int freq)
{
	while (!need_stop_)
	{
		auto time_point = std::chrono::steady_clock::now() + std::chrono::milliseconds(1000 / freq);
		tunnel_.publish("PREDICTEDOBJECT", &object_list_);
		//print("async mode: publish OBJECTLIST");
		std::this_thread::sleep_until(time_point);
	}
}

void MessageManager::pub_roadmarking_loop(int freq)
{
	while (!need_stop_)
	{
		auto time_point = std::chrono::steady_clock::now() + std::chrono::milliseconds(1000 / freq);
		tunnel_.publish("ROADMARKINGLIST", &roadmarking_list_);
		//print("async mode: publish ROADMARKINGLIST");
		std::this_thread::sleep_until(time_point);
	}
}

void MessageManager::pub_trafficlight_loop(int freq)
{
	while (!need_stop_)
	{
		auto time_point = std::chrono::steady_clock::now() + std::chrono::milliseconds(1000 / freq);
		tunnel_.publish("TRAFFICLIGHTSIGNAL", &trafficlight_);
		//print("async mode: publish TRAFFICLIGHTSIGNAL");
		std::this_thread::sleep_until(time_point);
	}
}

void MessageManager::publish_all_async(int freq_caninfo, int freq_navinfo, int freq_fusionmap,
									   int freq_objectlist, int freq_roadmarking, int freq_trafficlight)
{
	pub_threads_.push_back(std::thread(&MessageManager::pub_caninfo_loop, this, freq_caninfo));
	pub_threads_.push_back(std::thread(&MessageManager::pub_navinfo_loop, this, freq_navinfo));
	pub_threads_.push_back(std::thread(&MessageManager::pub_objectlist_loop, this, freq_objectlist));
	pub_threads_.push_back(std::thread(&MessageManager::pub_fusionmap_loop, this, freq_fusionmap));
	pub_threads_.push_back(std::thread(&MessageManager::pub_roadmarking_loop, this, freq_roadmarking));
	pub_threads_.push_back(std::thread(&MessageManager::pub_trafficlight_loop, this, freq_trafficlight));

	for (auto &t : pub_threads_)
	{
		t.detach();
	}
}

void MessageManager::pack_caninfo()
{
	std::lock_guard<std::mutex> caninfo_lock(caninfo_mutex, std::adopt_lock);
	caninfo_.timestamp = navinfo_.timestamp;

	caninfo_.drive_mode = 0;
	caninfo_.epb_mode = 0;
	caninfo_.eps_mode = 0;
	caninfo_.esp_mode = 0;
	caninfo_.gear_mode = 0;
	caninfo_.motor_mode = 0;
	caninfo_.eps_permission = 0;
	caninfo_.esp_permission = 0;
	caninfo_.epb_state = 0;

	if (vehState->GetControl().hand_brake)
		caninfo_.gear_state = 1;
	else if (vehState->GetControl().reverse)
		caninfo_.gear_state = 2;
	else
		caninfo_.gear_state = 4;

	auto vel = vehState->GetVelocity();
	caninfo_.velocity = mps2kph(norm2(vel.x, vel.y));
	caninfo_.wheel_speed_fl = 0;
	caninfo_.wheel_speed_fr = 0;
	caninfo_.wheel_speed_rl = 0;
	caninfo_.wheel_speed_rr = 0;

	caninfo_.yaw_rate = deg2rad(-vehState->GetAngularVelocity().z);

	auto acc = vehState->GetAcceleration();
	double yaw = deg2rad(vehState->GetTransform().rotation.yaw);
	caninfo_.acceleration_x = acc.x * cos(yaw) + acc.y * sin(yaw);
	caninfo_.acceleration_y = acc.x * sin(yaw) - acc.y * cos(yaw);
	//std::cout << "accX: " << CANINFO.acceleration_x << " || accY: " << CANINFO.acceleration_y << std::endl;

	caninfo_.steer_wheel_angle = -vehState->GetControl().steer * MAX_STEERINGWHEEL;
	caninfo_.steer_angular_speed = 0;

	caninfo_.motor_torque = 0;
	caninfo_.brake_deepness = vehState->GetControl().brake;
	caninfo_.accelerate_deepness = vehState->GetControl().throttle;
	caninfo_.brake_pedal_state = vehState->GetControl().brake == 0 ? 0 : 1;

	caninfo_.lamp_turn_l = 0;
	caninfo_.lamp_turn_r = 0;
	caninfo_.lamp_brake = 0;

	caninfo_.acceleration_x_desired = 0;
	caninfo_.steer_wheel_angle_desired = 0;
	caninfo_.emergency_control_state = 0;
}

void MessageManager::pack_navinfo(const csd::GnssMeasurement &gnssMsg)
{
	std::lock_guard<std::mutex> navinfo_lock(navinfo_mutex, std::adopt_lock);

	navinfo_.timestamp = gnssMsg.GetTimestamp() * 1000;

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

	navinfo_.latitude = gnssMsg.GetLatitude();
	navinfo_.longitude = gnssMsg.GetLongitude();
	navinfo_.altitude = gnssMsg.GetAltitude();
	coord.Reset(navinfo_.latitude, navinfo_.longitude);
	navinfo_.utm_x = coord.Easting();
	navinfo_.utm_y = coord.Northing();

	//printf("UE4 position: (%f, %f, %f, %f, %f, %f)\n", loc.x, loc.y, loc.z, rot.roll, rot.pitch, rot.yaw);
	//printf("GPS: (%f, %f)\n", navinfo_.latitude, navinfo_.longitude);
	//printf("error: (%f, %f)\n", navinfo_.utm_x - loc.x, navinfo_.utm_y + loc.y);

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
	navinfo_.angle_head = deg2rad(heading);
	navinfo_.angle_pitch = rot.pitch;
	navinfo_.angle_roll = rot.roll;
	auto rot_vel = vehState->GetAngularVelocity();
	navinfo_.angular_vel_z = deg2rad(-rot_vel.z);

	// speed, velocity, acceleration
	// TODO:rear axle to front axle
	auto vel = vehState->GetVelocity();
	navinfo_.speed = norm2(vel.x, vel.y);
	navinfo_.velocity_east = vel.x;
	navinfo_.velocity_north = -vel.y; // East-South-Up coordinate in Carla.
	navinfo_.acceleration_x = caninfo_.acceleration_x;
	navinfo_.acceleration_y = caninfo_.acceleration_y;

	//status
	navinfo_.curvature = 0;
	navinfo_.HPOS_accuracy = 0.01;
	navinfo_.RTK_status = 1;
	navinfo_.gps_num_satellites = 11;
	navinfo_.is_reckoning_vaild = 1;
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
	for (int i = 0; i < 4; ++i)
	{
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
	return predObj;
};

void MessageManager::pack_objectlist(const cc::ActorList &actors)
{
	std::lock_guard<std::mutex> objectlist_lock(objectlist_mutex, std::adopt_lock);
	object_list_.time_stamp = navinfo_.timestamp;
	object_list_.data_source = 1;
	object_list_.object_count = 0;
	object_list_.predicted_object.clear();
	for (auto actor : actors)
	{
		if (actor->GetId() == vehState->GetId())
		{
			continue;
		}
		if (!(start_with(actor->GetTypeId(), "vehicle") ||
			  start_with(actor->GetTypeId(), "walker") ||
			  start_with(actor->GetTypeId(), "static.prop")))
		{
			continue;
		}
		if (!in_vehframe_test(*vehState.get(), actor))
		{
			continue;
		}

		auto obj = pack_one_object(actor);
		object_list_.predicted_object.push_back(obj);
		++object_list_.object_count;
	}
};

void MessageManager::pack_fusionmap_raster()
{
	std::lock_guard<std::mutex> fusionmap_lock(fusionmap_mutex, std::adopt_lock);

	MAP_HISTORY_CELLS = fusionmap_.map_cells;
	fusionmap_.map_cells.assign(MAP_ROW_NUM, MAP_ROW_0);

	fusionmap_.time_stamp = navinfo_.timestamp;
	fusionmap_.car_utm_position_x = navinfo_.utm_x;
	fusionmap_.car_utm_position_y = navinfo_.utm_y;
	fusionmap_.car_heading = navinfo_.angle_head;
	fusionmap_.map_resolution = FUSIONMAP_RESOLUTION;
	fusionmap_.map_row_num = MAP_ROW_NUM;
	fusionmap_.map_column_num = MAP_COLUMN_NUM;
	fusionmap_.car_center_column = MAP_CENTER_COLUMN;
	fusionmap_.car_center_row = MAP_CENTER_ROW;

	for (auto &obj : object_list_.predicted_object)
	{
		auto box = obj.bounding_box;
		// calculate rasterize zoom
		float left = box[1][0] > box[1][1] ? box[1][0] : box[1][1];
		float right = box[1][2] < box[1][3] ? box[1][2] : box[1][3];
		float bottom = box[0][1] < box[0][3] ? box[0][1] : box[0][3];
		float top = box[0][0] > box[0][2] ? box[0][0] : box[0][2];
		int16_t leftCell = -ceil(left / fusionmap_.map_resolution) + fusionmap_.car_center_column;
		int16_t rightCell = -floor(right / fusionmap_.map_resolution) + fusionmap_.car_center_column;
		int16_t bottomCell = fusionmap_.car_center_row - floor(bottom / fusionmap_.map_resolution);
		int16_t topCell = fusionmap_.car_center_row - ceil(top / fusionmap_.map_resolution);
		// front x, right y
		int16_t ymin = leftCell > -1 ? leftCell : 0;
		int16_t ymax = rightCell < fusionmap_.map_column_num ? rightCell : (fusionmap_.map_column_num - 1);
		int16_t xmin = topCell > -1 ? topCell : 0;
		int16_t xmax = bottomCell < fusionmap_.map_row_num ? bottomCell : (fusionmap_.map_row_num - 1);

		// rasterize obstacles into map cells
		int16_t x = xmin;
		int16_t y = ymin;

		int rastered_point_num = 0;

		while (x <= xmax)
		{
			y = ymin;
			while (y <= ymax)
			{
				double posY = -(y - fusionmap_.car_center_column) * fusionmap_.map_resolution;
				double posX = (fusionmap_.car_center_row - x) * fusionmap_.map_resolution;
				bool isOccupied = in_area_test(posX, posY, obj);
				if (isOccupied)
				{
					// bit-0 history obstacle, bit-1 lidar obstacle, bit-2 moving obstacle
					fusionmap_.map_cells[x][y] |= 0b00000010;
					++rastered_point_num;
					bool isHistory = true;
					bool isMoving = (fabs(obj.velocity) > 100);
					if (isHistory)
						fusionmap_.map_cells[x][y] |= 0b00000001;
					if (isMoving)
						fusionmap_.map_cells[x][y] |= 0b00000100;
				}
				++y;
			}
			++x;
		}
		//printf("rastered points count: %d\n", rastered_point_num);
	}
};

void MessageManager::pack_fusionmap_lidar(const csd::LidarMeasurement &lidarMsg)
{
	std::lock_guard<std::mutex> fusionmap_lock(fusionmap_mutex, std::adopt_lock);

	//auto t1 = std::chrono::steady_clock::now();

	double duration = lidarMsg.GetTimestamp() * 1000 - fusionmap_.time_stamp;
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
		Eigen::Matrix3Xf points(3, pcdRawData.size());
		for (size_t i = 0; i < pcdRawData.size(); ++i)
		{
			points.col(i) = Eigen::Vector3f::Map(&pcdRawData[i][0], pcdRawData[i].size());
		}
		pcdRawData.clear();

		MAP_HISTORY_CELLS = fusionmap_.map_cells;
		fusionmap_.map_cells.assign(MAP_ROW_NUM, MAP_ROW_0);

		fusionmap_.time_stamp = lidarMsg.GetTimestamp() * 1000;
		fusionmap_.car_utm_position_x = navinfo_.utm_x;
		fusionmap_.car_utm_position_y = navinfo_.utm_y;
		fusionmap_.car_heading = navinfo_.angle_head;
		fusionmap_.map_resolution = FUSIONMAP_RESOLUTION;
		fusionmap_.map_row_num = MAP_ROW_NUM;
		fusionmap_.map_column_num = MAP_COLUMN_NUM;
		fusionmap_.car_center_column = MAP_CENTER_COLUMN;
		fusionmap_.car_center_row = MAP_CENTER_ROW;

		points *= 1 / FUSIONMAP_RESOLUTION;

		for (size_t i = 0; i < pcdRawData.size(); ++i)
		{
			;
		}
	}

	//auto t2 = std::chrono::steady_clock::now();
	//double dr_ms_pack_fusionmap = std::chrono::duration<double, std::milli>(t2 - t1).count();
	//std::cout << "time to pack lidar points: " << dr_ms_pack_fusionmap << std::endl;
}

void visRoadmarking(cc::DebugHelper &debugHelper)
{
	;
}

void MessageManager::pack_roadmarking(const SharedPtr<cc::Waypoint> current, cc::DebugHelper &debugHelper, bool enableDraw)
{
	std::lock_guard<std::mutex> roadmarking_lock(roadmarking_mutex, std::adopt_lock);

	auto t1 = std::chrono::steady_clock::now();

	typedef carla::road::element::LaneMarking::LaneChange LaneChange;
	typedef carla::road::element::LaneMarking::Type LaneLineType;

	if (current->IsJunction())
	{
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
	roadmarking_list_.current_lane_id = numRightLane;
	roadmarking_list_.num = numRightLane + numLeftLane + 1;

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
			lineL.line_type = lineL.kTypeDividing;
		}
		else if (markingL->type == LaneLineType::SolidSolid)
		{
			lineL.line_type = lineL.kTypeTypeNoPass;
		}
		else if (markingL->type == LaneLineType::SolidBroken ||
				 markingL->type == LaneLineType::BrokenSolid)
		{
			lineL.line_type = lineL.kTypeOneWayPass;
		}
		else if (markingL->type == LaneLineType::Broken ||
				 markingL->type == LaneLineType::BrokenBroken)
		{
			lineL.line_type = lineL.kTypeGuiding;
		}
		else
		{
			// BottsDots, Grass, Curb, Other, None
			lineL.line_type = lineL.kTypeTypeNoPass;
		}

		LaneLine lineR;
		lineR.distance = LINE_POINT_DISTANCE;
		auto markingR = wp->GetRightLaneMarking();
		if (wp->GetLaneId() * current->GetLaneId() < 0)
			markingR = wp->GetLeftLaneMarking();
		if (markingR->type == LaneLineType::Solid)
		{
			lineR.line_type = lineR.kTypeDividing;
		}
		else if (markingR->type == LaneLineType::SolidSolid)
		{
			lineR.line_type = lineR.kTypeTypeNoPass;
		}
		else if (markingR->type == LaneLineType::SolidBroken ||
				 markingR->type == LaneLineType::BrokenSolid)
		{
			lineR.line_type = lineR.kTypeOneWayPass;
		}
		else if (markingR->type == LaneLineType::Broken ||
				 markingR->type == LaneLineType::BrokenBroken)
		{
			lineR.line_type = lineR.kTypeGuiding;
		}
		else
		{
			// BottsDots, Grass, Curb, Other, None
			lineR.line_type = lineR.kTypeTypeNoPass;
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
						case LaneLine::kTypeDividing:
							colorL = red;
							break;
						case LaneLine::kTypeTypeNoPass:
							colorL = red;
							break;
						case LaneLine::kTypeGuiding:
							colorL = green;
							break;
						case LaneLine::kTypeOneWayPass:
							colorL = green;
							break;
						default:
							break;
						}
						switch (lineR.line_type)
						{
						case LaneLine::kTypeDividing:
							colorR = red;
							break;
						case LaneLine::kTypeTypeNoPass:
							colorR = red;
							break;
						case LaneLine::kTypeGuiding:
							colorR = green;
							break;
						case LaneLine::kTypeOneWayPass:
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
		roadmarking_list_.lanes.push_back(lane);
	}

	roadmarking_list_.stop_line.exist = 0;
	roadmarking_list_.stop_line.num = 1;
	roadmarking_list_.stop_line.stop_points.push_back(LinePoint{});
	roadmarking_list_.stop_line.distance = -1;

	roadmarking_list_.zebra.exist = 0;
	roadmarking_list_.zebra.num = 1;
	roadmarking_list_.zebra.zebra_points.push_back(LinePoint{});
	roadmarking_list_.zebra.distance = -1;

	roadmarking_list_.curb.exist = 0;
	roadmarking_list_.curb.num = 1;
	roadmarking_list_.curb.curb_points.push_back(LinePoint{});
	roadmarking_list_.curb.distance = -1;

	roadmarking_list_.no_parking.exist = 0;
	roadmarking_list_.no_parking.num = 1;
	roadmarking_list_.no_parking.no_parking_points.push_back(LinePoint{});
	roadmarking_list_.no_parking.distance = -1;

	roadmarking_list_.chevron.exist = 0;
	roadmarking_list_.chevron.num = 1;
	roadmarking_list_.chevron.chevron_points.push_back(LinePoint{});
	roadmarking_list_.chevron.distance = -1;

	auto t2 = std::chrono::steady_clock::now();
	double dr_ms_pack_roadmarking = std::chrono::duration<double, std::milli>(t2 - t1).count();
	//std::cout << "time to pack roadmarkinglist: " << dr_ms_pack_roadmarking << std::endl;
}

void MessageManager::pack_trafficlight()
{
	;
}