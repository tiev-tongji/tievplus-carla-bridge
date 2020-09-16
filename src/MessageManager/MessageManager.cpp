// Carla use Unreal-Engine coordinate system, left-hand
// X, Y, Z binded with East, South, Up
// rotation around Z (yaw): clockwise as positive
// rotation around X and Y (roll and pitch): counter-clockwise as positive

#include "MessageManager/MessageManager.hpp"

using carla::SharedPtr;
using std::list;
using std::string;
using std::vector;

namespace tievsim
{
    using namespace simutils;

    MessageManager::MessageManager(const string &url, const string &parameter_filepath)
        : MessageManagerBase(url) { PullParameter(parameter_filepath); }

    void MessageManager::PullParameter(const string &filepath)
    {
        FILE *fp = fopen(filepath.c_str(), "r");
        if (fp == 0)
        {
            printf("[Error] cannot open %s.\n", filepath.c_str());
            return;
        }
        printf("[INFO] message manager use %s.\n", filepath.c_str());
        char read_buffer[1024];
        rapidjson::FileReadStream is(fp, read_buffer, sizeof(read_buffer));
        rapidjson::Document d;
        d.ParseStream(is);

        kWheelbase = d["vehicle"]["wheelbase"].GetFloat();
        kMass = d["vehicle"]["mass"].GetFloat();
        kMassCenter.x = d["vehicle"]["mass_center"]["x"].GetFloat();
        kMassCenter.y = d["vehicle"]["mass_center"]["y"].GetFloat();
        kMassCenter.z = d["vehicle"]["mass_center"]["z"].GetFloat();
        kWheelRadius = d["vehicle"]["wheel_radius"].GetFloat();
        kMaxSteer = d["vehicle"]["max_steer_angle"].GetFloat();
        kMaxSteerWheel = d["vehicle"]["max_steerwheel_angle"].GetFloat();
        kPathPointNum = d["objectlist"]["num_path_points"].GetInt();
        kPathPointTimestep = d["objectlist"]["timestep"].GetInt();
        kMapResolution = d["fusionmap"]["resolution"].GetFloat();
        kMapRowNum = d["fusionmap"]["num_rows"].GetInt();
        kMapColNum = d["fusionmap"]["num_cols"].GetInt();
        kMapRowCenter = d["fusionmap"]["center_row"].GetInt();
        kMapColCenter = d["fusionmap"]["center_col"].GetInt();
        kLanelinePointNum = d["fusionmap"]["num_laneline_points"].GetInt();
        kLanelinePointDistance = d["fusionmap"]["distance_laneline_points"].GetFloat();
        fclose(fp);
    }

    void MessageManager::PackCaninfo(const csd::IMUMeasurement &imu_msg)
    {
        std::lock_guard<std::mutex> caninfo_lock(caninfo_mutex_, std::adopt_lock);

        caninfo_.timestamp = imu_msg.GetTimestamp() * 1000; // ms
        // 档位
        auto carla_control = ego_car_->GetControl();
        if (carla_control.hand_brake)
        {
            caninfo_.gear_state = 1; // 驻车档
        }
        else if (carla_control.reverse)
        {
            caninfo_.gear_state = 2; // 倒挡
        }
        else
        {
            caninfo_.gear_state = 4; // 前进挡
        }
        // 速度，角速度，加速度，后轴
        auto vel = ego_car_->GetVelocity();
        auto trans = ego_car_->GetTransform();
        float forward_vel = cg::Math::Dot(vel, trans.GetForwardVector()); // 在车头方向上的投影
        float right_vel = cg::Math::Dot(vel, trans.GetRightVector());     // 在车身右侧方向上的投影
        caninfo_.velocity = mps2kph(norm2(forward_vel, right_vel));
        caninfo_.velocity = forward_vel > 0 ? caninfo_.velocity : -caninfo_.velocity;
        auto rot_vel = ego_car_->GetAngularVelocity();
        caninfo_.yaw_rate = -deg2rad(rot_vel.z);
        auto acc = ego_car_->GetAcceleration();
        caninfo_.acceleration_x = cg::Math::Dot(acc, trans.GetForwardVector());
        caninfo_.acceleration_y = -cg::Math::Dot(acc, trans.GetRightVector());
        // 当前控制量
        caninfo_.steer_wheel_angle = -ego_car_->GetControl().steer * kMaxSteerWheel;
        caninfo_.brake_deepness = ego_car_->GetControl().brake;
        caninfo_.accelerate_deepness = ego_car_->GetControl().throttle;
        caninfo_.brake_pedal_state = ego_car_->GetControl().brake == 0 ? 0 : 1;
        // 暂未使用
        caninfo_.drive_mode = 0;
        caninfo_.epb_mode = 0;
        caninfo_.eps_mode = 0;
        caninfo_.esp_mode = 0;
        caninfo_.gear_mode = 0;
        caninfo_.motor_mode = 0;
        caninfo_.eps_permission = 0;
        caninfo_.esp_permission = 0;
        caninfo_.epb_state = 0;
        caninfo_.wheel_speed_fl = 0;
        caninfo_.wheel_speed_fr = 0;
        caninfo_.wheel_speed_rl = 0;
        caninfo_.wheel_speed_rr = 0;
        caninfo_.steer_angular_speed = 0;
        caninfo_.lamp_turn_l = 0;
        caninfo_.lamp_turn_r = 0;
        caninfo_.lamp_brake = 0;
        caninfo_.acceleration_x_desired = 0;
        caninfo_.steer_wheel_angle_desired = 0;
        caninfo_.emergency_control_state = 0;
        caninfo_.motor_torque = 0;
    }

    void MessageManager::PackNavinfo(const csd::GnssMeasurement &gnss_msg)
    {
        std::lock_guard<std::mutex> navinfo_lock(navinfo_mutex_, std::adopt_lock);

        navinfo_.timestamp = gnss_msg.GetTimestamp() * 1000;

        auto trans_rear = ego_car_->GetTransform();
        cg::Vector3D loc_front = {kWheelbase, 0.0, 0.0};
        trans_rear.TransformPoint(loc_front); // 变换到前轴

        // 位置
        GeographicLib::GeoCoords coord("121:12:44E 31:16:54N"); // 地图地理参考点，同济
        navinfo_.utm_x = coord.Easting() + loc_front.x;
        navinfo_.utm_y = coord.Northing() - loc_front.y;
        coord.Reset(coord.Zone(), coord.Northp(), navinfo_.utm_x, navinfo_.utm_y);
        navinfo_.latitude = coord.Latitude();
        navinfo_.longitude = coord.Longitude();
        navinfo_.altitude = gnss_msg.GetAltitude();

        // 旋转
        float heading = -trans_rear.rotation.yaw;
        if (heading < -180)
        {
            heading = heading + 360;
        }
        else if (heading > 180)
        {
            heading = heading - 360;
        }
        navinfo_.angle_head = deg2rad(heading);
        navinfo_.angle_pitch = trans_rear.rotation.pitch;
        navinfo_.angle_roll = trans_rear.rotation.roll;
        auto rot_vel = ego_car_->GetAngularVelocity();
        navinfo_.angular_vel_z = deg2rad(-rot_vel.z);

        // 速度和加速度
        auto vel = ego_car_->GetVelocity();
        float forward_vel = cg::Math::Dot(vel, trans_rear.GetForwardVector()); // 在车头方向上的投影
        float right_vel = cg::Math::Dot(vel, trans_rear.GetRightVector());     // 在车身右侧方向上的投影
        navinfo_.speed = norm2(forward_vel, right_vel);
        navinfo_.speed = forward_vel > 0 ? navinfo_.speed : -navinfo_.speed;
        navinfo_.velocity_east = vel.x;
        navinfo_.velocity_north = -vel.y; // East-South-Up coordinate in Carla.
        auto acc = ego_car_->GetAcceleration();
        navinfo_.acceleration_y = cg::Math::Dot(acc, trans_rear.GetForwardVector());
        navinfo_.acceleration_x = cg::Math::Dot(acc, trans_rear.GetRightVector());

        //状态
        navinfo_.curvature = 0;
        navinfo_.HPOS_accuracy = 0.01;
        navinfo_.RTK_status = 1;
        navinfo_.gps_num_satellites = 11;
        navinfo_.is_reckoning_vaild = 1;
    }

    PredictedObject MessageManager::PackOneObject(const SharedPtr<cc::Actor> actor)
    {
        PredictedObject pred_obj;
        pred_obj.id = actor->GetId();
        // 类别
        auto type_id = actor->GetTypeId();
        auto attrs = actor->GetAttributes();
        if (StartWith(type_id, "vehicle"))
        {
            string s_num_wheels = GetAttribute(attrs, "number_of_wheels");
            assert(s_num_wheels != "not found");
            if (std::stoi(s_num_wheels) >= 4)
            {
                pred_obj.type = 0; // 汽车
            }
            else
            {
                pred_obj.type = 1; // 自行车、摩托车
            }
        }
        else if (StartWith(type_id, "walker.pedestrian"))
        {
            pred_obj.type = 2; // 行人
        }
        else
        {
            pred_obj.type = 127; // for unknown type
        }

        // 速度和加速度
        auto trans_actor = actor->GetTransform();
        auto vel_actor = actor->GetVelocity();
        float vel_actor_forward = cg::Math::Dot(vel_actor, trans_actor.GetForwardVector());
        float vel_actor_right = cg::Math::Dot(vel_actor, trans_actor.GetRightVector());
        pred_obj.velocity = norm2(vel_actor_forward, vel_actor_right);
        pred_obj.velocity = vel_actor_forward > 0 ? pred_obj.velocity : -pred_obj.velocity;
        auto acc_actor = actor->GetAcceleration();
        float acc_actor_forward = cg::Math::Dot(acc_actor, trans_actor.GetForwardVector());
        float acc_actor_right = cg::Math::Dot(acc_actor, trans_actor.GetRightVector());
        pred_obj.accelerate = norm2(acc_actor_forward, acc_actor_right);
        pred_obj.accelerate = acc_actor_forward > 0 ? pred_obj.accelerate : -pred_obj.accelerate;

        // 航向角
        auto rot_actor = trans_actor.rotation;
        // float heading = 90 + rot_ego.yaw - rot_actor.yaw; // relative
        float heading = -rot_actor.yaw; // absolute
        if (heading < -180)
            heading = heading + 360;
        else if (heading > 180)
            heading = heading - 360;
        pred_obj.heading = deg2rad(heading);

        // 后推轨迹
        pred_obj.trajectory_point_num = kPathPointNum;
        pred_obj.trajectory_point.resize(2, std::vector<float>(kPathPointNum));
        auto loc_actor = trans_actor.location;
        auto trans_ego = ego_car_->GetTransform();
        for (int i = 0; i < kPathPointNum; ++i)
        {
            cg::Location pred_loc;
            double t = kPathPointTimestep;
            pred_loc.x = loc_actor.x + i * t * vel_actor.x + 0.5 * pow(i * t, 2) * acc_actor.x;
            pred_loc.y = loc_actor.y + i * t * vel_actor.y + 0.5 * pow(i * t, 2) * acc_actor.y;
            pred_loc.z = loc_actor.z + i * t * vel_actor.z + 0.5 * pow(i * t, 2) * acc_actor.z;
            cg::Location loc_actor_vehframe = ToVehFrame(trans_ego, loc_actor);
            pred_obj.trajectory_point[0][i] = loc_actor_vehframe.x;
            pred_obj.trajectory_point[1][i] = loc_actor_vehframe.y;
        }

        // 碰撞包围盒
        auto bb = actor->Serialize().bounding_box;
        pred_obj.length = actor->Serialize().bounding_box.extent.x * 2;
        pred_obj.width = actor->Serialize().bounding_box.extent.y * 2;
        vector<cg::Location> vertexs(4);
        vertexs[0] = {bb.location.x + bb.extent.x, bb.location.y + bb.extent.y, bb.location.z};
        vertexs[1] = {bb.location.x - bb.extent.x, bb.location.y - bb.extent.y, bb.location.z};
        vertexs[2] = {bb.location.x + bb.extent.x, bb.location.y - bb.extent.y, bb.location.z};
        vertexs[3] = {bb.location.x - bb.extent.x, bb.location.y + bb.extent.y, bb.location.z};
        for (auto &vertex : vertexs)
        {
            trans_actor.TransformPoint(vertex);
            vertex = ToVehFrame(trans_ego, vertex);
        }
        // 碰撞包围盒角点排序
        vector<size_t> ordered(4); // right-top, right-bottom, left-top, left-bottom
        for (size_t i = 0; i < 4; ++i)
        {
            if (ordered.empty())
            {
                ordered.push_back(i);
            }
            else if (vertexs[i].y > vertexs[ordered.back()].y)
            {
                ordered.push_back(i);
            }
            else
            {
                ordered.insert(ordered.begin(), i);
            }
        }
        if (vertexs[ordered[0]].x < vertexs[ordered[1]].x)
        {
            size_t temp = ordered[0];
            ordered[0] = ordered[1];
            ordered[1] = temp;
        }
        if (vertexs[ordered[2]].x < vertexs[ordered[3]].x)
        {
            size_t temp = ordered[2];
            ordered[2] = ordered[3];
            ordered[3] = temp;
        }
        // right-up vertex
        pred_obj.bounding_box[0][2] = vertexs[ordered[0]].x;
        pred_obj.bounding_box[1][2] = vertexs[ordered[0]].y;
        // right-bottom vertex
        pred_obj.bounding_box[0][3] = vertexs[ordered[1]].x;
        pred_obj.bounding_box[1][3] = vertexs[ordered[1]].y;
        // left-up vertex
        pred_obj.bounding_box[0][0] = vertexs[ordered[2]].x;
        pred_obj.bounding_box[1][0] = vertexs[ordered[2]].y;
        // left-bottom vertex
        pred_obj.bounding_box[0][1] = vertexs[ordered[3]].x;
        pred_obj.bounding_box[1][1] = vertexs[ordered[3]].y;
        return pred_obj;
    }

    void MessageManager::PackObjectlist(const cc::ActorList &actors)
    {
        std::lock_guard<std::mutex> objectlist_lock(objectlist_mutex_, std::adopt_lock);
        ResetObjectlist();
        for (auto actor : actors)
        {
            if (actor->GetId() == ego_car_->GetId())
            {
                continue;
            }
            if (!(StartWith(actor->GetTypeId(), "vehicle") ||
                  StartWith(actor->GetTypeId(), "walker") ||
                  StartWith(actor->GetTypeId(), "static.prop")))
            {
                continue;
            }

            auto rloc_actor = ToVehFrame(ego_car_->GetTransform(), actor->GetLocation());
            // 判断目标是否处于Map内，缩小3m范围
            bool in_vehframe = rloc_actor.x + 3 <= kMapRowCenter * kMapResolution &&
                               rloc_actor.x - 3 >= -(kMapRowNum - kMapRowCenter) * kMapResolution &&
                               rloc_actor.y + 3 <= kMapColCenter * kMapResolution &&
                               rloc_actor.y - 3 >= kMapColCenter * kMapResolution &&
                               fabs(rloc_actor.z) <= 3;
            if (!in_vehframe)
            {
                continue;
            }

            auto obj = PackOneObject(actor);
            object_list_.predicted_object.push_back(obj);
            ++object_list_.object_count;
        }
    }

    void MessageManager::RasterFusionmap()
    {
        for (auto &obj : object_list_.predicted_object)
        {
            auto box = obj.bounding_box;

            // calculate rasterize zoom
            float left = box[1][0] > box[1][1] ? box[1][0] : box[1][1];
            float right = box[1][2] < box[1][3] ? box[1][2] : box[1][3];
            float bottom = box[0][1] < box[0][3] ? box[0][1] : box[0][3];
            float top = box[0][0] > box[0][2] ? box[0][0] : box[0][2];
            int16_t left_cell = -ceil(left / fusionmap_.map_resolution) + fusionmap_.car_center_column;
            int16_t right_cell = -floor(right / fusionmap_.map_resolution) + fusionmap_.car_center_column;
            int16_t bottom_cell = fusionmap_.car_center_row - floor(bottom / fusionmap_.map_resolution);
            int16_t top_cell = fusionmap_.car_center_row - ceil(top / fusionmap_.map_resolution);

            // front x, right y
            int16_t ymin = left_cell > -1 ? left_cell : 0;
            int16_t ymax = right_cell < fusionmap_.map_column_num ? right_cell : (fusionmap_.map_column_num - 1);
            int16_t xmin = top_cell > -1 ? top_cell : 0;
            int16_t xmax = bottom_cell < fusionmap_.map_row_num ? bottom_cell : (fusionmap_.map_row_num - 1);

            // rasterize obstacles into map cells
            int16_t x = xmin;
            int16_t y = ymin;
            while (x <= xmax)
            {
                y = ymin;
                while (y <= ymax)
                {
                    double posx = -(y - fusionmap_.car_center_column) * fusionmap_.map_resolution;
                    double posy = (fusionmap_.car_center_row - x) * fusionmap_.map_resolution;
                    bool occupied = InAreaTest(posy, posx, obj);
                    if (occupied)
                    {
                        // bit-0 history obstacle, bit-1 lidar obstacle, bit-2 moving obstacle
                        fusionmap_.map_cells[x][y] |= 0b00000010;
                        bool isMoving = (fabs(obj.velocity) > 0.5);
                        if (isMoving)
                        {
                            fusionmap_.map_cells[x][y] |= 0b00000100;
                        }
                    }
                    ++y;
                }
                ++x;
            }
        }
    }

    void MessageManager::PackFusionmap(const csd::LidarMeasurement &lidar_msg)
    {
        std::lock_guard<std::mutex> fusionmap_lock(fusionmap_mutex_, std::adopt_lock);
        ResetFusionmap();
        RasterFusionmap();
    }

    list<SharedPtr<cc::Waypoint>> MessageManager::GetSlice(SharedPtr<cc::Waypoint> current,
                                                           int *left_lane_num, int *right_lane_num)
    {
        list<SharedPtr<cc::Waypoint>> slice;
        slice.push_back(current);

        // 往左遍历
        bool has_left = true;
        bool reverse_left = false;
        *left_lane_num = 0;
        auto current_left = current;
        while (has_left)
        {
            SharedPtr<cc::Waypoint> left;
            if (!reverse_left)
                left = current_left->GetLeft();
            else
                left = current_left->GetRight();

            if (!left.get())
            {
                has_left = false;
            }
            else if (!CheckLaneType(left->GetType()))
            {
                has_left = false;
            }
            else if (left->GetLaneId() * current->GetLaneId() < 0)
            {
                slice.push_back(left);
                *left_lane_num += 1;
                current_left = left;
                reverse_left = true;
            }
            else
            {
                slice.push_back(left);
                *left_lane_num += 1;
                current_left = left;
            }
        }

        // 往右遍历
        bool has_right = true;
        bool reverse_right = false;
        *right_lane_num = 0;
        auto current_right = current;
        while (has_right)
        {
            SharedPtr<cc::Waypoint> right;
            if (!reverse_right)
                right = current_right->GetRight();
            else
                right = current_right->GetLeft();

            if (!right.get())
            {
                has_right = false;
            }
            else if (!CheckLaneType(right->GetType()))
            {
                has_right = false;
            }
            else if (right->GetLaneId() * current->GetLaneId() < 0)
            {
                slice.push_front(right);
                *right_lane_num += 1;
                current_left = right;
                reverse_right = true;
            }
            else
            {
                slice.push_front(right);
                *right_lane_num += 1;
                current_right = right;
            }
        }
        return slice;
    }

    Lane MessageManager::PackLane(SharedPtr<cc::Waypoint> waypoint, SharedPtr<cc::Waypoint> current)
    {
        typedef carla::road::element::LaneMarking::LaneChange LaneChange;
        typedef carla::road::element::LaneMarking::Type LaneLineType;

        Lane lane;
        lane.width = waypoint->GetLaneWidth();
        lane.lane_type = Lane::kTypeNone; // TODO: carla暂不支持

        LaneLine line_left;
        line_left.distance = kLanelinePointDistance;
        auto marking_left = waypoint->GetLeftLaneMarking();
        if (waypoint->GetLaneId() * current->GetLaneId() < 0)
        {
            marking_left = waypoint->GetRightLaneMarking();
        }
        ParseLaneLineType(marking_left->type, &line_left);

        LaneLine line_right;
        line_right.distance = kLanelinePointDistance;
        auto marking_right = waypoint->GetRightLaneMarking();
        if (waypoint->GetLaneId() * current->GetLaneId() < 0)
        {
            marking_right = waypoint->GetLeftLaneMarking();
        }
        ParseLaneLineType(marking_right->type, &line_right);

        vector<SharedPtr<cc::Waypoint>> next_waypoints =
            waypoint->GetNextUntilLaneEnd(kLanelinePointDistance);
        vector<SharedPtr<cc::Waypoint>> prev_waypoints =
            waypoint->GetPreviousUntilLaneStart(kLanelinePointDistance);

        auto trans_ego = ego_car_->GetTransform();
        for (auto it = prev_waypoints.rbegin(); it != prev_waypoints.rend(); ++it)
        {
            auto wp = *it;
            double width = wp->GetLaneWidth();
            cg::Location loc_left{float(-width / 2), 0.0, 0.0};
            cg::Location loc_right{float(width / 2), 0.0, 0.0};
            auto trans = wp->GetTransform();
            trans.TransformPoint(loc_left);
            trans.TransformPoint(loc_right);
            auto rloc_left = ToVehFrame(trans_ego, loc_left);
            auto rloc_right = ToVehFrame(trans_ego, loc_right);
            LinePoint point_left;
            LinePoint point_right;
            point_left.x = rloc_left.x;
            point_left.y = rloc_left.y;
            point_right.x = rloc_right.x;
            point_right.y = rloc_right.y;
            lane.left_line.points.push_back(point_left);
            lane.left_line.points.push_back(point_right);
        }

        for (auto wp : next_waypoints)
        {
            double width = wp->GetLaneWidth();
            cg::Location loc_left{float(-width / 2), 0.0, 0.0};
            cg::Location loc_right{float(width / 2), 0.0, 0.0};
            auto trans = wp->GetTransform();
            trans.TransformPoint(loc_left);
            trans.TransformPoint(loc_right);
            auto rloc_left = ToVehFrame(trans_ego, loc_left);
            auto rloc_right = ToVehFrame(trans_ego, loc_right);
            LinePoint point_left;
            LinePoint point_right;
            point_left.x = rloc_left.x;
            point_left.y = rloc_left.y;
            point_right.x = rloc_right.x;
            point_right.y = rloc_right.y;
            lane.left_line.points.push_back(point_left);
            lane.left_line.points.push_back(point_right);
        }

        lane.left_line.num = lane.left_line.points.size();
        lane.right_line.num = lane.right_line.points.size();
        return lane;
    }

    void MessageManager::PackRoadmarking(SharedPtr<cc::Map> map)
    {
        std::lock_guard<std::mutex> roadmarking_lock(roadmarking_mutex_, std::adopt_lock);

        auto t1 = std::chrono::steady_clock::now();

        ResetRoadmarking();

        // 车道线
        auto current = map->GetWaypoint(ego_car_->GetLocation());
        if (current->IsJunction())
        {
            return;
        }

        int left_lane_num = 0;
        int right_lane_num = 0;
        auto slice = GetSlice(current, &left_lane_num, &right_lane_num);
        roadmarking_list_.current_lane_id = right_lane_num;
        roadmarking_list_.num = right_lane_num + left_lane_num + 1;
        for (auto wp : slice)
        {
            Lane lane = PackLane(wp, current);
            roadmarking_list_.lanes.push_back(lane);
        }

        auto t2 = std::chrono::steady_clock::now();
        double dr_ms_pack_roadmarking = std::chrono::duration<double, std::milli>(t2 - t1).count();
        //std::cout << "time to pack roadmarkinglist: " << dr_ms_pack_roadmarking << std::endl;
    }

    void MessageManager::PackTrafficlight()
    {
        ;
    }

    void MessageManager::ResetFusionmap()
    {
        fusionmap_.map_cells.assign(kMapRowNum, vector<uint8_t>(kMapColNum));
        fusionmap_.time_stamp = navinfo_.timestamp;
        fusionmap_.car_utm_position_x = navinfo_.utm_x;
        fusionmap_.car_utm_position_y = navinfo_.utm_y;
        fusionmap_.car_heading = navinfo_.angle_head;
        fusionmap_.map_resolution = kMapResolution;
        fusionmap_.map_row_num = kMapRowNum;
        fusionmap_.map_column_num = kMapColNum;
        fusionmap_.car_center_column = kMapColCenter;
        fusionmap_.car_center_row = kMapRowCenter;
    }

    void MessageManager::ResetRoadmarking()
    {
        roadmarking_list_.num = 0;
        roadmarking_list_.lanes.clear();
        roadmarking_list_.current_lane_id = 0;

        roadmarking_list_.stop_line.exist = 0;
        roadmarking_list_.stop_line.num = 0;
        roadmarking_list_.stop_line.stop_points.clear();
        roadmarking_list_.stop_line.distance = -1;

        roadmarking_list_.zebra.exist = 0;
        roadmarking_list_.zebra.num = 0;
        roadmarking_list_.zebra.zebra_points.clear();
        roadmarking_list_.zebra.distance = -1;

        roadmarking_list_.curb.exist = 0;
        roadmarking_list_.curb.num = 0;
        roadmarking_list_.curb.curb_points.clear();
        roadmarking_list_.curb.distance = -1;

        roadmarking_list_.no_parking.exist = 0;
        roadmarking_list_.no_parking.num = 0;
        roadmarking_list_.no_parking.no_parking_points.clear();
        roadmarking_list_.no_parking.distance = -1;

        roadmarking_list_.chevron.exist = 0;
        roadmarking_list_.chevron.num = 0;
        roadmarking_list_.chevron.chevron_points.clear();
        roadmarking_list_.chevron.distance = -1;
    }

    void MessageManager::ResetObjectlist()
    {
        object_list_.time_stamp = navinfo_.timestamp;
        object_list_.data_source = 1;
        object_list_.object_count = 0;
        object_list_.predicted_object.clear();
    }

} // namespace tievsim