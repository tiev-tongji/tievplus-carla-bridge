// Carla use Unreal-Engine coordinate system, left-hand
// X, Y, Z binded with East, South, Up
// rotation around Z (yaw): clockwise as positive
// rotation around X and Y (roll and pitch): counter-clockwise as positive

#include "MessageManager.hpp"

namespace tievsim
{
    using namespace utils;

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
        printf("[INFO] controller use %s.\n", filepath.c_str());
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
        kLanelinPointDistance = d["fusionmap"]["distance_laneline_points"].GetFloat();
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
        vertexs[0] = {bb.extent.x, bb.extent.y, 0};
        vertexs[1] = {-bb.extent.x, -bb.extent.y, 0};
        vertexs[2] = {bb.extent.x, -bb.extent.y, 0};
        vertexs[3] = {-bb.extent.x, bb.extent.y, 0};
        for (auto &vertex : vertexs)
        {
            trans_actor.TransformPoint(vertex);
            vertex = ToVehFrame(trans_ego, vertex);
        }
        std::list<size_t> right_to_left(4);
        std::list<size_t> bottom_to_up(4);
        for (size_t i = 0; i < 4; ++i)
        {
            if (right_to_left.empty())
            {
                right_to_left.push_back(i);
            }
            else if (vertexs[i].y > vertexs[right_to_left.back()].y)
            {
                right_to_left.push_back(i);
            }
            else
            {
                right_to_left.push_front(i);
            }

            if (bottom_to_up.empty())
            {
                bottom_to_up.push_back(i);
            }
            else if (vertexs[i].x > vertexs[bottom_to_up.back()].x)
            {
                bottom_to_up.push_back(i);
            }
            else
            {
                bottom_to_up.push_front(i);
            }
        }
        // left-up vertex
        pred_obj.bounding_box[0][0] = vertexs[right_to_left[3]].x;
        pred_obj.bounding_box[1][0] = vertexs[right_to_left[3]].y;
        if (vertexs[i].y >= pred_obj.trajectory_point[1][0])
        {
            if (vertexs[i].x <= pred_obj.trajectory_point[0][0])
            {
                // left-bottom vertex
                pred_obj.bounding_box[0][1] = vertexs[i].x;
                pred_obj.bounding_box[1][1] = vertexs[i].y;
            }
            else
            {
                // left-up vertex
                pred_obj.bounding_box[0][0] = vertexs[i].x;
                pred_obj.bounding_box[1][0] = vertexs[i].y;
            }
        }
        else
        {
            if (vertexs[i].x <= pred_obj.trajectory_point[0][0])
            {
                // right-bottom vertex
                pred_obj.bounding_box[0][3] = vertexs[i].x;
                pred_obj.bounding_box[1][3] = vertexs[i].y;
            }
            else
            {
                // right-up vertex
                pred_obj.bounding_box[0][2] = vertexs[i].x;
                pred_obj.bounding_box[1][2] = vertexs[i].y;
            }
        }
    }
    return pred_obj;
}; // namespace tievsim
} // namespace tievsim