#include "MessageManager.hpp"

namespace tievsim
{
    using namespace utils;

    MessageManager::MessageManager(const string &url)
        : MessageManagerBase(url){};

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

        // 速度，角速度，加速度
        auto vel = ego_car_->GetVelocity();
        auto vel_angle = ego_car_->GetAngularVelocity();
        auto acc = ego_car_->GetAcceleration();
        auto rot = ego_car_->GetTransform().rotation;
        caninfo_.velocity = mps2kph(norm2(vel.x, vel.y));
        caninfo_.yaw_rate = -deg2rad(vel_angle.z);
        caninfo_.acceleration_x = acc.x * cos(rot.yaw) + acc.y * sin(rot.yaw);
        caninfo_.acceleration_y = acc.x * sin(rot.yaw) - acc.y * cos(rot.yaw);

        // 当前控制量
        caninfo_.steer_wheel_angle = -ego_car_->GetControl().steer * 500;
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

        auto loc_rear = ego_car_->GetTransform().location;
        auto rot = ego_car_->GetTransform().rotation;
        auto loc_front = rel2abs(ego_car_->GetTransform(), {2.3, 0, 0}); // 轴距2.3

        // 位置
        GeographicLib::GeoCoords coord("121:12:44E 31:16:54N"); // 地图地理参考点，同济
        navinfo_.utm_x = coord.Easting() + loc_front.x;
        navinfo_.utm_y = coord.Northing() - loc_front.y;
        coord.Reset(coord.Zone(), coord.Northp(), navinfo_.utm_x, navinfo_.utm_y);
        navinfo_.latitude = coord.Latitude();
        navinfo_.longitude = coord.Longitude();
        navinfo_.altitude = gnss_msg.GetAltitude();

        // 旋转
        // Carla use Unreal-Engine coordinate system, left-hand
        // X, Y, Z binded with East, South, Up
        // rotation around Z (yaw): clockwise as positive
        // rotation around X and Y (roll and pitch): counter-clockwise as positive
        float heading = -rot.yaw;
        if (heading < -180)
        {
            heading = heading + 360;
        }
        else if (heading > 180)
        {
            heading = heading - 360;
        }
        navinfo_.angle_head = deg2rad(heading);
        navinfo_.angle_pitch = rot.pitch;
        navinfo_.angle_roll = rot.roll;
        auto rot_vel = ego_car_->GetAngularVelocity();
        navinfo_.angular_vel_z = deg2rad(-rot_vel.z);

        // 速度和加速度
        auto vel = ego_car_->GetVelocity();
        navinfo_.speed = norm2(vel.x, vel.y);
        navinfo_.velocity_east = vel.x;
        navinfo_.velocity_north = -vel.y; // East-South-Up coordinate in Carla.
        navinfo_.acceleration_x = caninfo_.acceleration_x;
        navinfo_.acceleration_y = caninfo_.acceleration_y;

        //状态
        navinfo_.curvature = 0;
        navinfo_.HPOS_accuracy = 0.01;
        navinfo_.RTK_status = 1;
        navinfo_.gps_num_satellites = 11;
        navinfo_.is_reckoning_vaild = 1;
    } // namespace tievsim
} // namespace tievsim