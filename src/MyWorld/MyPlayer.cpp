#include "MyPlayer.hpp"

using boost::static_pointer_cast;
using carla::SharedPtr;
using std::string;
using std::vector;

namespace tievsim
{
    MyPlayer::MyPlayer() : cnt_objectlist_(0), cnt_roadmarking_(0){};

    SharedPtr<cc::Vehicle> MyPlayer::get_veh() { return veh_; }

    void MyPlayer::SetAutoPilot()
    {
        veh_->SetAutopilot(true);
        driver_ = "carla";
    }

    void MyPlayer::SetManualControl()
    {
        veh_->SetAutopilot(false);
        driver_ = "user";
    }

    void MyPlayer::SetTievControl()
    {
        veh_->SetAutopilot(false);
        driver_ = "tievplus";
    }

    void MyPlayer::ControlTick()
    {
        if (driver_ == "carla")
        {
            ;
        }
        else if (driver_ == "tievplus")
        {
            TievControlTick();
        }
        else if (driver_ == "user")
        {
            ManualControlTick();
        }
        else if (driver_ == "none" || driver_ == "")
        {
            NoneControlTick();
        }
    }

    void MyPlayer::TievControlTick()
    {
        auto acc = veh_->GetAcceleration();
        auto trans = veh_->GetTransform();
        double acceleration_x = cg::Math::Dot(acc, trans.GetForwardVector());

        int64_t timestamp = msg_manager_->chassis_command_.timestamp;
        double aim_acc = msg_manager_->chassis_command_.longitudinal_acceleration_command;
        double aim_steer = msg_manager_->chassis_command_.steer_wheel_angle_command;
        uint8_t autonomous_mode = msg_manager_->chassis_command_.autonomous_mode_control_command;
        uint8_t gear = msg_manager_->chassis_command_.car_gear_command;

        double error_acc = aim_acc - acceleration_x;
        pid_controller_->Tick(error_acc, aim_steer);

        auto control = veh_->GetControl();
        control.throttle = pid_controller_->throttle;
        control.brake = pid_controller_->brake;
        control.hand_brake = (gear == 1);
        control.reverse = (gear == 2);
        control.manual_gear_shift = true;
        control.gear = 3;
        veh_->ApplyControl(control);
    }

    void MyPlayer::NoneControlTick()
    {
        auto control = veh_->GetControl();
        control.throttle = 0;
        control.brake = 0;
        control.hand_brake = true;
        veh_->ApplyControl(control);
    }

    void MyPlayer::ManualControlTick()
    {
        ; //TODO
    }

    void MyPlayer::ClientSideSensorTick(
        int frame_duration_objectlist, carla::SharedPtr<cc::ActorList> actor_list,
        int frame_duration_roadmarking, carla::SharedPtr<cc::Map> map)
    {
        ++cnt_objectlist_;
        ++cnt_roadmarking_;
        if (cnt_objectlist_ == frame_duration_objectlist)
        {
            msg_manager_->PackObjectlist(*actor_list);
            cnt_objectlist_ = 0;
        }
        if (cnt_roadmarking_ == frame_duration_roadmarking)
        {
            msg_manager_->PackRoadmarking(map);
            cnt_roadmarking_ = 0;
        }
    }

    void MyPlayer::Listen()
    {
        for (auto s : sensors_)
        {
            if (s.first == "gnss")
            {
                s.second->Listen(
                    [this](auto data) {
                        auto gnssMsg = static_pointer_cast<csd::GnssMeasurement>(data);
                        this->msg_manager_->PackNavinfo(*gnssMsg);
                    });
            }
            else if (s.first == "imu")
            {
                s.second->Listen(
                    [this](auto data) {
                        auto imuMsg = static_pointer_cast<csd::IMUMeasurement>(data);
                        this->msg_manager_->PackCaninfo(*imuMsg);
                    });
            }
            else if (s.first == "lidar")
            {
                s.second->Listen(
                    [this](auto data) {
                        auto lidarMsg = static_pointer_cast<csd::LidarMeasurement>(data);
                        this->msg_manager_->PackFusionmap(*lidarMsg);
                    });
            }
            else if (s.first == "camera.rgb")
            {
                ;
            }
            else if (s.first == "camera.depth")
            {
                ;
            }
        }
    }

    void MyPlayer::StopListen(const string &name)
    {
        if (name == "all")
        {
            for (auto s : sensors_)
            {
                s.second->Stop();
            }
            return;
        }
        for (auto s : sensors_)
        {
            if (s.first == name)
            {
                s.second->Stop();
                return;
            }
        }
        printf("[ERROR] invalid sensor name\n");
    }

    void MyPlayer::AddImu()
    {
        ;
    }

    void MyPlayer::AddGnss()
    {
        ;
    }
} // namespace tievsim