#include "MyPlayer.hpp"

using carla::SharedPtr;
using std::string;
using std::vector;

namespace tievsim
{
    MyPlayer::MyPlayer(SharedPtr<cc::World> world) : world_(world){};

    SharedPtr<cc::Vehicle> MyPlayer::get_veh() { return veh_; }

    SharedPtr<cc::World> MyPlayer::get_world() { return world_; }

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
        driver_ = "tiev";
    }

    void MyPlayer::ControlTick()
    {
        ClientSideSensorTick();
        if (driver_ == "carla")
        {
            ;
        }
        else if (driver_ == "tiev")
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
        auto control = veh_->GetControl();
        aim_acc = msg_manager_->chassis_command_.;
        aim_steer = msg_manager_->chassis_command_.;
        }

} // namespace tievsim