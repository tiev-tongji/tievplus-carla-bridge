#include "MessageManagerBase.hpp"

using namespace std::chrono_literals;
using std::string;
using std::vector;

namespace tievsim
{

    MessageManagerBase::MessageManagerBase(const string &url)
        : tunnel_(url), need_stop_(false)
    {
        chassis_command_.longitudinal_acceleration_command = 0.0;
        chassis_command_.steer_wheel_angle_command = 0.0;
        chassis_command_.timestamp = 0.0;
        chassis_command_.car_gear_command = 1;                // 驻车档
        chassis_command_.autonomous_mode_control_command = 2; // 非自动驾驶模式
    };

    MessageManagerBase::~MessageManagerBase()
    {
        need_stop_ = true;
        for (auto &t : pub_threads_)
        {
            if (t.joinable())
                t.join();
        }
#ifdef USE_ZCM
        tunnel_.stop();
        for (auto t : sub_threads_)
        {
            tunnel_.unsubscribe(t);
        }
#endif
#ifdef USE_LCM
        for (auto &t : sub_threads_)
        {
            if (t.joinable())
                t.join();
        }
#endif
    }

#ifdef USE_ZCM
    void MessageManagerBase::ChassisCommandHandler(const zcm::ReceiveBuffer *rbuf, const std::string &chan,
                                                   const MsgChassisCommandSignal *msg)
    {
        std::lock_guard<std::mutex> lock(chassis_command_mutex_, std::adopt_lock);
        chassis_command_.timestamp = msg->timestamp;
        chassis_command_.autonomous_mode_control_command = msg->autonomous_mode_control_command;
        chassis_command_.car_gear_command = msg->car_gear_command;
        chassis_command_.steer_wheel_angle_command = msg->steer_wheel_angle_command;
        chassis_command_.longitudinal_acceleration_command = msg->longitudinal_acceleration_command;
    }

    void MessageManagerBase::SubscribeAll()
    {
        tunnel_.start();
    }
#endif

#ifdef USE_LCM
    void MessageManagerBase::ChassisCommandHandler(const lcm::ReceiveBuffer *rbuf, const std::string &chan,
                                                   const MsgChassisCommandSignal *msg)
    {
        std::lock_guard<std::mutex> lock(chassis_command_mutex_, std::adopt_lock);
        chassis_command_.timestamp = msg->timestamp;
        chassis_command_.autonomous_mode_control_command = msg->autonomous_mode_control_command;
        chassis_command_.car_gear_command = msg->car_gear_command;
        chassis_command_.steer_wheel_angle_command = msg->steer_wheel_angle_command;
        chassis_command_.longitudinal_acceleration_command = msg->longitudinal_acceleration_command;
    }

    void MessageManagerBase::SubLoop()
    {
        while (!need_stop_)
        {
            TUNNEL.handle();
        }
    }

    void MessageManagerBase::SubscribeAll()
    {
        sub_threads_.push_back(std::thread(&MessageManagerBase::SubLoop, this));
        for (auto &t : sub_threads_)
        {
            t.detach();
        }
    }
#endif

    void MessageManagerBase::PublishCaninfo() const
    {
        std::lock_guard<std::mutex> caninfo_lock(caninfo_mutex_, std::adopt_lock);
        tunnel_.publish("CANINFO", &caninfo_);
    }

    void MessageManagerBase::PublishNavinfo() const
    {
        std::lock_guard<std::mutex> navinfo_lock(navinfo_mutex_, std::adopt_lock);
        tunnel_.publish("NAVINFO", &navinfo_);
    }

    void MessageManagerBase::PublishFusionmap() const
    {
        std::lock_guard<std::mutex> fusionmap_lock(fusionmap_mutex_, std::adopt_lock);
        tunnel_.publish("FUSIONMAP", &fusionmap_);
    }

    void MessageManagerBase::PublishObjectlist() const
    {
        std::lock_guard<std::mutex> objectlist_lock(objectlist_mutex_, std::adopt_lock);
        tunnel_.publish("PREDICTEDOBJECT", &object_list_);
    }

    void MessageManagerBase::PublishRoadmarking() const
    {
        std::lock_guard<std::mutex> roadmarking_lock(roadmarking_mutex_, std::adopt_lock);
        tunnel_.publish("ROADMARKINGLIST", &roadmarking_list_);
    }

    void MessageManagerBase::PublishTrafficlight() const
    {
        std::lock_guard<std::mutex> trafficlight_lock(trafficlight_mutex_, std::adopt_lock);
        tunnel_.publish("TRAFFICLIGHTSIGNAL", &trafficlight_);
    }

    void MessageManagerBase::PublishAll() const
    {
        PublishCaninfo();
        PublishNavinfo();
        PublishFusionmap();
        PublishObjectlist();
        PublishRoadmarking();
        PublishTrafficlight();
    }

    void MessageManagerBase::PubLoopCaninfo(int freq)
    {
        while (!need_stop_)
        {
            auto time_point = std::chrono::steady_clock::now() + std::chrono::milliseconds(1000 / freq);
            PublishCaninfo();
            std::this_thread::sleep_until(time_point);
        }
    }

    void MessageManagerBase::PubLoopNavinfo(int freq)
    {
        auto time_point = std::chrono::steady_clock::now() + std::chrono::milliseconds(1000 / freq);
        PublishNavinfo();
        std::this_thread::sleep_until(time_point);
    }

    void MessageManagerBase::PubLoopFusionmap(int freq)
    {
        auto time_point = std::chrono::steady_clock::now() + std::chrono::milliseconds(1000 / freq);
        PublishFusionmap();
        std::this_thread::sleep_until(time_point);
    }

    void MessageManagerBase::PubLoopObjectlist(int freq)
    {
        auto time_point = std::chrono::steady_clock::now() + std::chrono::milliseconds(1000 / freq);
        PublishObjectlist();
        std::this_thread::sleep_until(time_point);
    }

    void MessageManagerBase::PubLoopRoadmarking(int freq)
    {
        auto time_point = std::chrono::steady_clock::now() + std::chrono::milliseconds(1000 / freq);
        PublishRoadmarking();
        std::this_thread::sleep_until(time_point);
    }

    void MessageManagerBase::PubLoopTrafficlight(int freq)
    {
        auto time_point = std::chrono::steady_clock::now() + std::chrono::milliseconds(1000 / freq);
        PublishTrafficlight();
        std::this_thread::sleep_until(time_point);
    }

    void MessageManagerBase::PublishAllAsync(int freq_caninfo, int freq_navinfo, int freq_fusionmap,
                                             int freq_objectlist, int freq_roadmarking, int freq_trafficlight)
    {
        pub_threads_.push_back(std::thread(&MessageManagerBase::PubLoopCaninfo, this, freq_caninfo));
        pub_threads_.push_back(std::thread(&MessageManagerBase::PubLoopNavinfo, this, freq_navinfo));
        pub_threads_.push_back(std::thread(&MessageManagerBase::PubLoopObjectlist, this, freq_objectlist));
        pub_threads_.push_back(std::thread(&MessageManagerBase::PubLoopFusionmap, this, freq_fusionmap));
        pub_threads_.push_back(std::thread(&MessageManagerBase::PubLoopRoadmarking, this, freq_roadmarking));
        pub_threads_.push_back(std::thread(&MessageManagerBase::PubLoopTrafficlight, this, freq_trafficlight));

        for (auto &t : pub_threads_)
        {
            t.detach();
        }
    }
} // namespace tievsim