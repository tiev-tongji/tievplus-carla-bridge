#pragma once
#include <thread>
#include <mutex>
#include <condition_variable>
#include <chrono>
#include <vector>
#include <string>

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

namespace tievsim
{
        class MessageManagerBase
        {
        public:
                MessageManagerBase(const std::string &url);

                MessageManagerBase(MessageManagerBase const &) = delete;

                MessageManagerBase &operator=(MessageManagerBase const &) = delete;

                virtual ~MessageManagerBase();

#ifdef USE_ZCM
                void ChassisCommandHandler(const zcm::ReceiveBuffer *rbuf, const std::string &channel,
                                           const MsgChassisCommandSignal *msg);
#endif
#ifdef USE_LCM
                void ChassisCommandHandler(const lcm::ReceiveBuffer *rbuf, const std::string &chan,
                                           const MsgChassisCommandSignal *msg);
#endif

                void PublishCaninfo() const;
                void PublishNavinfo() const;
                void PublishFusionmap() const;
                void PublishObjectlist() const;
                void PublishRoadmarking() const;
                void PublishTrafficlight() const;
                void PublishAll() const;
                void PublishAllAsync(int freq_caninfo, int freq_navinfo, int freq_fusionmap,
                                     int freq_objectlist, int freq_roadmarking, int freq_trafficlight);
                void SubscribeAll();

        private:
                void PubLoopCaninfo(int freq);
                void PubLoopNavinfo(int freq);
                void PubLoopFusionmap(int freq);
                void PubLoopObjectlist(int freq);
                void PubLoopRoadmarking(int freq);
                void PubLoopTrafficlight(int freq);
#ifdef USE_LCM
                void SubLoop();
#endif

        public:
#ifdef USE_ZCM
                mutable zcm::ZCM tunnel_;
#endif
#ifdef USE_LCM
                mutable lcm::LCM tunnel_;
#endif
                MsgChassisCommandSignal chassis_command_;

        protected:
                MsgCanInfoSignal caninfo_;
                MsgNavInfoSignal navinfo_;
                MsgFusionMap fusionmap_;
                MsgPredictedObjectTrajectoryList object_list_;
                MsgRoadMarkingList roadmarking_list_;
                MsgTrafficLightSignal trafficlight_;

                mutable std::mutex chassis_command_mutex_;
                mutable std::mutex caninfo_mutex_;
                mutable std::mutex navinfo_mutex_;
                mutable std::mutex fusionmap_mutex_;
                mutable std::mutex objectlist_mutex_;
                mutable std::mutex roadmarking_mutex_;
                mutable std::mutex trafficlight_mutex_;

        private:
                std::vector<std::thread> pub_threads_;
#ifdef USE_ZCM
                std::vector<zcm::Subscription *> sub_threads_;
#endif
#ifdef USE_LCM
                std::vector<std::thread> sub_threads_;
#endif
                bool need_stop_;
        };
} // namespace tievsim