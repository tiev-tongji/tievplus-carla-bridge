#pragma once

#include "rapidjson/document.h"
#include "rapidjson/filereadstream.h"

#include "PidControllerBase.hpp"

namespace tievsim
{
    class LongitudinalController : public PidControllerBase
    {
    public:
        explicit LongitudinalController(const std::string &parameter_filepath);

        virtual void Tick(double error);

        double throttle;
        double brake;

    protected:
        virtual double Intergrate(double error);

        //参数表
        double kp_throttle_;
        double ki_throttle_;
        double kd_throttle_;
        double kp_brake_;
        double ki_brake_;
        double kd_brake_;
        double sep_ratio_;
        double min_throttle_;
        double max_throttle_;
        double min_brake_;
        double max_brake_;
        double deadzone_throttle_;
        double deadzone_brake_;

    private:
        void PullParameter(const std::string &filepath);
    };
} // namespace tievsim