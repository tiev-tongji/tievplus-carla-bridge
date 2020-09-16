#pragma once

#include "rapidjson/document.h"
#include "rapidjson/filereadstream.h"

#include "LongitudinalController.hpp"

namespace tievsim
{
    class TievplusController : public LongitudinalController
    {
    public:
        explicit TievplusController(const std::string &parameter_filepath);

        void Tick(double error_lon, double error_steer);

        double steer;

    protected:
        double max_steer_;
        double max_steerwheel_;

    private:
        void PullParameter(const std::string &filepath);
    };
} // namespace tievsim