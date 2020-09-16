#include "PidControllerBase.hpp"

namespace tievsim
{

    PidControllerBase::PidControllerBase()
        : time_step_(0.01), intergration_(0.0), error_buffer_(){};

    double PidControllerBase::Intergrate(double error)
    {
        intergration_ += error * time_step_;
        return intergration_;
    }

    double PidControllerBase::Differential(double error)
    {
        error_buffer_.push_back(error);
        while (error_buffer_.size() >= 3)
        {
            error_buffer_.pop_front();
        }

        std::vector<double> vals;
        for (auto it = error_buffer_.rbegin(); it != error_buffer_.rend(); ++it)
        {
            vals.push_back(*it);
        }

        double result = 0;
        size_t len = vals.size();
        if (len >= 3)
        {
            // 3-point backwards
            result = (vals[len - 3] - 4 * vals[len - 2] + 3 * vals[len - 1]) / (2 * time_step_);
        }
        else if (len >= 2)
        {
            // 2-point backwards
            result = (vals[len - 1] - vals[len - 2]) / time_step_;
        }
        else
        {
            // no enough points
            result = 0;
        }
        return result;
    }

} // namespace tievsim