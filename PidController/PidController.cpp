#include "PidController.hpp"

namespace tievsim
{

    namespace utils
    {

        Range::Range(double min, double max)
            : min_(min), max_(max), len_(max_ - min_){};

        double Range::get_min() const { return min_; }

        double Range::get_max() const { return max_; }

        double Range::get_len() const { return len_; }

        void Range::set_max(double value) { max_ = value; }

        void Range::set_min(double value) { min_ = value; }

        bool Range::InRange(double target) const
        {
            return target > min_ && target < max_;
        }

        PidController::PidController(double kp, double ki, double kd,
                                     double time_step, double sep_ratio,
                                     Range input_deadzone, Range output_deadzone)
            : time_step_(time_step), kp_(kp), ki_(ki), kd_(kd), result_(0.0),
              output_deadzone_(input_deadzone), output_limit_(output_deadzone),
              intergration_(0.0), sep_ratio_(sep_ratio), error_buffer_(){};

        double PidController::get_kp() const { return kp_; }

        double PidController::get_ki() const { return ki_; }

        double PidController::get_kd() const { return kd_; }

        double PidController::get_sep_ratio() const { return sep_ratio_; }

        double PidController::get_result() const { return result_; }

        const Range &PidController::get_output_deadzone() const { return output_deadzone_; }

        const Range &PidController::get_output_limit() const { return output_limit_; }

        void PidController::set_kp(double value) { kp_ = value; }

        void PidController::set_ki(double value) { ki_ = value; }

        void PidController::set_kd(double value) { kd_ = value; }

        void PidController::set_sep_ratio(double value) { sep_ratio_ = value; }

        void PidController::set_output_deadzone(const Range &range) { output_deadzone_ = range; }

        void PidController::set_output_limit(const Range &range) { output_limit_ = range; }

        double PidController::Tick(double error)
        {
            result_ = kp_ * error + ki_ * Intergrate(error) + kd_ * Differential(error);

            if (output_deadzone_.InRange(result_))
            {
                result_ = 0.0;
                return result_;
            }
            if (output_limit_.InRange(result_))
            {
                return result_;
            }
            if (result_ < output_limit_.get_min())
            {
                result_ = output_limit_.get_min();
                return result_;
            }
            else
            {
                result_ = output_limit_.get_max();
                return result_;
            }
        }

        double PidController::Intergrate(double error)
        {
            double temp = intergration_ + error * time_step_;
            // 积分分离，误差较大时，取消积分环节
            if (output_limit_.InRange(temp / sep_ratio_))
            {
                intergration_ = temp;
            }
            return intergration_;
        }

        double PidController::Differential(double error)
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

    } // namespace utils

} // namespace tievsim