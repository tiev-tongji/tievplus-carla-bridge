#pragma once
#include <list>
#include <vector>
#include <string>

namespace tievsim
{

    namespace utils
    {

        class Range
        {
        public:
            Range(double min, double max);

            double get_min() const;
            double get_max() const;
            double get_len() const;

            /**
         * @brief 判断目标值是否在范围内
         *
         * @param target 目标值
         * @return true 在范围内
         * @return false 不在范围内
         */
            bool InRange(double target) const;

            void set_min(double value);
            void set_max(double value);

        private:
            double min_;
            double max_;
            double len_;
        };

        class PidController
        {
        public:
            /**
         * @brief Construct a new Pid Controller object
         *
         * @param kp 比例系数
         * @param ki 积分系数
         * @param kd 微分系数
         * @param time_step 控制器时间步长, s
         * @param sep_ratio 积分分离控制系数, 当积分项不在
         *  sep_ratio * [输出下界，输出上界] 内时, 取消积分
         * @param output_deadzone 输出死区, e.g. {0.0,0.1}
         * @param output_limit 输出上下限, e.g. {0.0,1.0}
         */
            PidController(double kp, double ki, double kd,
                          double time_step, double sep_ratio,
                          Range output_deadzone, Range output_limit);

            /**
         * @brief 计算下一时间步的控制量
         *
         * @param error 当前误差, 期望-实际
         * @return double 控制量
         */
            double Tick(double error);

            double get_result() const;
            double get_kp() const;
            double get_ki() const;
            double get_kd() const;
            double get_sep_ratio() const;
            const Range &get_output_deadzone() const;
            const Range &get_output_limit() const;

            void set_kp(double value);
            void set_ki(double value);
            void set_kd(double value);
            void set_sep_ratio(double value);
            void set_output_deadzone(const Range &range);
            void set_output_limit(const Range &range);

        private:
            double Intergrate(double error);
            double Differential(double error);

        private:
            double time_step_;
            double kp_;
            double ki_;
            double kd_;
            double result_;
            Range output_deadzone_;
            Range output_limit_;
            double intergration_;
            double sep_ratio_; // 积分分离控制比例
            std::list<double> error_buffer_;
        };

    } // namespace utils

} // namespace tievsim