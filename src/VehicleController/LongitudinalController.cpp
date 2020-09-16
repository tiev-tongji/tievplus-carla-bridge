#include "LongitudinalController.hpp"

using std::string;
using std::vector;

namespace tievsim
{
    LongitudinalController::LongitudinalController(const string &parameter_filepath)
    {
        PullParameter(parameter_filepath);
    }

    void LongitudinalController::PullParameter(const string &filepath)
    {
        FILE *fp = fopen(filepath.c_str(), "r");
        if (fp == 0)
        {
            printf("[Error] cannot open %s.\n", filepath.c_str());
            return;
        }
        printf("[INFO] controller use %s.\n", filepath.c_str());
        char read_buffer[1024];
        rapidjson::FileReadStream is(fp, read_buffer, sizeof(read_buffer));
        rapidjson::Document d;
        d.ParseStream(is);

        time_step_ = d["time_step"].GetDouble();
        sep_ratio_ = d["intergration_sep_ratio"].GetDouble();
        kp_throttle_ = d["kp_throttle"].GetDouble();
        ki_throttle_ = d["ki_throttle"].GetDouble();
        kd_throttle_ = d["kd_throttle"].GetDouble();
        kp_brake_ = d["kp_brake"].GetDouble();
        ki_brake_ = d["ki_brake"].GetDouble();
        kd_brake_ = d["kd_throttle"].GetDouble();
        min_throttle_ = d["min_throttle"].GetDouble();
        max_throttle_ = d["max_throttle"].GetDouble();
        min_brake_ = d["min_brake"].GetDouble();
        max_brake_ = d["max_brake"].GetDouble();
        deadzone_throttle_ = d["deadzone_throttle"].GetDouble();
        deadzone_brake_ = d["deadzone_brake"].GetDouble();

        fclose(fp);
    }

    double LongitudinalController::Intergrate(double error)
    {
        double temp = intergration_ + error * time_step_;
        // 积分分离，当积分项作用超过一定阈值时停止积分
        if (error > 0)
        {
            if (temp * kp_throttle_ > max_throttle_ * sep_ratio_)
            {
                ;
            }
            else
            {
                intergration_ = temp;
            }
        }
        else
        {
            if (temp * kp_brake_ < -max_brake_ * sep_ratio_)
            {
                ;
            }
            else
            {
                intergration_ = temp;
            }
        }

        return intergration_;
    }

    void LongitudinalController::Tick(double error)
    {
        double interg = Intergrate(error);
        double diff = Differential(error);

        brake = -kp_brake_ * error - ki_brake_ * interg - kd_brake_ * diff;
        throttle = kp_throttle_ * error + ki_throttle_ * interg + kd_throttle_ * diff;
        throttle = brake > 0 ? 0 : throttle;

        throttle = throttle < deadzone_throttle_ ? 0 : throttle;
        throttle = throttle > 1 ? 1 : throttle;
        brake = brake < deadzone_brake_ ? 0 : brake;
        brake = brake > 1 ? 1 : brake;
    }

} // namespace tievsim