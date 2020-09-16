#include "TievplusController.hpp"

using std::string;
using std::vector;

namespace tievsim
{
    TievplusController::TievplusController(const string &parameter_filepath)
        : LongitudinalController(parameter_filepath)
    {
        PullParameter(parameter_filepath);
    }

    void TievplusController::PullParameter(const string &filepath)
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

        max_steer_ = d["max_steer"].GetDouble();
        max_steerwheel_ = d["max_steerwheel"].GetDouble();

        fclose(fp);
    }

    void TievplusController::Tick(double error_lon, double aim_steer)
    {
        steer = -aim_steer / 500;
        steer = steer < -1 ? -1 : steer;
        steer = steer > 1 ? 1 : steer;

        LongitudinalController::Tick(error_lon);
    }
} // namespace tievsim