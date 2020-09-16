#pragma once
#include <list>
#include <vector>
#include <string>

namespace tievsim
{

    class PidControllerBase
    {
    public:
        explicit PidControllerBase();

        virtual ~PidControllerBase() = default;

        virtual void Tick(double error) = 0;

    protected:
        virtual double Intergrate(double error);

        virtual double Differential(double error);

        double time_step_;
        double intergration_;
        std::list<double> error_buffer_;
    };

} // namespace tievsim