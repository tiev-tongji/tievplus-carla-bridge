#pragma once
#include <vector>
#include <string>
#include <list>
#include <string>
#include <cmath>
#include <exception>

namespace tievsim
{
    namespace utils
    {
        // units transform
        inline double rad2deg(double rad)
        {
            return rad / M_PI * 180.0;
        }

        inline double deg2rad(double deg)
        {
            return deg * M_PI / 180.0;
        }

        inline double kph2mps(double kph)
        {
            return kph / 3.6;
        }

        inline double mps2kph(double mps)
        {
            return mps * 3.6;
        }

        // basic math
        inline double norm2(double x, double y, double z = 0)
        {
            return sqrt(x * x + y * y + z * z);
        }

        // string operation
        inline bool StartWith(const std::string &s1, const std::string &s2)
        {
            return s2.size() <= s1.size() && s1.compare(0, s2.size(), s2) == 0;
        }

        // carla specified
        string GetAttribute(const vector<cc::ActorAttributeValue> &attrs, const string &attr_name)
        {
            for (auto attr : attrs)
            {
                if (attr.GetId() == attr_name)
                    return attr.GetValue();
            }
            return "not found";
        }

    } // namespace utils
} // namespace tievsim