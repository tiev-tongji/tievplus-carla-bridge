#pragma once
#include <vector>
#include <string>
#include <list>
#include <string>
#include <cmath>

#include "carla/geom/Transform.h"
#include "carla/client/ActorAttribute.h"

namespace tievsim
{
    namespace cg = carla::geom;
    namespace cc = carla::client;
    namespace utils
    {
        /*----------------units transform----------------*/

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

        /*-----------------basic math--------------------*/

        inline double norm2(double x, double y, double z = 0)
        {
            return sqrt(x * x + y * y + z * z);
        }

        /*--------------string operation-----------------*/

        inline bool StartWith(const std::string &s1, const std::string &s2)
        {
            return s2.size() <= s1.size() && s1.compare(0, s2.size(), s2) == 0;
        }

        /*-----------------carla specified---------------*/

        string GetAttribute(const vector<cc::ActorAttributeValue> &attrs, const string &attr_name)
        {
            for (auto attr : attrs)
            {
                if (attr.GetId() == attr_name)
                    return attr.GetValue();
            }
            return "not found";
        }

        cg::Location ToVehFrame(const cg::Transform &veh_transform, const cg::Location &loc)
        {
            auto fvec = veh_transform.GetForwardVector();
            auto rvec = veh_transform.GetRightVector();
            auto uvec = veh_transform.GetUpVector();
            auto rloc = loc - veh_transform.location;
            float x = cg::Math::Dot(rloc, fvec);
            float y = -cg::Math::Dot(rloc, rvec);
            float z = cg::Math::Dot(rloc, uvec);
            return cg::Location{x, y, z};
        }

        cg::Location ToCarlaFrame(const cg::Transform &veh_transform, const cg::Location &rloc)
        {
            cg::Location loc{rloc.x, -rloc.y, rloc.z};
            veh_transform.TransformPoint(loc);
            return loc;
        }

    } // namespace utils
} // namespace tievsim