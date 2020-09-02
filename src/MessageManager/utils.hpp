#pragma once
#include <vector>
#include <string>
#include <list>
#include <string>
#ifndef _USE_MATH_DEFINES
#define _USE_MATH_DEFINES
#endif
#include <math.h>

#include "carla/geom/Transform.h"
#include "carla/client/ActorAttribute.h"
#include "carla/client/Map.h"

#include "PredictedObject.hpp"
#include "Lane.hpp"

using std::string;
using std::vector;

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

        bool CheckLaneType(const carla::road::Lane::LaneType &type)
        {
            typedef carla::road::Lane::LaneType LaneType;
            switch (type)
            {
            case LaneType::Driving:
                return true;
                break;
            case LaneType::Bidirectional:
                return true;
                break;
            case LaneType::Shoulder:
                return false;
                break;
            default:
                return false;
                break;
            }
        }

        void ParseLaneLineType(const carla::road::element::LaneMarking::Type &type, LaneLine *line)
        {
            typedef carla::road::element::LaneMarking::Type Type;
            switch (type)
            {
            case Type::Solid:
                line->line_type = LaneLine::kTypeDividing;
                break;
            case Type::SolidSolid:
                line->line_type = LaneLine::kTypeTypeNoPass;
                break;
            case Type::BrokenSolid:
                line->line_type = LaneLine::kTypeOneWayPass;
                break;
            case Type::SolidBroken:
                line->line_type = LaneLine::kTypeOneWayPass;
                break;
            case Type::Broken:
                line->line_type = LaneLine::kTypeGuiding;
                break;
            case Type::BrokenBroken:
                line->line_type = LaneLine::kTypeGuiding;
                break;
            default:
                line->line_type = LaneLine::kTypeTypeNoPass;
                break;
            };
        }

        /*---------------tiev-plus specified-------------*/

        bool InAreaTest(double x, double y, const PredictedObject &obj)
        {
            // only need to calculate the z dimension of vectors' cross products.
            // if all cross products have the same symbol, the point is located within the area.
            double z1 = (obj.bounding_box[0][1] - obj.bounding_box[0][0]) * (y - obj.bounding_box[1][0]) -
                        (obj.bounding_box[1][1] - obj.bounding_box[1][0]) * (x - obj.bounding_box[0][0]);
            bool is_neg = z1 < 0;
            double z2 = (obj.bounding_box[0][3] - obj.bounding_box[0][1]) * (y - obj.bounding_box[1][1]) -
                        (obj.bounding_box[1][3] - obj.bounding_box[1][1]) * (x - obj.bounding_box[0][1]);
            if ((is_neg && z2 >= 0) || (!is_neg && z2 < 0))
            {
                return false; // once result's symbol is different from previous ones, return false.
            }
            double z3 = (obj.bounding_box[0][2] - obj.bounding_box[0][3]) * (y - obj.bounding_box[1][3]) -
                        (obj.bounding_box[1][2] - obj.bounding_box[1][3]) * (x - obj.bounding_box[0][3]);
            if ((is_neg && z3 >= 0) || (!is_neg && z3 < 0))
            {
                return false;
            }
            double z4 = (obj.bounding_box[0][0] - obj.bounding_box[0][2]) * (y - obj.bounding_box[1][2]) -
                        (obj.bounding_box[1][0] - obj.bounding_box[1][2]) * (x - obj.bounding_box[0][2]);
            if ((is_neg && z4 >= 0) || (!is_neg && z4 < 0))
            {
                return false;
            }
            return true;
        }

    } // namespace utils
} // namespace tievsim