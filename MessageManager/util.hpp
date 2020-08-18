#pragma once
#include <vector>
#include <string>
#include <list>
#include <string>
#include <cmath>

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
        inline bool start_with(const std::string &s1, const std::string &s2)
        {
            return s2.size() <= s1.size() && s1.compare(0, s2.size(), s2) == 0;
        }

        // geometry
        enum GeoOrientation
        {
            East,
            North,
            West,
            South
        };

        class Location
        {
        public:
            /**
             * @brief Construct a new Location object
             * 
             * @param x in meters
             * @param y in meters
             * @param z in meters
             * @param leftHand true for left-hand-axis, false for right-hand-axis
             * @param binding binding x-axis to given geo-orientation
             */
            Location(double x = 0.0, double y = 0.0, double z = 0.0,
                     bool leftHand = false, GeoOrientation binding = GeoOrientation::East)
                : x(x), y(y), z(z), leftHand(leftHand), binding(binding){};

        public:
            double x;
            double y;
            double z;

        private:
            bool leftHand;
            GeoOrientation binding;
        };

        class Rotation
        {
        public:
            /**
             * @brief Construct a new Rotation object
             * 
             * @param roll rotation around x, rad
             * @param pitch rotation around y, rad
             * @param yaw  rotation around z, rad
             * @param rollOrientation true for counter-clockwise, false for clockwise
             * @param pitchOrientation true for counter-clockwise, false for clockwise
             * @param yawOrientation true for counter-clockwise, false for clockwise
             */
            Rotation(double roll = 0.0, double pitch = 0.0, double yaw = 0.0,
                     bool rollOrientation = false, bool pitchOrientation = true, bool yawOrientation = true)
                : roll(roll), pitch(pitch), yaw(yaw),
                  ro(rollOrientation), po(pitchOrientation), yo(yawOrientation){};

        public:
            double roll;
            double pitch;
            double yaw;

        private:
            double ro;
            double po;
            double yo;
        };

        class Transform
        {
        public:
            Transform(double x = 0.0, double y = 0.0, double z = 0.0,
                      double roll = 0.0, double pitch = 0.0, double yaw = 0.0,
                      bool leftHand = false, GeoOrientation binding = GeoOrientation::East,
                      bool rollOrientation = false, bool pitchOrientation = true, bool yawOrientation = true)
                : loc(x, y, z, leftHand, binding),
                  rot(roll, pitch, yaw, rollOrientation, pitchOrientation, yawOrientation){};
            Transform(const Location &loc, const Rotation &rot)
                : loc(loc), rot(rot){};

        public:
            Location loc;
            Rotation rot;
        };

        inline Transform carla2vehframe(const Transform &veh, const Transform &target)
        {
            ;
        }

        inline Transform vehframe2carla(const Transform &veh, const Transform &target)
        {
            ;
        }

        inline bool checkInVehframe(const Transform &veh, const Transform &target,
                                    double front = 170, double back = 30,
                                    double left = 40, double right = 40,
                                    double top = 10, double bottom = 10)
        {
            ;
        }

        inline bool checkInArea()
        {
            ;
        }

    } // namespace utils
} // namespace tievsim