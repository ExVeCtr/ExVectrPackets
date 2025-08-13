#ifndef EXVECTRPACKET_PACKETVEHICLE_HPP
#define EXVECTRPACKET_PACKETVEHICLE_HPP

#include "stdint.h"

#include "ExVectrMath/constants.hpp"
#include "ExVectrMath/matrix_base.hpp"
#include "ExVectrMath/matrix_vector.hpp"
#include "ExVectrMath/matrix_quaternion.hpp"

#include "data_packing.hpp"

namespace VCTR
{

    namespace Net
    {

        struct PacketAttitude
        {
        private:
            int16_t angularVelocity[3]; // Angular velocity [wx, wy, wz] in rad/s. Assuming max value of 1000 deg/s.
            int16_t quaternion[4];      // Quaternion [w, x, y, z]. Assuming max value of 1.0 for each component.
            int16_t tiltAccuracy;       // Tilt accuracy in radians. Assuming max value of 360 deg.
            int16_t northAccuracy;      // North accuracy in radians. Assuming max value of 360 deg.

        public:
            PacketAttitude(const Math::Vector_F &angVel = 0, const Math::Quat_F &attQuat = Math::Quat_F(1, 0, 0, 0), const float tiltAcc = 0, const float northAcc = 0)
            {
                for (int i = 0; i < 3; i++)
                {
                    angularVelocity[i] = packFixedPoint<float, int16_t>(angVel[i][0], 1000 * DEGREES);
                }
                for (int i = 0; i < 4; i++)
                {
                    quaternion[i] = packFixedPoint<float, int16_t>(attQuat[i][0], 1.0f);
                }
                tiltAccuracy = packFixedPoint<float, int16_t>(tiltAcc, 360 * DEGREES);
                northAccuracy = packFixedPoint<float, int16_t>(northAcc, 360 * DEGREES);
            }

            Math::Vector_F getAngularVelocity() const
            {
                return Math::Vector_F({unpackFixedPoint<int16_t, float>(angularVelocity[0], 1000 * DEGREES),
                                       unpackFixedPoint<int16_t, float>(angularVelocity[1], 1000 * DEGREES),
                                       unpackFixedPoint<int16_t, float>(angularVelocity[2], 1000 * DEGREES)});
            }
            Math::Quat_F getQuaternion() const
            {
                return Math::Quat_F(unpackFixedPoint<int16_t, float>(quaternion[0], 1.0f),
                                    unpackFixedPoint<int16_t, float>(quaternion[1], 1.0f),
                                    unpackFixedPoint<int16_t, float>(quaternion[2], 1.0f),
                                    unpackFixedPoint<int16_t, float>(quaternion[3], 1.0f));
            }
            float getTiltAccuracy() const
            {
                return unpackFixedPoint<int16_t, float>(tiltAccuracy, 360 * DEGREES);
            }
            float getNorthAccuracy() const
            {
                return unpackFixedPoint<int16_t, float>(northAccuracy, 360 * DEGREES);
            }

        }
        __attribute__((packed));

        struct PacketPosition
        {
        private:
            int16_t position[3]; // Position [x, y, z] in m. Assuming max value of 1000 meters
            int16_t velocity[3]; // Velocity [vx, vy, vz] in m/s. Assuming max value of 100 m/s.
            int16_t hAccuracy;   // Horizontal accuracy in m. Assuming max value of 10 meters.
            int16_t vAccuracy;   // Vertical accuracy in m. Assuming max value of 10 meters.

        public:
            PacketPosition(const Math::Vector_F &pos = 0, const Math::Vector_F &vel = 0, const float hAcc = 0, const float vAcc = 0)
            {
                for (int i = 0; i < 3; i++)
                {
                    position[i] = packFixedPoint<float, int16_t>(pos[i][0], 1000);
                }
                for (int i = 0; i < 3; i++)
                {
                    velocity[i] = packFixedPoint<float, int16_t>(vel[i][0], 100);
                }
                hAccuracy = packFixedPoint<float, int16_t>(hAcc, 10);
                vAccuracy = packFixedPoint<float, int16_t>(vAcc, 10);
            }

            Math::Vector_F getPosition() const
            {
                return Math::Vector_F({unpackFixedPoint<int16_t, float>(position[0], 1000),
                                       unpackFixedPoint<int16_t, float>(position[1], 1000),
                                       unpackFixedPoint<int16_t, float>(position[2], 1000)});
            }
            Math::Vector_F getVelocity() const
            {
                return Math::Vector_F({unpackFixedPoint<int16_t, float>(velocity[0], 100),
                                       unpackFixedPoint<int16_t, float>(velocity[1], 100),
                                       unpackFixedPoint<int16_t, float>(velocity[2], 100)});
            }
            float getHAccuracy() const
            {
                return unpackFixedPoint<int16_t, float>(hAccuracy, 10);
            }
            float getVAccuracy() const
            {
                return unpackFixedPoint<int16_t, float>(vAccuracy, 10);
            }

        } __attribute__((packed));

        struct PacketGPS
        {
        private:
            int32_t latitude;    // Latitude in degrees multiplied by 1e7
            int32_t longitude;   // Longitude in degrees multiplied by 1e7
            int16_t altitude;    // Altitude in meters. Assuming max value of 1000 meters
            int16_t velocity[3]; // Velocity [vx, vy, vz] in m/s. Assuming max value of 100 m/s.
            uint8_t numSats;     // Number of satellites used for the fix

            int16_t positionAccuracy; // Position accuracy in meters. Assuming max value of 10 meters
            int16_t altitudeAccuracy; // Altitude accuracy in meters. Assuming max value of 10 meters
            int16_t velocityAccuracy; // Velocity accuracy in m/s. Assuming max value of 10 m/s

        public:
            PacketGPS(const Math::Vector_F &vel = 0, const double lat = 0, const double lon = 0, const float alt = 0, const uint8_t numSats = 0, const float posAcc = 0, const float altAcc = 0, const float velAcc = 0)
            {
                for (int i = 0; i < 3; i++)
                {
                    velocity[i] = packFixedPoint<float, int16_t>(vel[i][0], 100);
                }
                latitude = lat * 1e7;                                          // Convert to integer representation
                longitude = lon * 1e7;                                         // Convert to integer representation
                altitude = packFixedPoint<float, int16_t>(alt, 1000);          // Convert altitude to integer representation
                positionAccuracy = packFixedPoint<float, int16_t>(posAcc, 10); // Convert position accuracy to integer representation
                altitudeAccuracy = packFixedPoint<float, int16_t>(altAcc, 10); // Convert altitude accuracy to integer representation
                velocityAccuracy = packFixedPoint<float, int16_t>(velAcc, 10); // Convert velocity accuracy to integer representation
                this->numSats = numSats;
            }

            double getLatitude() const
            {
                return latitude / 1e7; // Convert to degrees
            }
            double getLongitude() const
            {
                return longitude / 1e7; // Convert to degrees
            }
            double getAltitude() const
            {
                return unpackFixedPoint<int16_t, float>(altitude, 1000); // Convert to meters
            }
            Math::Vector_F getVelocity() const
            {
                return Math::Vector_F({unpackFixedPoint<int16_t, float>(velocity[0], 100),
                                       unpackFixedPoint<int16_t, float>(velocity[1], 100),
                                       unpackFixedPoint<int16_t, float>(velocity[2], 100)});
            }
            uint8_t getNumSats() const
            {
                return numSats;
            }
            float getPositionAccuracy() const
            {
                return unpackFixedPoint<int16_t, float>(positionAccuracy, 10); // Convert to meters
            }
            float getAltitudeAccuracy() const
            {
                return unpackFixedPoint<int16_t, float>(altitudeAccuracy, 10); // Convert to meters
            }
            float getVelocityAccuracy() const
            {
                return unpackFixedPoint<int16_t, float>(velocityAccuracy, 10); // Convert to m/s
            }

        } __attribute__((packed));

    }
}

#endif