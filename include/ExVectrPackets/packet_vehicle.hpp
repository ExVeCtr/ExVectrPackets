#ifndef EXVECTRPACKET_PACKETVEHICLE_HPP
#define EXVECTRPACKET_PACKETVEHICLE_HPP

#include "stdint.h"

#include "ExVectrMath/constants.hpp"
#include "ExVectrMath/matrix_base.hpp"
#include "ExVectrMath/matrix_vector.hpp"
#include "ExVectrMath/matrix_quaternion.hpp"

namespace VCTR
{

    namespace Net
    {

        struct PacketAttitude
        {

            int16_t angularVelocity[3]; // Angular velocity [wx, wy, wz] in rad/s. Assuming max value of 1000 deg/s.
            int16_t quaternion[4];      // Quaternion [w, x, y, z]. Assuming max value of 1.0 for each component.
            int16_t tiltAccuracy;       // Tilt accuracy in radians. Assuming max value of 360 deg.
            int16_t northAccuracy;      // North accuracy in radians. Assuming max value of 360 deg.

            PacketAttitude(const Math::Vector_F &angVel = 0, const Math::Quat_F &attQuat = Math::Quat_F(1, 0, 0, 0), const float tiltAcc = 0, const float northAcc = 0)
            {
                float buf = 0;
                for (int i = 0; i < 3; i++)
                {
                    buf = angVel[i][0] / (1000 * DEGREES) * INT16_MAX;
                    buf = buf > INT16_MAX ? INT16_MAX : buf < -INT16_MAX ? -INT16_MAX
                                                                         : buf;
                    angularVelocity[i] = buf;
                }
                for (int i = 0; i < 4; i++)
                {
                    buf = attQuat[i][0] * INT16_MAX;
                    buf = buf > INT16_MAX ? INT16_MAX : buf < -INT16_MAX ? -INT16_MAX
                                                                         : buf;
                    quaternion[i] = buf;
                }
                buf = tiltAcc / (360 * DEGREES) * INT16_MAX;
                buf = buf > INT16_MAX ? INT16_MAX : buf < -INT16_MAX ? -INT16_MAX
                                                                     : buf;
                tiltAccuracy = buf;
                buf = northAcc / (360 * DEGREES) * INT16_MAX;
                buf = buf > INT16_MAX ? INT16_MAX : buf < -INT16_MAX ? -INT16_MAX
                                                                     : buf;
                northAccuracy = buf;
            }

            Math::Vector_F getAngularVelocity() const
            {
                return Math::Vector_F({(float)angularVelocity[0] / INT16_MAX * 1000 * DEGREES, (float)angularVelocity[1] / INT16_MAX * 1000 * DEGREES, (float)angularVelocity[2] / INT16_MAX * 1000 * DEGREES});
            }
            Math::Quat_F getQuaternion() const
            {
                return Math::Quat_F((float)quaternion[0] / INT16_MAX, (float)quaternion[1] / 10000.0f, (float)quaternion[2] / 10000.0f, (float)quaternion[3] / 10000.0f);
            }
            float getTiltAccuracy() const
            {
                return (float)tiltAccuracy / INT16_MAX * 360 * DEGREES; // Convert to radians
            }
            float getNorthAccuracy() const
            {
                return (float)northAccuracy / INT16_MAX * 360 * DEGREES; // Convert to radians
            }

        } __attribute__((packed));

        struct PacketPosition
        {

            int16_t position[3]; // Position [x, y, z] in m. Assuming max value of 1000 meters
            int16_t velocity[3]; // Velocity [vx, vy, vz] in m/s. Assuming max value of 100 m/s.
            int16_t hAccuracy;   // Horizontal accuracy in m. Assuming max value of 10 meters.
            int16_t vAccuracy;   // Vertical accuracy in m. Assuming max value of 10 meters.

            PacketPosition(const Math::Vector_F &pos = 0, const Math::Vector_F &vel = 0, const float hAcc = 0, const float vAcc = 0)
            {
                float buf = 0;
                for (int i = 0; i < 3; i++)
                {
                    buf = pos[i][0] * 1000 / INT16_MAX;
                    position[i] = buf > INT16_MAX ? INT16_MAX : buf < -INT16_MAX ? -INT16_MAX
                                                                                 : buf;
                }
                for (int i = 0; i < 3; i++)
                {
                    buf = vel[i][0] * 100 / INT16_MAX;
                    velocity[i] = buf > INT16_MAX ? INT16_MAX : buf < -INT16_MAX ? -INT16_MAX
                                                                                 : buf;
                }
                buf = hAcc / 10 * INT16_MAX;
                hAccuracy = buf > INT16_MAX ? INT16_MAX : buf < -INT16_MAX ? -INT16_MAX
                                                                           : buf;
                buf = vAcc / 10 * INT16_MAX;
                vAccuracy = buf > INT16_MAX ? INT16_MAX : buf < -INT16_MAX ? -INT16_MAX
                                                                           : buf;
            }

            Math::Vector_F getPosition() const
            {
                return Math::Vector_F({(float)position[0] * 1000 / INT16_MAX, (float)position[1] * 1000 / INT16_MAX, (float)position[2] * 1000 / INT16_MAX});
            }
            Math::Vector_F getVelocity() const
            {
                return Math::Vector_F({(float)velocity[0] * 100 / INT16_MAX, (float)velocity[1] * 100 / INT16_MAX, (float)velocity[2] * 100 / INT16_MAX});
            }
            float getHAccuracy() const
            {
                return (float)hAccuracy / INT16_MAX * 10; // Convert to meters
            }
            float getVAccuracy() const
            {
                return (float)vAccuracy / INT16_MAX * 10; // Convert to meters
            }

        } __attribute__((packed));

        struct PacketGPS
        {

            int32_t latitude;    // Latitude in degrees multiplied by 1e7
            int32_t longitude;   // Longitude in degrees multiplied by 1e7
            int16_t altitude;    // Altitude in meters. Assuming max value of 1000 meters
            int16_t velocity[3]; // Velocity [vx, vy, vz] in m/s. Assuming max value of 100 m/s.
            uint8_t numSats;     // Number of satellites used for the fix

            int16_t positionAccuracy; // Position accuracy in meters. Assuming max value of 10 meters
            int16_t altitudeAccuracy; // Altitude accuracy in meters. Assuming max value of 10 meters
            int16_t velocityAccuracy; // Velocity accuracy in m/s. Assuming max value of 10 m/s

            PacketGPS(const Math::Vector_F &vel = 0, const float lat = 0, const float lon = 0, const float alt = 0, const uint8_t numSats = 0, const float posAcc = 0, const float altAcc = 0, const float velAcc = 0)
            {
                float buf = 0;
                for (int i = 0; i < 3; i++)
                {
                    buf = vel[i][0] * 100 / INT16_MAX;
                    velocity[i] = buf > INT16_MAX ? INT16_MAX : buf < -INT16_MAX ? -INT16_MAX
                                                                                 : buf;
                }
                latitude = lat * 1e7;         // Convert to integer representation
                longitude = lon * 1e7;        // Convert to integer representation
                buf = alt * 1000 / INT16_MAX; // Convert altitude to integer representation
                altitude = buf > INT16_MAX ? INT16_MAX : buf < -INT16_MAX ? -INT16_MAX
                                                                          : buf;
                buf = posAcc / 10 * INT16_MAX; // Convert position accuracy to integer representation
                positionAccuracy = buf > INT16_MAX ? INT16_MAX : buf < -INT16_MAX ? -INT16_MAX
                                                                                  : buf;
                buf = altAcc / 10 * INT16_MAX; // Convert altitude accuracy to integer representation
                altitudeAccuracy = buf > INT16_MAX ? INT16_MAX : buf < -INT16_MAX ? -INT16_MAX
                                                                                  : buf;
                buf = velAcc / 10 * INT16_MAX; // Convert velocity accuracy to integer representation
                velocityAccuracy = buf > INT16_MAX ? INT16_MAX : buf < -INT16_MAX ? -INT16_MAX
                                                                                  : buf;
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
                return altitude / 1000.0; // Convert to meters
            }
            Math::Vector_F getVelocity() const
            {
                return Math::Vector_F({(float)velocity[0] * 100 / INT16_MAX, (float)velocity[1] * 100 / INT16_MAX, (float)velocity[2] * 100 / INT16_MAX});
            }
            uint8_t getNumSats() const
            {
                return numSats;
            }
            float getPositionAccuracy() const
            {
                return (float)positionAccuracy / INT16_MAX * 10; // Convert to meters
            }
            float getAltitudeAccuracy() const
            {
                return (float)altitudeAccuracy / INT16_MAX * 10; // Convert to meters
            }
            float getVelocityAccuracy() const
            {
                return (float)velocityAccuracy / INT16_MAX * 10; // Convert to m/s
            }

        } __attribute__((packed));

    }

}

#endif