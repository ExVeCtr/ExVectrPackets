#ifndef EXVECTRPACKET_PACKETVEHICLE_HPP
#define EXVECTRPACKET_PACKETVEHICLE_HPP


#include "stdint.h"

#include "ExVectrMath/matrix_base.hpp"
#include "ExVectrMath/matrix_vector.hpp"
#include "ExVectrMath/matrix_quaternion.hpp"


namespace VCTR
{

    namespace Net {


        struct PacketAttitude {

            float angularVelocity[3]; // Angular velocity [wx, wy, wz] in rad/s
            float quaternion[4]; // Quaternion [w, x, y, z] Usually tranformation from 
            float tiltAccuracy; // Tilt accuracy in radians
            float northAccuracy; // North accuracy in radians

            PacketAttitude(const Math::Vector_F& angVel = 0, const Math::Quat_F& attQuat = Math::Quat_F(1, 0, 0, 0), const float tiltAcc = 0, const float northAcc = 0) {
                for (int i = 0; i < 3; i++) angularVelocity[i] = angVel[i][0];
                for (int i = 0; i < 4; i++) quaternion[i] = attQuat[i][0];
                tiltAccuracy = tiltAcc;
                northAccuracy = northAcc;
            }

            Math::Vector_F getAngularVelocity() const {
                return Math::Vector_F({angularVelocity[0], angularVelocity[1], angularVelocity[2]});
            }
            Math::Quat_F getQuaternion() const {
                return Math::Quat_F(quaternion[0], quaternion[1], quaternion[2], quaternion[3]);
            }

        } __attribute__((packed));

        struct PacketPosition {

            float position[3]; // Position [x, y, z] in m
            float velocity[3]; // Velocity [vx, vy, vz] in m/s
            float hAccuracy; // Horizontal accuracy in m
            float vAccuracy; // Vertical accuracy in m

            PacketPosition(const Math::Vector_F& pos = 0, const Math::Vector_F& vel = 0, const float hAcc = 0, const float vAcc = 0) {
                for (int i = 0; i < 3; i++) position[i] = pos[i][0];
                for (int i = 0; i < 3; i++) velocity[i] = vel[i][0];
                hAccuracy = hAcc;
                vAccuracy = vAcc;
            }

            Math::Vector_F getPosition() const {
                return Math::Vector_F({position[0], position[1], position[2]});
            }
            Math::Vector_F getVelocity() const {
                return Math::Vector_F({velocity[0], velocity[1], velocity[2]});
            }

        } __attribute__((packed));

        struct PacketGPS {

            int32_t latitude; // Latitude in degrees multiplied by 1e7
            int32_t longitude; // Longitude in degrees multiplied by 1e7
            float altitude; // Altitude in meters   
            float velocity[3]; // Velocity [vx, vy, vz] in m/s
            uint8_t numSats; // Number of satellites used for the fix

            float positionAccuracy; // Position accuracy in meters
            float altitudeAccuracy; // Altitude accuracy in meters
            float velocityAccuracy; // Velocity accuracy in m/s

            PacketGPS(const Math::Vector_F& vel = 0, const float lat = 0, const float lon = 0, const float alt = 0, const uint8_t numSats = 0, const float posAcc = 0, const float altAcc = 0, const float velAcc = 0) {
                for (int i = 0; i < 3; i++) velocity[i] = vel[i][0];
                latitude = lat * 1e7; // Convert to integer representation
                longitude = lon * 1e7; // Convert to integer representation
                altitude = alt;
                this->numSats = numSats;
                positionAccuracy = posAcc;
                altitudeAccuracy = altAcc;
                velocityAccuracy = velAcc;
            }

            double getLatitude() const {
                return latitude / 1e7; // Convert to degrees
            }
            double getLongitude() const {
                return longitude / 1e7; // Convert to degrees
            }

        } __attribute__((packed));


    }

}

#endif