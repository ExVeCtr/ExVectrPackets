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

            PacketAttitude(const Math::Vector_F& angVel = 0, const Math::Quat_F& attQuat = Math::Quat_F(1, 0, 0, 0)) {
                for (int i = 0; i < 3; i++) angularVelocity[i] = angVel[i][0];
                for (int i = 0; i < 4; i++) quaternion[i] = attQuat[i][0];
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

            PacketPosition(const Math::Vector_F& pos = 0, const Math::Vector_F& vel = 0) {
                for (int i = 0; i < 3; i++) position[i] = pos[i][0];
                for (int i = 0; i < 3; i++) velocity[i] = vel[i][0];
            }

            Math::Vector_F getPosition() const {
                return Math::Vector_F({position[0], position[1], position[2]});
            }
            Math::Vector_F getVelocity() const {
                return Math::Vector_F({velocity[0], velocity[1], velocity[2]});
            }

        } __attribute__((packed));



    }

}

#endif