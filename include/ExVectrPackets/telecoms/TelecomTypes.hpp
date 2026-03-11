#ifndef EXVECTR_TELECOM_HPP
#define EXVECTR_TELECOM_HPP

#include <concepts>
#include <cstddef>
#include <cstdint>
#include <stdint.h>

#include "ExVectrCore/CanSerialize.hpp"

namespace VCTR::packets::telecoms {

enum class TelecommandType : size_t { Reboot, UpdateMode, MAX };

enum class TelemetryType : size_t { Heartbeat, MAX };

} // namespace VCTR::Controller::telecom

#endif // EXVECTR_TELECOM_HPP