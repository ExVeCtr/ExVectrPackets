#ifndef EXVECTR_TELECOM_HPP
#define EXVECTR_TELECOM_HPP

#include <concepts>
#include <cstddef>
#include <cstdint>
#include <stdint.h>

#include "ExVectrCore/CanSerialize.hpp"

namespace VCTR::packets::telecoms {

enum class TelecommandType : size_t {
  Reboot, //
  UpdateMode,
  MAX
};

enum class TelemetryType : size_t {
  Heartbeat, //
  GNSS = 5,  ///< Speed/satellites/altitude -- see Telemetry_GNSS.
  Battery,
  GNSSPosition, ///< Latitude/longitude -- see Telemetry_GNSSPosition. Kept
                ///< separate from GNSS since lat+lon don't fit alongside
                ///< the rest of the GPS fields in one OTA frame.
  MAX
};

} // namespace VCTR::packets::telecoms

#endif // EXVECTR_TELECOM_HPP