#ifndef EXVECTR_CRSFTYPES_HPP
#define EXVECTR_CRSFTYPES_HPP

#include <cstddef>
#include <cstdint>
#include <stdint.h>

namespace VCTR::packets::crsf {

/**
 * CRSF frame type IDs, as defined by the CRSF protocol spec (see
 * `extern/CRSF details.md`). Unlike TelecommandType/TelemetryType these values
 * are fixed by the external protocol, not sequential, so there is no MAX
 * sentinel.
 */
enum class CRSFFrameType : uint8_t {
  GPS = 0x02,
  BatterySensor = 0x08,
  LinkStatistics = 0x14,
  RcChannelsPacked = 0x16,
  Command = 0x32,
};

} // namespace VCTR::packets::crsf

#endif // EXVECTR_CRSFTYPES_HPP
