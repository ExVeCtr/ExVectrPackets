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
  ParameterPing = 0x28,
  ParameterDeviceInfo = 0x29,
  ParameterSettingsEntry = 0x2B,
  ParameterRead = 0x2C,
  ParameterWrite = 0x2D,
  ElrsStatus = 0x2E, // ELRS-specific good/bad packet count + status flags
  Command = 0x32,
};

/**
 * Status/step value carried by the CRSF "Parameter Settings (Entry)" COMMAND
 * data type and the "Parameter Value (Write)" request that drives it (see
 * `extern/CRSF details.md`, "COMMAND" payload / "Command chain example").
 * Named cmd_status in the spec; matches ExpressLRS's commandStep_e
 * (CRSFParameters.h) value-for-value.
 */
enum class CRSFCommandStep : uint8_t {
  Ready = 0,               // --> feedback: idle, nothing running
  Start = 1,               // <-- input: host asks to execute
  Progress = 2,            // --> feedback: command is executing
  ConfirmationNeeded = 3,  // --> feedback: awaiting user confirm/cancel
  Confirm = 4,             // <-- input: user confirmed
  Cancel = 5,              // <-- input: user cancelled
  Poll = 6,                // <-- input: host is requesting a status refresh
};

} // namespace VCTR::packets::crsf

#endif // EXVECTR_CRSFTYPES_HPP
