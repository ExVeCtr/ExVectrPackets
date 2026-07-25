#ifndef EXVECTR_CRSF_PACKETS_HPP
#define EXVECTR_CRSF_PACKETS_HPP

#include <cstddef>
#include <stdint.h>

#include "ExVectrPackets/Packets.hpp"
#include "ExVectrPackets/crsf/CRSFTypes.hpp"

namespace VCTR::packets::crsf {

/**
 * CRSF frame type 0x16, RC Channels Packed Payload.
 * Channel values are raw 11-bit ticks (0-2047), no domain scaling applied.
 */
class CRSFPacket_RcChannelsPacked {
public:
  uint16_t channels[16] = {0};

  size_t getPacketType() const;
  size_t getPacketDataType() const;

  size_t numBytes() const;
  void serialize(uint8_t *buffer) const;
  bool deserialize(const uint8_t *buffer);
};

/**
 * CRSF frame type 0x14, Link Statistics.
 */
class CRSFPacket_LinkStatistics {
public:
  uint8_t upRssiAnt1 = 0;   // Uplink RSSI Antenna 1 (dBm * -1)
  uint8_t upRssiAnt2 = 0;   // Uplink RSSI Antenna 2 (dBm * -1)
  uint8_t upLinkQuality = 0; // Uplink link quality (%)
  int8_t upSnr = 0;         // Uplink SNR (dB)
  uint8_t activeAntenna = 0;
  uint8_t rfProfile = 0;
  uint8_t upRfPower = 0;
  uint8_t downRssi = 0;       // Downlink RSSI (dBm * -1)
  uint8_t downLinkQuality = 0; // Downlink link quality (%)
  int8_t downSnr = 0;         // Downlink SNR (dB)

  size_t getPacketType() const;
  size_t getPacketDataType() const;

  size_t numBytes() const;
  void serialize(uint8_t *buffer) const;
  bool deserialize(const uint8_t *buffer);
};

/**
 * CRSF frame type 0x08, Battery Sensor.
 */
class CRSFPacket_BatterySensor {
public:
  float voltage = 0.0f;           // Volts
  float current = 0.0f;           // Amps
  uint32_t capacityUsedMah = 0;   // mAh, 24-bit range on the wire
  uint8_t remainingPercent = 0;   // %

  size_t getPacketType() const;
  size_t getPacketDataType() const;

  size_t numBytes() const;
  void serialize(uint8_t *buffer) const;
  bool deserialize(const uint8_t *buffer);
};

/**
 * CRSF frame type 0x02, GPS.
 */
class CRSFPacket_GPS {
public:
  double latitude = 0.0;      // degrees
  double longitude = 0.0;     // degrees
  float groundspeedKmh = 0.0f;
  float headingDeg = 0.0f;
  float altitudeM = 0.0f;
  uint8_t satellites = 0;

  size_t getPacketType() const;
  size_t getPacketDataType() const;

  size_t numBytes() const;
  void serialize(uint8_t *buffer) const;
  bool deserialize(const uint8_t *buffer);
};

/**
 * CRSF frame type 0x32, Direct Command, CommandID 0x10 (Crossfire) /
 * SubCommand 0x01 (Set receiver in bind mode). Extended-header frame.
 */
class CRSFPacket_CommandRxBind {
public:
  uint8_t destAddress = 0xEC; // R/C Receiver / Crossfire Rx
  uint8_t origAddress = 0xC8; // Flight controller

  size_t getPacketType() const;
  size_t getPacketDataType() const;

  size_t numBytes() const;
  void serialize(uint8_t *buffer) const;
  bool deserialize(const uint8_t *buffer);
};

/**
 * CRSF frame type 0x28, Parameter Ping Devices. Extended-header frame with no
 * payload -- a host (e.g. a handset's Lua config screen) sends this to
 * discover CRSF devices on the bus; each addressed device is expected to
 * reply with a CRSFPacket_ParameterDeviceInfo.
 */
class CRSFPacket_ParameterPing {
public:
  uint8_t destAddress = 0x00; // Broadcast, or a specific device address
  uint8_t origAddress = 0xEA; // Requesting device (e.g. the handset)

  size_t getPacketType() const;
  size_t getPacketDataType() const;

  size_t numBytes() const;
  void serialize(uint8_t *buffer) const;
  bool deserialize(const uint8_t *buffer);
};

/**
 * CRSF frame type 0x29, Parameter Device Information. Sent in reply to a
 * CRSFPacket_ParameterPing (or after any device ping) to describe this
 * device and how many parameters it exposes.
 */
class CRSFPacket_ParameterDeviceInfo {
public:
  uint8_t destAddress = 0xEA;
  uint8_t origAddress = 0xEE; // R/C Transmitter Module / Crossfire Tx
  char deviceName[16] = "ExVectr TX";
  // The ELRS handset Lua (elrs.lua) decides a device is a native ELRS TX
  // purely by testing this serial number == 0x454C5253 ('E','L','R','S').
  // Without the magic value it is treated as a generic CRSF device (wrong
  // handset addressing, no link-status indicator), so it must be sent as-is
  // to make the config screen behave like the real ELRS system. See
  // ExpressLRS's CRSFEndpoint::sendDeviceInformationPacket().
  uint32_t serialNumber = 0x454C5253;
  uint32_t hardwareId = 0;
  uint32_t firmwareId = 0;
  // Stats first: TX Temp (1), RX runtime/desync/FHSS offset/FHSS interval
  // (2-5), TX/RX missed slots (6-7); then Dyn Power enable/min/max (8-10);
  // then TX/RX Upload Mode (11-12); then bench-test commands RX Block Test/
  // TX Desync Test/RX Force Search (13-15); then RX Sync Offset readout +/-
  // buttons (16-18); then RX RC-frame gap readout + reset button (19-20).
  // The handset only ever requests parameter numbers 1..parametersTotal, so
  // this must be bumped every time a parameter is added -- forgetting to is
  // exactly why a newly-added entry is invisible in the Lua menu despite the
  // device answering reads for it.
  uint8_t parametersTotal = 20;
  uint8_t parameterVersion = 0;

  size_t getPacketType() const;
  size_t getPacketDataType() const;

  size_t numBytes() const;
  void serialize(uint8_t *buffer) const;
  bool deserialize(const uint8_t *buffer);
};

/**
 * CRSF frame type 0x2C, Parameter Settings (Read). Host request to (re-)read
 * a parameter entry (or one chunk of it, for parameters too big for a single
 * frame -- not needed for the single short COMMAND parameter this device
 * currently exposes).
 */
class CRSFPacket_ParameterRead {
public:
  uint8_t destAddress = 0xEE;
  uint8_t origAddress = 0xEA;
  uint8_t parameterNumber = 0;
  uint8_t chunkNumber = 0;

  size_t getPacketType() const;
  size_t getPacketDataType() const;

  size_t numBytes() const;
  void serialize(uint8_t *buffer) const;
  bool deserialize(const uint8_t *buffer);
};

/**
 * CRSF frame type 0x2D, Parameter Value (Write). Only the single-byte
 * payload used to write a COMMAND-type parameter's status (see
 * CRSFCommandStep) is supported -- this device does not currently expose any
 * other parameter type.
 */
class CRSFPacket_ParameterWrite {
public:
  uint8_t destAddress = 0xEE;
  uint8_t origAddress = 0xEA;
  uint8_t parameterNumber = 0;
  uint8_t commandStatus = 0; // CRSFCommandStep value written by the host

  size_t getPacketType() const;
  size_t getPacketDataType() const;

  size_t numBytes() const;
  void serialize(uint8_t *buffer) const;
  bool deserialize(const uint8_t *buffer);
};

/**
 * CRSF frame type 0x2B, Parameter Settings (Entry), COMMAND data type. This
 * is the wire format for a single "command" parameter a handset's Lua config
 * screen can list and execute -- currently only "Bind" is exposed (see
 * CRSFRcInput's parameter-1 handling). Always sent as a single chunk (the
 * name/info strings are short enough to always fit in one CRSF frame).
 */
class CRSFPacket_ParameterSettingsEntryCommand {
public:
  uint8_t destAddress = 0xEA;
  uint8_t origAddress = 0xEE;
  uint8_t parameterNumber = 1;
  uint8_t parentFolder = 0; // 0 = root folder
  char name[16] = "Bind";
  uint8_t status = 0;    // CRSFCommandStep value
  uint8_t timeout = 200; // ms * 100 (2 s), mirrors ExpressLRS's luaBind
  char info[32] = "";

  size_t getPacketType() const;
  size_t getPacketDataType() const;

  size_t numBytes() const;
  void serialize(uint8_t *buffer) const;
  bool deserialize(const uint8_t *buffer);
};

/**
 * CRSF frame type 0x2B, Parameter Settings (Entry), INFO data type. Read-only
 * string entry -- used here to show the TX module's own device temperature
 * inside the config menu (see CRSFRcInput's parameter-2 handling). Shares its
 * wire frame type (0x2B) with CRSFPacket_ParameterSettingsEntryCommand, which
 * is fine since this is only ever serialized as an outgoing reply, never
 * deserialized out of an incoming frame (the host only ever sends Ping/Read/
 * Write requests, never an Entry frame itself). Always sent as a single
 * chunk (name/info are short enough to always fit in one CRSF frame).
 */
class CRSFPacket_ParameterSettingsEntryInfo {
public:
  uint8_t destAddress = 0xEA;
  uint8_t origAddress = 0xEE;
  uint8_t parameterNumber = 2;
  uint8_t parentFolder = 0; // 0 = root folder
  char name[16] = "TX Temp";
  char info[32] = "";

  size_t getPacketType() const;
  size_t getPacketDataType() const;

  size_t numBytes() const;
  void serialize(uint8_t *buffer) const;
  bool deserialize(const uint8_t *buffer);
};

/**
 * CRSF frame type 0x2B, Parameter Settings (Entry), TEXT_SELECTION data type.
 * A menu of string options the handset can cycle through and write back the
 * chosen index of -- used here for the dynamic-power enable/min/max controls
 * (see CRSFRcInput's parameter-3/4/5 handling). Shares its wire frame type
 * (0x2B) with the Command/Info entry classes above; see
 * CRSFPacket_ParameterSettingsEntryInfo's doc comment for why that's fine.
 * Always sent as a single chunk.
 */
class CRSFPacket_ParameterSettingsEntryTextSelection {
public:
  uint8_t destAddress = 0xEA;
  uint8_t origAddress = 0xEE;
  uint8_t parameterNumber = 0;
  uint8_t parentFolder = 0;      // 0 = root folder
  char name[16] = "";
  char options[32] = "";         // ';'-delimited list of option labels
  uint8_t value = 0;             // current selection, index into options
  uint8_t min = 0;               // always 0
  uint8_t max = 0;               // index of the last option
  uint8_t defaultValue = 0;      // index shown as the "default" selection
  char unit[8] = "";

  size_t getPacketType() const;
  size_t getPacketDataType() const;

  size_t numBytes() const;
  void serialize(uint8_t *buffer) const;
  bool deserialize(const uint8_t *buffer);
};

/**
 * CRSF frame type 0x2E, ELRS Status. Extended-header frame the ELRS handset
 * Lua polls for (by writing parameter 0) to show the good/bad packet counter
 * and the little status icon in its title bar. Mirrors ExpressLRS's
 * elrsStatusParameter / TXModuleEndpoint::sendELRSstatus().
 */
class CRSFPacket_ElrsStatus {
public:
  uint8_t destAddress = 0xEA;
  uint8_t origAddress = 0xEE; // R/C Transmitter Module / Crossfire Tx
  uint8_t pktsBad = 0;        // "bad" packet count shown in the title bar
  uint16_t pktsGood = 0;      // "good" packet count shown in the title bar
  // Status/warning bitfield. Bit 0 (LUA_FLAG_CONNECTED) drives the "C" vs "-"
  // connection icon; higher bits are model-match/armed/critical warnings we
  // don't currently raise. Matches ExpressLRS's warningFlags enum.
  uint8_t flags = 0;
  char info[24] = ""; // warning message string (empty unless a flag is set)

  size_t getPacketType() const;
  size_t getPacketDataType() const;

  size_t numBytes() const;
  void serialize(uint8_t *buffer) const;
  bool deserialize(const uint8_t *buffer);
};

} // namespace VCTR::packets::crsf

#endif // EXVECTR_CRSF_PACKETS_HPP
