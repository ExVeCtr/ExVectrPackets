#ifndef EXVECTR_PACKETS_HPP
#define EXVECTR_PACKETS_HPP

#include <concepts>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <stdint.h>

#include "ExVectrCore/handler.hpp"
#include "ExVectrCore/list_array.hpp"
#include "ExVectrCore/print.hpp"

#include "ExVectrCore/CanSerialize.hpp"

namespace VCTR::packets {

enum class PacketType : size_t {
  Telecommand, // Telecoms
  Telemetry,   // Telecoms
  RadioLink,   // RadioLink
  MAX          //
};

} // namespace VCTR::packets

namespace VCTR::packets {

template <typename T>
concept IsPacket = VCTR::Core::CanSerialize<T> && requires(const T a) {
  { a.getPacketType() } -> std::same_as<size_t>;
  { a.getPacketDataType() } -> std::same_as<size_t>;
};

} // namespace VCTR::packets

namespace VCTR::packets {

template <typename TYPE, size_t PACKETTYPE, size_t PACKETDATATYPE>
class PacketCustom {
public:
  TYPE data;

  size_t getPacketType() const { return PACKETTYPE; }
  size_t getPacketDataType() const { return PACKETDATATYPE; }

  size_t numBytes() const { return sizeof(TYPE) + 2 * sizeof(uint8_t); }
  void serialize(uint8_t *buffer) const {
    uint8_t packetType = static_cast<uint8_t>(getPacketType());
    uint8_t packetDataType = static_cast<uint8_t>(getPacketDataType());
    std::memcpy(buffer, &packetType, sizeof(uint8_t));
    std::memcpy(buffer + sizeof(uint8_t), &packetDataType, sizeof(uint8_t));
    std::memcpy(buffer + 2 * sizeof(uint8_t), &data, sizeof(TYPE));
  }
  bool deserialize(const uint8_t *buffer) {
    uint8_t packetType;
    uint8_t packetDataType;
    std::memcpy(&packetType, buffer, sizeof(uint8_t));
    std::memcpy(&packetDataType, buffer + sizeof(uint8_t), sizeof(uint8_t));
    if (packetType != static_cast<uint8_t>(PACKETTYPE) ||
        packetDataType != static_cast<uint8_t>(PACKETDATATYPE)) {
      return false;
    }
    std::memcpy(&data, buffer + 2 * sizeof(uint8_t), sizeof(TYPE));
    return true;
  }
};

} // namespace VCTR::packets

namespace VCTR::packets {

class PacketManager {

  const size_t packetTypeBits;
  const size_t packetDataTypeBits;

  using PacketSendingHandler =
      std::function<void(const VCTR::Core::ListArray<uint8_t> &)>;
  using PacketReceivingHandler =
      std::function<void(const VCTR::Core::ListArray<uint8_t> &)>;

  PacketSendingHandler sendingHandler;

  struct PacketHandler {
    PacketReceivingHandler handler;
  };

  Core::ListArray<PacketHandler> receiveHandlers;

public:
  PacketManager(PacketSendingHandler sendingHandler, size_t packetTypeBits = 8,
                size_t packetDataTypeBits = 8)
      : packetTypeBits(packetTypeBits), packetDataTypeBits(packetDataTypeBits) {
    this->sendingHandler = sendingHandler;
  }

  void receivePacketData(const Core::ListArray<uint8_t> &data) {
    for (size_t i = 0; i < receiveHandlers.size(); ++i) {
      auto &handler = receiveHandlers[i];
      handler.handler(data);
    }
  }

  template <IsPacket T> void sendPacket(const T &packet) {
    VCTR::Core::ListArray<uint8_t> buffer;
    buffer.setSize(packet.numBytes());
    packet.serialize(buffer.getPtr());
    sendingHandler(buffer);
  }

  template <IsPacket T>
  void addPacketReceiveHandler(std::function<void(const T &)> handler) {
    receiveHandlers.append({[handler](const Core::ListArray<uint8_t> &data) {
      T packet;
      if (data.size() != packet.numBytes()) {
        // return;
      }
      if (!packet.deserialize(data.getPtr())) {
        return;
      }
      handler(packet);
    }});
  }
};

} // namespace VCTR::packets

#endif // EXVECTR_PACKETS_HPP