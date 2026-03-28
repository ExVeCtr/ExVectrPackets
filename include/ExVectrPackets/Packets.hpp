#ifndef EXVECTR_PACKETS_HPP
#define EXVECTR_PACKETS_HPP

#include <concepts>
#include <cstddef>
#include <cstdint>
#include <stdint.h>

#include "ExVectrCore/handler.hpp"
#include "ExVectrCore/list_array.hpp"

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

class PacketManager {

  const size_t packetTypeBits;
  const size_t packetDataTypeBits;

  using PacketSendingHandler =
      std::function<void(const VCTR::Core::ListArray<uint8_t> &)>;
  using PacketReceivingHandler =
      std::function<void(const VCTR::Core::ListArray<uint8_t> &)>;

  PacketSendingHandler sendingHandler;

  struct PacketHandler {
    size_t packetType;
    size_t packetDataType;
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
    receiveHandlers.append({T().getPacketType(), T().getPacketDataType(),
                            [handler](const Core::ListArray<uint8_t> &data) {
                              if (data.size() < 2) {
                                return;
                              }

                              T packet;
                              if (data.size() != packet.numBytes()) {
                                return;
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