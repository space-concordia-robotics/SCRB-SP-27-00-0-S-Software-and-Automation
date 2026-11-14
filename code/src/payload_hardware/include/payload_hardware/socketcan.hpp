// socketcan.hpp -- thin Linux SocketCAN wrapper (works with can0 or the virtual vcan0).
#pragma once
#include <string>

#include "payload_hardware/can_codec.hpp"
#include "payload_hardware/can_core.hpp"

namespace payload_hardware {
namespace can {

enum class RecvStatus { kFrame, kTimeout, kBusError, kClosed };

class SocketCan : public Transmitter {
 public:
  SocketCan() = default;
  ~SocketCan() override { close(); }
  SocketCan(const SocketCan&) = delete;
  SocketCan& operator=(const SocketCan&) = delete;

  // Open `ifname` (e.g. "can0", "vcan0").  Returns false and fills `err` on failure.
  bool open(const std::string& ifname, std::string* err);
  void close();
  bool is_open() const { return fd_ >= 0; }

  // Non-blocking transmit (called from the control loop).  False if the kernel queue is full.
  bool send(const Frame& frame) override;

  // Blocking receive with timeout (called from the RX thread).
  RecvStatus recv(Frame* frame, int timeout_ms);

 private:
  int fd_ = -1;
};

}  // namespace can
}  // namespace payload_hardware
