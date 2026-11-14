// can_types.hpp -- plain data structures shared by the generated table and the core.
#pragma once
#include <array>
#include <cstdint>
#include <string>
#include <vector>

namespace payload_hardware {
namespace can {

constexpr uint8_t kCompUrc = 1;   // bit 0
constexpr uint8_t kCompErc = 2;   // bit 1

// One named field inside a message's 57-bit payload.
struct SignalDef {
  const char* name;
  uint8_t bit;        // first payload bit (bit 0 = most significant payload bit)
  uint8_t width;      // number of bits
  bool is_signed;     // two's complement
  int64_t raw_min;    // allowed raw range for COMMANDS (from the sheet)
  int64_t raw_max;
  bool na_all_ones;   // telemetry: all-ones means "not applicable" (NaN)
};

// One row of the CAN table.
struct MessageDef {
  uint8_t id;                 // 6-bit message id
  const char* name;
  bool is_command;            // Jetson -> payload (else payload -> Jetson)
  uint8_t competitions;       // bitmask of kCompUrc / kCompErc
  const SignalDef* signals;
  uint8_t n_signals;
};

// telemetry signal -> state interface:  value = raw * scale + offset
struct StateBindingDef {
  const char* interface;      // "component/interface"
  uint8_t message_id;
  uint8_t signal_index;
  double scale;
  double offset;
};

struct ConstDef { uint8_t signal_index; int64_t raw; };

// command interface value -> signal:  raw = round(value * scale + offset)
struct ValueDef {
  uint8_t signal_index;
  const char* interface;
  double scale;
  double offset;
};

// How one CAN command message is produced from ros2_control command interfaces.
struct CommandBindingDef {
  uint8_t message_id;
  std::vector<ConstDef> consts;
  std::vector<ValueDef> values;
  const char* trigger;                // "" = send on value change; else send when this counter changes
  const char* readback_state;         // "" = no confirmation
  double tolerance;                   // |state - command| allowed (ros units)
  int timeout_ms;                     // time allowed to reach the command before resending
  bool skip_initial_if_at_readback;   // do not send a first "hold" command that equals the state
};

struct SafeCommandDef { uint8_t message_id; std::vector<ConstDef> raws; };

}  // namespace can
}  // namespace payload_hardware
