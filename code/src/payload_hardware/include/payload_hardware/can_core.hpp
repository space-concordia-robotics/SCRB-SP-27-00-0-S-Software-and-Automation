// can_core.hpp -- the brain of the hardware interface, with NO ROS dependency.
//
// The ros2_control plugin (payload_can_system.cpp) is a thin adapter around this class.
// Keeping the logic here means it can be unit-tested on any laptop with plain g++.
//
// Data flow
//   RX thread  --on_rx()-->  [telemetry slots]  --read_states()-->  ros2_control state interfaces
//   ros2_control command interfaces --write_commands()--> encode --> Transmitter (SocketCAN)
#pragma once
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

#include "payload_hardware/can_codec.hpp"
#include "payload_hardware/can_table_generated.hpp"
#include "payload_hardware/can_types.hpp"

namespace payload_hardware {
namespace can {

// Anything that can put a frame on the bus (real SocketCAN, or a fake in tests).
class Transmitter {
 public:
  virtual ~Transmitter() = default;
  virtual bool send(const Frame& frame) = 0;   // must not block; false = could not send
};

// Latched fault bits, reported through "payload_link/fault_flags".
enum FaultBits : uint32_t {
  kFaultRxTimeout = 1u << 0,          // no frame from the payload for rx_timeout (not latched)
  kFaultBusError = 1u << 1,           // bus-off / error frame from the controller
  kFaultCommandNotReached = 1u << 2,  // a command never reached its readback after all retries
  kFaultCommandRejected = 1u << 3,    // a command was outside the range allowed by the CAN table
  kFaultTxFailed = 1u << 4,           // the socket refused a frame
};

struct CoreConfig {
  uint8_t competition = kCompUrc;        // which telemetry is "real" (the rest reads NaN)
  double stale_s = 0.5;                  // telemetry older than this reads NaN
  double rx_timeout_s = 0.5;             // silence longer than this = link down
  int retry_max = 2;                     // resend attempts when a readback is not reached
  bool strict_competition = true;        // reject commands that belong to the other competition
  bool honour_na_all_ones = true;        // per-signal sentinel switch from can_signals.yaml
};

struct BindReport {
  std::vector<std::string> errors;       // plugin must not start
  std::vector<std::string> warnings;     // informational
};

class Core {
 public:
  using Clock = std::chrono::steady_clock;
  static constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();

  // Diagnostic interfaces the core fills in when they are declared in the ros2_control block.
  static constexpr const char* kLinkOk = "payload_link/link_ok";
  static constexpr const char* kRxAgeMs = "payload_link/rx_age_ms";
  static constexpr const char* kFaultFlags = "payload_link/fault_flags";
  static constexpr const char* kTxCount = "payload_link/tx_count";
  static constexpr const char* kRxCount = "payload_link/rx_count";
  static constexpr const char* kRejectedCount = "payload_link/rejected_count";
  static constexpr const char* kClearFaults = "payload_control/clear_faults";

  explicit Core(CoreConfig cfg);

  // Match the generated bindings to the interfaces declared in the ros2_control block.
  // `state_names` / `command_names` are "component/interface" in the order of the caller's arrays.
  BindReport bind(const std::vector<std::string>& state_names,
                  const std::vector<std::string>& command_names);

  // RX thread: store one received frame.
  void on_rx(const Frame& frame, Clock::time_point now);
  // RX thread: the CAN controller reported an error (bus-off etc.).
  void on_bus_error();

  // Control loop, read(): fill every bound state interface (+ diagnostics).
  void read_states(double* states, Clock::time_point now);

  // Control loop, write(): turn changed command interfaces into frames.
  // `states` is what read_states() just produced (used for readback confirmation).
  void write_commands(const double* commands, const double* states, Clock::time_point now,
                      Transmitter& tx);

  // Deactivate: send the stop-type commands from can_signals.yaml (never a position move).
  void send_safe_state(Transmitter& tx);

  uint32_t fault_flags(Clock::time_point now) const;
  std::vector<std::string> take_events();     // human-readable log lines produced since last call

  // Visible for tests
  uint64_t build_command_word(size_t binding, const double* commands, bool* ok);
  size_t n_command_bindings() const { return cmd_.size(); }

 private:
  struct Slot { uint64_t word = 0; Clock::time_point t{}; bool valid = false; };
  struct StateBind { size_t idx; const MessageDef* msg; const SignalDef* sig; double scale, offset; };
  struct CmdValue { size_t idx; const SignalDef* sig; double scale, offset; };
  struct CmdBind {
    CommandBindingDef def;
    const MessageDef* msg = nullptr;
    std::vector<CmdValue> values;
    int trigger_idx = -1;
    int readback_idx = -1;
    bool has_last = false;
    uint64_t last_word = 0;
    bool trig_seen = false;
    double last_trigger = 0;
    bool pending = false;
    Clock::time_point t_sent{};
    int tries = 0;
  };

  void event(const std::string& s);
  bool applicable(const MessageDef& m) const { return (m.competitions & cfg_.competition) != 0; }
  static const SignalDef& sig(const MessageDef& m, int i) { return m.signals[i]; }
  Frame make_frame(uint64_t word) const;
  bool transmit(Transmitter& tx, uint64_t word);
  void check_confirmation(CmdBind& cb, const double* commands, const double* states,
                          Clock::time_point now, Transmitter& tx);

  CoreConfig cfg_;
  std::array<const MessageDef*, 64> msg_by_id_{};

  // bindings resolved by bind()
  std::vector<StateBind> state_;
  std::vector<CmdBind> cmd_;
  int diag_idx_[6];                 // positions of the diagnostic interfaces in the state array (-1 = absent)
  int clear_idx_ = -1;
  bool clear_seen_ = false;
  double clear_last_ = 0;

  // shared with the RX thread
  mutable std::mutex rx_mutex_;
  std::array<Slot, 64> slots_{};
  Clock::time_point last_rx_{};
  bool any_rx_ = false;

  std::atomic<uint32_t> latched_{0};
  std::atomic<uint64_t> rx_count_{0}, tx_count_{0}, rejected_count_{0};
  std::atomic<bool> bus_error_{false};

  std::mutex ev_mutex_;
  std::vector<std::string> events_;
};

}  // namespace can
}  // namespace payload_hardware
