#include "payload_hardware/can_core.hpp"

#include <algorithm>
#include <sstream>

namespace payload_hardware {
namespace can {

namespace {
double seconds(Core::Clock::duration d) { return std::chrono::duration<double>(d).count(); }
}  // namespace

Core::Core(CoreConfig cfg) : cfg_(cfg) {
  for (auto& p : msg_by_id_) p = nullptr;
  for (int i = 0; i < kNumMessages; ++i) msg_by_id_[kMessages[i].id] = &kMessages[i];
  for (auto& d : diag_idx_) d = -1;
}

void Core::event(const std::string& s) {
  std::lock_guard<std::mutex> lock(ev_mutex_);
  if (events_.size() < 200) events_.push_back(s);   // bounded: never grows without limit
}

std::vector<std::string> Core::take_events() {
  std::lock_guard<std::mutex> lock(ev_mutex_);
  std::vector<std::string> out;
  out.swap(events_);
  return out;
}

// ---------------------------------------------------------------------------------------
// bind(): connect the generated tables to the interfaces the URDF/xacro declared.
// Rule of thumb:  an interface that is declared but has no CAN source is an ERROR (it could
// never be served); a binding whose interfaces are not declared is simply skipped (this is
// how a URC-only xacro can leave out the ERC devices).
// ---------------------------------------------------------------------------------------
BindReport Core::bind(const std::vector<std::string>& state_names,
                      const std::vector<std::string>& command_names) {
  BindReport rep;
  state_.clear();
  cmd_.clear();

  std::unordered_map<std::string, size_t> s_idx, c_idx;
  for (size_t i = 0; i < state_names.size(); ++i) s_idx[state_names[i]] = i;
  for (size_t i = 0; i < command_names.size(); ++i) c_idx[command_names[i]] = i;

  std::vector<bool> s_used(state_names.size(), false), c_used(command_names.size(), false);

  // ---- diagnostics ----
  const char* diag_names[6] = {kLinkOk, kRxAgeMs, kFaultFlags, kTxCount, kRxCount, kRejectedCount};
  for (int i = 0; i < 6; ++i) {
    auto it = s_idx.find(diag_names[i]);
    diag_idx_[i] = (it == s_idx.end()) ? -1 : static_cast<int>(it->second);
    if (it != s_idx.end()) s_used[it->second] = true;
  }
  auto clr = c_idx.find(kClearFaults);
  clear_idx_ = (clr == c_idx.end()) ? -1 : static_cast<int>(clr->second);
  if (clr != c_idx.end()) c_used[clr->second] = true;

  // ---- telemetry -> state interfaces ----
  for (const auto& b : state_bindings()) {
    auto it = s_idx.find(b.interface);
    if (it == s_idx.end()) continue;                       // not declared: skip
    const MessageDef* m = msg_by_id_[b.message_id];
    state_.push_back({it->second, m, &m->signals[b.signal_index], b.scale, b.offset});
    s_used[it->second] = true;
  }

  // ---- command interfaces -> command messages ----
  for (const auto& d : command_bindings()) {
    CmdBind cb;
    cb.def = d;
    cb.msg = msg_by_id_[d.message_id];
    int declared = 0, total = 0;
    for (const auto& v : d.values) {
      ++total;
      auto it = c_idx.find(v.interface);
      if (it != c_idx.end()) { ++declared; cb.values.push_back({it->second, &cb.msg->signals[v.signal_index], v.scale, v.offset}); }
    }
    if (std::string(d.trigger).size()) {
      ++total;
      auto it = c_idx.find(d.trigger);
      if (it != c_idx.end()) { ++declared; cb.trigger_idx = static_cast<int>(it->second); }
    }
    if (declared == 0) continue;                           // whole binding absent: skip
    if (declared != total) {
      rep.errors.push_back(std::string("command binding for ") + cb.msg->name +
                           " is only partly declared in the ros2_control block");
      continue;
    }
    if (std::string(d.readback_state).size()) {
      auto it = s_idx.find(d.readback_state);
      if (it != s_idx.end()) cb.readback_idx = static_cast<int>(it->second);
      else rep.warnings.push_back(std::string(cb.msg->name) + ": readback state '" + d.readback_state + "' not declared; confirmation disabled");
    }
    for (const auto& v : cb.values) c_used[v.idx] = true;
    if (cb.trigger_idx >= 0) c_used[cb.trigger_idx] = true;
    cmd_.push_back(std::move(cb));
  }

  for (size_t i = 0; i < state_names.size(); ++i)
    if (!s_used[i]) rep.errors.push_back("state interface '" + state_names[i] + "' has no CAN source in can_signals.yaml");
  for (size_t i = 0; i < command_names.size(); ++i)
    if (!c_used[i]) rep.errors.push_back("command interface '" + command_names[i] + "' has no CAN message in can_signals.yaml");
  return rep;
}

// ---------------------------------------------------------------------------------------
// RX side
// ---------------------------------------------------------------------------------------
void Core::on_rx(const Frame& f, Clock::time_point now) {
  if (f.id != kArbitrationId || f.dlc < 8) return;          // not ours / truncated
  const uint64_t w = to_word(f);
  if (get_dest(w) != kDestJetson) return;                   // a command (our own echo) or foreign traffic
  const uint8_t id = get_msg_id(w);
  if (!msg_by_id_[id]) { event("rx: unknown message id " + std::to_string(id)); return; }
  std::lock_guard<std::mutex> lock(rx_mutex_);
  slots_[id] = {w, now, true};
  last_rx_ = now;
  any_rx_ = true;
  ++rx_count_;
}

void Core::on_bus_error() {
  bus_error_ = true;
  latched_ |= kFaultBusError;
}

// ---------------------------------------------------------------------------------------
// read(): decode telemetry -> state interfaces
// ---------------------------------------------------------------------------------------
void Core::read_states(double* states, Clock::time_point now) {
  double age_ms = kNaN;
  bool link_ok = false;
  {
    std::lock_guard<std::mutex> lock(rx_mutex_);
    for (const auto& b : state_) {
      double v = kNaN;
      const Slot& s = slots_[b.msg->id];
      // NaN unless: message belongs to this competition, has arrived, and is fresh.
      if (applicable(*b.msg) && s.valid && seconds(now - s.t) <= cfg_.stale_s) {
        const uint64_t raw = get_payload(s.word, b.sig->bit, b.sig->width);
        if (cfg_.honour_na_all_ones && b.sig->na_all_ones && raw == mask_bits(b.sig->width)) {
          v = kNaN;                                         // firmware says "not applicable"
        } else {
          const int64_t sr = b.sig->is_signed ? sign_extend(raw, b.sig->width) : static_cast<int64_t>(raw);
          v = static_cast<double>(sr) * b.scale + b.offset;
        }
      }
      states[b.idx] = v;
    }
    if (any_rx_) {
      age_ms = seconds(now - last_rx_) * 1000.0;
      link_ok = seconds(now - last_rx_) <= cfg_.rx_timeout_s;
    }
  }
  uint32_t flags = latched_.load();
  if (!link_ok) flags |= kFaultRxTimeout;                   // live condition, not latched
  if (diag_idx_[0] >= 0) states[diag_idx_[0]] = link_ok ? 1.0 : 0.0;
  if (diag_idx_[1] >= 0) states[diag_idx_[1]] = age_ms;
  if (diag_idx_[2] >= 0) states[diag_idx_[2]] = static_cast<double>(flags);
  if (diag_idx_[3] >= 0) states[diag_idx_[3]] = static_cast<double>(tx_count_.load());
  if (diag_idx_[4] >= 0) states[diag_idx_[4]] = static_cast<double>(rx_count_.load());
  if (diag_idx_[5] >= 0) states[diag_idx_[5]] = static_cast<double>(rejected_count_.load());
}

uint32_t Core::fault_flags(Clock::time_point now) const {
  uint32_t flags = latched_.load();
  std::lock_guard<std::mutex> lock(rx_mutex_);
  if (!any_rx_ || seconds(now - last_rx_) > cfg_.rx_timeout_s) flags |= kFaultRxTimeout;
  return flags;
}

// ---------------------------------------------------------------------------------------
// write(): encode commands
// ---------------------------------------------------------------------------------------
Frame Core::make_frame(uint64_t word) const {
  Frame f;
  f.id = kArbitrationId;
  from_word(f, word);
  return f;
}

bool Core::transmit(Transmitter& tx, uint64_t word) {
  if (tx.send(make_frame(word))) { ++tx_count_; return true; }
  latched_ |= kFaultTxFailed;
  return false;
}

// Build the 64-bit word for one command binding from the current command values.
// ok=false means "not sendable": a value is NaN (nobody commanded it yet) or out of range.
uint64_t Core::build_command_word(size_t binding, const double* commands, bool* ok) {
  CmdBind& cb = cmd_[binding];
  *ok = true;
  uint64_t w = make_header(kDestPayload, cb.msg->id);
  for (const auto& c : cb.def.consts) {
    const SignalDef& s = cb.msg->signals[c.signal_index];
    w = put_payload(w, s.bit, s.width, static_cast<uint64_t>(c.raw));
  }
  for (const auto& v : cb.values) {
    const double x = commands[v.idx];
    if (std::isnan(x)) { *ok = false; return 0; }           // NaN = "no command yet" in ros2_control
    const double rawf = x * v.scale + v.offset;
    const long long raw = std::llround(rawf);
    if (!std::isfinite(rawf) || raw < v.sig->raw_min || raw > v.sig->raw_max) {
      ++rejected_count_;
      latched_ |= kFaultCommandRejected;
      std::ostringstream os;
      os << "rejected " << cb.msg->name << "." << v.sig->name << ": value " << x << " -> raw " << raw
         << " outside [" << v.sig->raw_min << ", " << v.sig->raw_max << "]";
      event(os.str());
      *ok = false;
      return 0;
    }
    w = put_payload(w, v.sig->bit, v.sig->width, static_cast<uint64_t>(raw));
  }
  return w;
}

// Has the device reached what we asked? If not in time: resend (bounded), then raise a fault.
void Core::check_confirmation(CmdBind& cb, const double* commands, const double* states,
                              Clock::time_point now, Transmitter& tx) {
  if (!cb.pending || cb.readback_idx < 0 || cb.values.empty()) return;
  const double target = commands[cb.values[0].idx];
  const double have = states[cb.readback_idx];
  if (!std::isnan(have) && !std::isnan(target) && std::fabs(have - target) <= cb.def.tolerance) {
    cb.pending = false;                                     // arrived
  } else if (seconds(now - cb.t_sent) * 1000.0 > cb.def.timeout_ms) {
    if (cb.tries < cfg_.retry_max) {                        // bounded retry, NOT a continuous stream
      ++cb.tries;
      cb.t_sent = now;
      event(std::string("resending ") + cb.msg->name + " (try " + std::to_string(cb.tries) + ")");
      transmit(tx, cb.last_word);
    } else {
      cb.pending = false;
      latched_ |= kFaultCommandNotReached;
      event(std::string("command not reached: ") + cb.msg->name);
    }
  }
}

void Core::write_commands(const double* commands, const double* states, Clock::time_point now,
                          Transmitter& tx) {
  // Fault reset: a changed "clear_faults" counter clears the latched bits.
  if (clear_idx_ >= 0 && !std::isnan(commands[clear_idx_])) {
    if (clear_seen_ && commands[clear_idx_] != clear_last_) { latched_ = 0; bus_error_ = false; }
    clear_seen_ = true;
    clear_last_ = commands[clear_idx_];
  }

  for (size_t i = 0; i < cmd_.size(); ++i) {
    CmdBind& cb = cmd_[i];
    bool send_now = false;
    bool ok = true;
    uint64_t word = 0;

    if (cb.trigger_idx >= 0) {
      // ---- trigger style (home, zero, kill, timed actions) ----
      // Send once per change of a counter.  The very first value only fires if it is non-zero,
      // so an initial 0 is a baseline, not a command.
      const double t = commands[cb.trigger_idx];
      if (std::isnan(t)) continue;
      const bool changed = cb.trig_seen ? (t != cb.last_trigger) : (t != 0.0);
      if (changed) {
        word = build_command_word(i, commands, &ok);
        if (!ok) continue;                                  // parameters not ready: keep the trigger pending
        send_now = true;
      }
      cb.trig_seen = true;
      cb.last_trigger = t;
    } else {
      // ---- value style: send only when the encoded word differs from the last one sent ----
      word = build_command_word(i, commands, &ok);
      if (!ok) continue;                                    // NaN (no command yet) or out of range
      if (!cb.has_last) {
        // Controllers write "hold the current position" when they activate. Don't send that.
        if (cb.def.skip_initial_if_at_readback && cb.readback_idx >= 0 && cb.values.size() == 1 &&
            !std::isnan(states[cb.readback_idx]) &&
            std::fabs(states[cb.readback_idx] - commands[cb.values[0].idx]) <= cb.def.tolerance) {
          cb.has_last = true;
          cb.last_word = word;
          continue;
        }
        send_now = true;
      } else if (word != cb.last_word) {
        send_now = true;
      }
    }

    if (send_now) {
      if (cfg_.strict_competition && !applicable(*cb.msg)) {
        ++rejected_count_;
        latched_ |= kFaultCommandRejected;
        event(std::string("rejected ") + cb.msg->name + ": belongs to the other competition");
        cb.has_last = true;                                 // don't repeat the complaint every cycle
        cb.last_word = word;
        continue;
      }
      if (transmit(tx, word)) {
        cb.has_last = true;
        cb.last_word = word;
        cb.pending = (cb.readback_idx >= 0);
        cb.t_sent = now;
        cb.tries = 0;
      }
      continue;
    }

    check_confirmation(cb, commands, states, now, tx);
  }
}

void Core::send_safe_state(Transmitter& tx) {
  for (const auto& s : safe_state_commands()) {
    const MessageDef* m = msg_by_id_[s.message_id];
    uint64_t w = make_header(kDestPayload, m->id);
    for (const auto& r : s.raws) {
      const SignalDef& sg = m->signals[r.signal_index];
      w = put_payload(w, sg.bit, sg.width, static_cast<uint64_t>(r.raw));
    }
    transmit(tx, w);                                        // best effort
  }
}

}  // namespace can
}  // namespace payload_hardware
