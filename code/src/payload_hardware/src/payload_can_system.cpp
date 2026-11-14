#include "payload_hardware/payload_can_system.hpp"

#include <chrono>
#include <cmath>
#include <limits>

#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/logger.hpp"
#include "rclcpp/logging.hpp"

namespace payload_hardware {

namespace {
const rclcpp::Logger kLogger = rclcpp::get_logger("PayloadCanSystem");
constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();
}  // namespace

PayloadCanSystem::~PayloadCanSystem() { stop_rx_thread(); }

// =======================================================================================
// on_init: read parameters, collect the interfaces the xacro declared, build the Core.
// Runs once when the controller manager loads the plugin.
// =======================================================================================
PayloadCanSystem::CallbackReturn PayloadCanSystem::on_init(const hardware_interface::HardwareInfo& info) {
  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }

  auto param = [&](const std::string& key, const std::string& def) {
    auto it = info_.hardware_parameters.find(key);
    return it == info_.hardware_parameters.end() ? def : it->second;
  };
  auto as_bool = [](const std::string& s) { return s == "true" || s == "True" || s == "1"; };

  can_interface_ = param("can_interface", "can0");
  safe_state_on_deactivate_ = as_bool(param("safe_state_on_deactivate", "true"));
  kill_on_deactivate_ = as_bool(param("kill_on_deactivate", "false"));

  can::CoreConfig cfg;
  const std::string comp = param("competition", "urc");
  if (comp == "urc") cfg.competition = can::kCompUrc;
  else if (comp == "erc") cfg.competition = can::kCompErc;
  else {
    RCLCPP_ERROR(kLogger, "hardware parameter 'competition' must be 'urc' or 'erc' (got '%s')", comp.c_str());
    return CallbackReturn::ERROR;
  }
  cfg.stale_s = std::stod(param("stale_ms", "500")) / 1000.0;
  cfg.rx_timeout_s = std::stod(param("rx_timeout_ms", "500")) / 1000.0;
  cfg.retry_max = std::stoi(param("retry_max", "2"));
  cfg.strict_competition = as_bool(param("strict_competition", "true"));
  cfg.honour_na_all_ones = as_bool(param("honour_na_all_ones", "true"));

  // Every joint and gpio in the <ros2_control> block contributes interfaces.  The full name
  // "component/interface" is what can_signals.yaml refers to.
  auto collect = [&](const hardware_interface::ComponentInfo& c) {
    for (const auto& si : c.state_interfaces) {
      state_component_.push_back(c.name);
      state_interface_.push_back(si.name);
      state_names_.push_back(c.name + "/" + si.name);
      state_values_.push_back(si.initial_value.empty() ? kNaN : std::stod(si.initial_value));
    }
    for (const auto& ci : c.command_interfaces) {
      command_component_.push_back(c.name);
      command_interface_.push_back(ci.name);
      command_names_.push_back(c.name + "/" + ci.name);
      command_values_.push_back(ci.initial_value.empty() ? kNaN : std::stod(ci.initial_value));
    }
  };
  for (const auto& j : info_.joints) collect(j);
  for (const auto& g : info_.gpios) collect(g);
  for (const auto& s : info_.sensors) collect(s);

  core_ = std::make_unique<can::Core>(cfg);
  const can::BindReport report = core_->bind(state_names_, command_names_);
  for (const auto& w : report.warnings) RCLCPP_WARN(kLogger, "%s", w.c_str());
  for (const auto& e : report.errors) RCLCPP_ERROR(kLogger, "%s", e.c_str());
  if (!report.errors.empty()) {
    RCLCPP_ERROR(kLogger, "the ros2_control block and can_signals.yaml disagree; fix one of them");
    return CallbackReturn::ERROR;
  }

  RCLCPP_INFO(kLogger, "initialised: %zu state / %zu command interfaces, competition=%s, can=%s",
              state_names_.size(), command_names_.size(), comp.c_str(), can_interface_.c_str());
  return CallbackReturn::SUCCESS;
}

// ros2_control stores POINTERS to our doubles, so the vectors must never be resized after this.
std::vector<hardware_interface::StateInterface> PayloadCanSystem::export_state_interfaces() {
  std::vector<hardware_interface::StateInterface> out;
  out.reserve(state_values_.size());
  for (size_t i = 0; i < state_values_.size(); ++i)
    out.emplace_back(state_component_[i], state_interface_[i], &state_values_[i]);
  return out;
}

std::vector<hardware_interface::CommandInterface> PayloadCanSystem::export_command_interfaces() {
  std::vector<hardware_interface::CommandInterface> out;
  out.reserve(command_values_.size());
  for (size_t i = 0; i < command_values_.size(); ++i)
    out.emplace_back(command_component_[i], command_interface_[i], &command_values_[i]);
  return out;
}

// =======================================================================================
// on_configure: open the CAN socket and start the receive thread.
// =======================================================================================
PayloadCanSystem::CallbackReturn PayloadCanSystem::on_configure(const rclcpp_lifecycle::State&) {
  std::string err;
  if (!socket_.open(can_interface_, &err)) {
    RCLCPP_ERROR(kLogger, "cannot open CAN interface '%s': %s  (for testing: sudo ip link add dev vcan0 type vcan && sudo ip link set up vcan0)",
                 can_interface_.c_str(), err.c_str());
    return CallbackReturn::ERROR;
  }
  running_ = true;
  rx_thread_ = std::thread(&PayloadCanSystem::rx_loop, this);
  RCLCPP_INFO(kLogger, "listening on %s", can_interface_.c_str());
  return CallbackReturn::SUCCESS;
}

void PayloadCanSystem::rx_loop() {
  can::Frame frame;
  while (running_) {
    switch (socket_.recv(&frame, 50)) {
      case can::RecvStatus::kFrame:    core_->on_rx(frame, can::Core::Clock::now()); break;
      case can::RecvStatus::kBusError: core_->on_bus_error(); break;
      case can::RecvStatus::kTimeout:  break;
      case can::RecvStatus::kClosed:   std::this_thread::sleep_for(std::chrono::milliseconds(20)); break;
    }
  }
}

void PayloadCanSystem::stop_rx_thread() {
  running_ = false;
  if (rx_thread_.joinable()) rx_thread_.join();
}

PayloadCanSystem::CallbackReturn PayloadCanSystem::on_activate(const rclcpp_lifecycle::State&) {
  RCLCPP_INFO(kLogger, "activated");
  return CallbackReturn::SUCCESS;
}

// =======================================================================================
// on_deactivate: leave the payload in a known state.
//   * stop-type commands only (drill 0 rpm, pumps 0, mixer off, LEDs off, spool stop);
//     we never command a position move while shutting down.
//   * optionally the table's KILL message (off by default: its exact effect must be agreed with electrical).
// =======================================================================================
PayloadCanSystem::CallbackReturn PayloadCanSystem::on_deactivate(const rclcpp_lifecycle::State&) {
  if (safe_state_on_deactivate_) {
    core_->send_safe_state(socket_);
    RCLCPP_INFO(kLogger, "safe-state commands sent");
  }
  if (kill_on_deactivate_) {
    uint64_t w = can::make_header(can::kDestPayload, can::kMsg_KILL);
    w = can::put_payload(w, 0, 1, 1);
    can::Frame f;
    f.id = can::kArbitrationId;
    can::from_word(f, w);
    socket_.send(f);
    RCLCPP_WARN(kLogger, "KILL sent on deactivate");
  }
  return CallbackReturn::SUCCESS;
}

PayloadCanSystem::CallbackReturn PayloadCanSystem::on_cleanup(const rclcpp_lifecycle::State&) {
  stop_rx_thread();
  socket_.close();
  return CallbackReturn::SUCCESS;
}

void PayloadCanSystem::log_core_events() {
  for (const auto& line : core_->take_events()) RCLCPP_WARN(kLogger, "%s", line.c_str());
}

// =======================================================================================
// The control loop.  Keep these two functions short and non-blocking.
// =======================================================================================
hardware_interface::return_type PayloadCanSystem::read(const rclcpp::Time&, const rclcpp::Duration&) {
  core_->read_states(state_values_.data(), can::Core::Clock::now());
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type PayloadCanSystem::write(const rclcpp::Time&, const rclcpp::Duration&) {
  core_->write_commands(command_values_.data(), state_values_.data(), can::Core::Clock::now(), socket_);
  log_core_events();     // allocates only when something noteworthy happened
  return hardware_interface::return_type::OK;
}

}  // namespace payload_hardware

PLUGINLIB_EXPORT_CLASS(payload_hardware::PayloadCanSystem, hardware_interface::SystemInterface)
