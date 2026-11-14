// payload_can_system.hpp -- the ros2_control "hardware interface" for the science payload.
//
// WHAT THIS CLASS IS
//   ros2_control calls read() and write() on every control cycle.  This class is the piece
//   that sits between those calls and the CAN bus.  It replaces the old separate "CAN Node"
//   and "Payload State" node: telemetry arrives as CAN frames and is exposed as state
//   interfaces (so the standard state broadcaster publishes them), and command interfaces
//   written by controllers become CAN frames.
//
// WHAT IT DELIBERATELY DOES NOT DO
//   * It contains no mission logic.
//   * It does not re-send a command every cycle ("hold"): the board holds by itself.
//   * It does not enforce physical limits (limit switches, stall protection): the firmware does.
//
// All real logic lives in payload_hardware::can::Core (ROS-free and unit tested).  This file
// only translates between the ros2_control API and Core.
#pragma once

#include <atomic>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "payload_hardware/can_core.hpp"
#include "payload_hardware/socketcan.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace payload_hardware {

class PayloadCanSystem : public hardware_interface::SystemInterface {
 public:
  RCLCPP_SHARED_PTR_DEFINITIONS(PayloadCanSystem)

  ~PayloadCanSystem() override;

  // ---- lifecycle ----
  CallbackReturn on_init(const hardware_interface::HardwareInfo& info) override;
  CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State& previous_state) override;

  // ---- what we offer to controllers ----
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  // ---- the control loop ----
  hardware_interface::return_type read(const rclcpp::Time& time, const rclcpp::Duration& period) override;
  hardware_interface::return_type write(const rclcpp::Time& time, const rclcpp::Duration& period) override;

 private:
  void rx_loop();                 // background thread: SocketCAN -> Core::on_rx
  void stop_rx_thread();
  void log_core_events();

  // Parameters from the <hardware> block of the ros2_control xacro.
  std::string can_interface_ = "can0";
  bool safe_state_on_deactivate_ = true;
  bool kill_on_deactivate_ = false;

  // One slot per declared interface.  ros2_control holds raw pointers into these vectors,
  // so they are sized once in on_init and never resized afterwards.
  std::vector<std::string> state_component_, state_interface_, state_names_;
  std::vector<std::string> command_component_, command_interface_, command_names_;
  std::vector<double> state_values_, command_values_;

  std::unique_ptr<can::Core> core_;
  can::SocketCan socket_;
  std::thread rx_thread_;
  std::atomic<bool> running_{false};
};

}  // namespace payload_hardware
