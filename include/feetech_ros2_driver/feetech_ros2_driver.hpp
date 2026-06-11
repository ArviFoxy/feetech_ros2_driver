#pragma once

#include <feetech_driver/communication_protocol.hpp>
#include <feetech_driver/serial_port.hpp>
#include <hardware_interface/handle.hpp>
#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/system_interface.hpp>
#include <map>
#include <rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include <vector>

#if __has_include(<hardware_interface/hardware_interface/version.h>)
#include <hardware_interface/hardware_interface/version.h>
#else
#include <hardware_interface/version.h>
#endif

#include "feetech_ros2_driver/joint_config.hpp"

namespace feetech_ros2_driver {

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class FeetechHardwareInterface : public hardware_interface::SystemInterface {
 public:
#if HARDWARE_INTERFACE_VERSION_GTE(4, 34, 0)
  CallbackReturn on_init(const hardware_interface::HardwareComponentInterfaceParams& params) override;
#else
  CallbackReturn on_init(const hardware_interface::HardwareInfo& info) override;
#endif

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::return_type read(const rclcpp::Time& time, const rclcpp::Duration& period) override;

  hardware_interface::return_type write(const rclcpp::Time& time, const rclcpp::Duration& period) override;

  CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

 private:
  std::unique_ptr<feetech_driver::CommunicationProtocol> communication_protocol_;

  std::vector<double> hw_positions_;
  std::vector<double> state_hw_positions_;
  std::vector<double> state_hw_velocities_;
  std::vector<uint8_t> previous_hw_positions_;

  std::vector<uint8_t> joint_ids_;

  CallbackReturn init_transport_();
  CallbackReturn load_yaml_config_and_warn_(JointIdConfigMap& out_yaml);
  CallbackReturn configure_joints_(const JointIdConfigMap& yaml_by_id);
  CallbackReturn validate_model_series_();

  // Comm self-healing (see read()/write()): a transient bus error must NEVER surface as
  // return_type::ERROR — ros2_control latches the whole component on the first ERROR and only a
  // node restart revives it (observed live: one 5 ms read timeout froze the arm permanently).
  static constexpr int kTorqueRecoveryStreak = 10;  // outage cycles before assuming brownout-reset
  static constexpr int kReconnectEvery = 200;       // outage cycles between port-reopen attempts
  int consecutive_read_failures_ = 0;
  int consecutive_write_failures_ = 0;
  bool needs_torque_recovery_ = false;
  void recover_torque_();

  // Per-servo working-status byte from the last sync_read (0 = healthy); edge-triggered fault
  // logging so a tripped protection (overload/overheat/...) is visible the cycle it happens.
  std::vector<uint8_t> last_servo_statuses_;
  void report_servo_faults_(const std::vector<uint8_t>& statuses);
  void log_firmware_versions_();
};
}  // namespace feetech_ros2_driver
