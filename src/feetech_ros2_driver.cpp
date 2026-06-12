#include <fmt/ranges.h>

#include <algorithm>
#include <cmath>
#include <feetech_driver/common.hpp>
#include <feetech_driver/communication_protocol.hpp>
#include <feetech_ros2_driver/feetech_ros2_driver.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <range/v3/range/conversion.hpp>
#include <range/v3/view/all.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <string_view>
#include <tuple>
#include <vector>

namespace {
// Clamp a commanded position to what the sign-magnitude goal register can encode (bit15 -> ±32767).
// Deliberately NOT clamped to a single turn (0..4095): that would defeat multi-turn mode. ±pi is
// only ±2048 ticks around center, far inside this bound, and the URDF already bounds the command —
// so this is a safety net that keeps encode_sign_magnitude from throwing out of the realtime loop.
constexpr int kMaxEncodableTick = (1 << SMS_STS_SIGN_BIT_POSITION) - 1;  // 32767
inline int to_commanded_tick(double rad) {
  return std::clamp(feetech_driver::from_radians(rad) + feetech_driver::kStsMidpoint, -kMaxEncodableTick,
                    kMaxEncodableTick);
}
}  // namespace

namespace feetech_ros2_driver {
#if HARDWARE_INTERFACE_VERSION_GTE(4, 34, 0)
CallbackReturn FeetechHardwareInterface::on_init(const hardware_interface::HardwareComponentInterfaceParams& params) {
  if (hardware_interface::SystemInterface::on_init(params) != CallbackReturn::SUCCESS) {
#else
CallbackReturn FeetechHardwareInterface::on_init(const hardware_interface::HardwareInfo& info) {
  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS) {
#endif
    return CallbackReturn::ERROR;
  }

  if (init_transport_() != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }

  // Optional servo-side motion profile (see header): <param name="goal_speed">/<param
  // name="goal_acceleration"> on the <hardware> block. Defaults keep the historical values, so
  // existing setups are byte-for-byte unchanged.
  for (auto [name, member] : {std::pair<const char*, int*>{"goal_speed", &goal_speed_},
                              std::pair<const char*, int*>{"goal_acceleration", &goal_acceleration_}}) {
    if (const auto it = info_.hardware_parameters.find(name); it != info_.hardware_parameters.end()) {
      try {
        *member = std::stoi(it->second);
      } catch (const std::exception&) {
        spdlog::error("FeetechHardwareInterface::on_init invalid {} '{}'", name, it->second);
        return CallbackReturn::ERROR;
      }
    }
  }
  if (goal_speed_ < 0 || goal_speed_ > 2400 || goal_acceleration_ < 0 || goal_acceleration_ > 255) {
    spdlog::error("FeetechHardwareInterface::on_init goal_speed {} (0-2400) / goal_acceleration {} (0-255) out of range",
                  goal_speed_, goal_acceleration_);
    return CallbackReturn::ERROR;
  }

  JointIdConfigMap yaml_by_id;
  if (load_yaml_config_and_warn_(yaml_by_id) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }

  if (configure_joints_(yaml_by_id) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }

  if (validate_model_series_() != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }

  log_firmware_versions_();

  return CallbackReturn::SUCCESS;
}

void FeetechHardwareInterface::log_firmware_versions_() {
  // Firmware 3.9 corrupts SYNC READ status packets (fixed in 3.10, huggingface/lerobot#1010) —
  // surface the per-servo version at startup so that failure mode is diagnosable, not guessed.
  for (const auto id : joint_ids_) {
    const auto fw = communication_protocol_->read_word(id, SMS_STS_FIRMWARE_VER_L);
    if (!fw) {
      spdlog::warn("FeetechHardwareInterface: read firmware version (id={}) -> {}", id, fw.error());
      continue;
    }
    const int major = *fw & 0xFF;
    const int minor = (*fw >> 8) & 0xFF;
    if (major < 3 || (major == 3 && minor < 10)) {
      spdlog::warn("Servo id={} firmware {}.{} — SYNC READ is unreliable on firmware < 3.10", id, major, minor);
    } else {
      spdlog::info("Servo id={} firmware {}.{}", id, major, minor);
    }
  }
}

CallbackReturn FeetechHardwareInterface::init_transport_() {
  const auto usb_port_it = info_.hardware_parameters.find("usb_port");
  if (usb_port_it == info_.hardware_parameters.end()) {
    spdlog::error(
        "FeetechHardwareInterface::init_transport_ Hardware parameter [usb_port] not found! "
        "Make sure to have <param name=\"usb_port\">/dev/XXXX</param>");
    return CallbackReturn::ERROR;
  }

  auto serial_port = std::make_unique<feetech_driver::SerialPort>(usb_port_it->second);

  if (const auto result = serial_port->configure(); !result) {
    spdlog::error("FeetechHardwareInterface::init_transport_ -> {}", result.error());
    return CallbackReturn::ERROR;
  }

  communication_protocol_ = std::make_unique<feetech_driver::CommunicationProtocol>(std::move(serial_port));

  return CallbackReturn::SUCCESS;
}

// Optional YAML overlay — if not provided, URDF params are used as-is.
// Builds an ID-keyed map: URDF id is the hardware identity, YAML name is just a label.
CallbackReturn FeetechHardwareInterface::load_yaml_config_and_warn_(JointIdConfigMap& out_yaml) {
  out_yaml.clear();

  const auto cfg_it = info_.hardware_parameters.find("joint_config_file");
  if (cfg_it == info_.hardware_parameters.end() || cfg_it->second.empty()) {
    return CallbackReturn::SUCCESS;  // no YAML — fall back to URDF params only
  }

  auto loaded = load_joint_config(cfg_it->second);
  if (!loaded) {
    return CallbackReturn::ERROR;
  }

  // Re-key by servo id
  for (auto& [name, params] : *loaded) {
    auto it = params.find("id");
    if (it == params.end()) {
      spdlog::error("YAML joint '{}' has no 'id' parameter", name);
      return CallbackReturn::ERROR;
    }
    int id = std::stoi(it->second);
    if (!out_yaml.emplace(id, std::move(params)).second) {
      spdlog::error("Duplicate servo id {} in YAML (joint '{}')", id, name);
      return CallbackReturn::ERROR;
    }
  }

  // Warn: URDF ids missing in YAML
  for (const auto& j : info_.joints) {
    auto id_it = j.parameters.find("id");
    if (id_it != j.parameters.end() && out_yaml.find(std::stoi(id_it->second)) == out_yaml.end()) {
      spdlog::warn("URDF joint '{}' (id={}) has no YAML entry (using URDF defaults)", j.name, id_it->second);
    }
  }

  return CallbackReturn::SUCCESS;
}

CallbackReturn FeetechHardwareInterface::configure_joints_(const JointIdConfigMap& yaml_by_id) {
  joint_ids_.assign(info_.joints.size(), 0);

  for (size_t i = 0; i < info_.joints.size(); ++i) {
    const auto& joint = info_.joints[i];
    const std::string& joint_name = joint.name;

    // Required: id (from URDF — hardware identity)
    const auto urdf_id_it = joint.parameters.find("id");
    if (urdf_id_it == joint.parameters.end()) {
      spdlog::error("Joint '{}' does not have required 'id' parameter", joint_name);
      return CallbackReturn::ERROR;
    }
    const int id = std::stoi(urdf_id_it->second);
    joint_ids_[i] = static_cast<uint8_t>(id);

    // Merge YAML config (looked up by servo id) over URDF params
    JointParams merged_params;
    if (auto it = yaml_by_id.find(id); it != yaml_by_id.end()) {
      merged_params = merge_joint_params(it->second, joint.parameters);
    } else {
      merged_params = JointParams(joint.parameters.begin(), joint.parameters.end());
    }

    if (merged_params.find("offset") != merged_params.end()) {
      spdlog::warn("Joint '{}': 'offset' param is deprecated and ignored — use 'homing_offset' instead", joint_name);
    }

    // Multi-turn opt-in (e.g. a free-spinning wrist): the driver sets MIN==MAX angle-limit = 0 below
    // to select absolute multi-turn mode, which removes the single-turn encoder-seam wall. The
    // range_min/range_max window is meaningless once that wall is gone, so it is skipped.
    const bool multi_turn = [&] {
      const auto it = merged_params.find("multi_turn");
      return it != merged_params.end() && (it->second == "true" || it->second == "1");
    }();

    // Disable torque and unlock EPROM before writing parameters
    if (const auto result = communication_protocol_->disable_torque(joint_ids_[i]); !result) {
      spdlog::error("FeetechHardwareInterface::configure_joints_ disable_torque -> {}", result.error());
      return CallbackReturn::ERROR;
    }

    // Single-byte parameters (0-255)
    for (const auto& [parameter_name, address] : {std::pair{"p_coefficient", SMS_STS_P_COEF},
                                                  {"d_coefficient", SMS_STS_D_COEF},
                                                  {"i_coefficient", SMS_STS_I_COEF},
                                                  {"overload_torque", SMS_STS_OVERLOAD_TORQUE},
                                                  {"return_delay_time", SMS_STS_RETURN_DELAY},
                                                  {"acceleration", SMS_STS_ACC}}) {
      if (const auto param_it = merged_params.find(parameter_name); param_it != merged_params.end()) {
        const auto result = communication_protocol_->write(
            joint_ids_[i], address, std::experimental::make_array(static_cast<uint8_t>(std::stoi(param_it->second))));
        if (!result) {
          spdlog::error("FeetechHardwareInterface::configure_joints_ -> {}", result.error());
          return CallbackReturn::ERROR;
        }
      }
    }

    // Two-byte unsigned parameters
    for (const auto& [parameter_name, address] : {std::pair{"range_min", SMS_STS_MIN_ANGLE_LIMIT_L},
                                                  {"range_max", SMS_STS_MAX_ANGLE_LIMIT_L},
                                                  {"max_torque_limit", SMS_STS_MAX_TORQUE_L},
                                                  {"protection_current", SMS_STS_PROTECTION_CURRENT_L}}) {
      // A multi-turn joint has no single-turn angle-limit window; MIN/MAX are forced to 0 below.
      if (multi_turn && (std::string_view(parameter_name) == "range_min" ||
                         std::string_view(parameter_name) == "range_max")) {
        continue;
      }
      if (const auto param_it = merged_params.find(parameter_name); param_it != merged_params.end()) {
        std::array<uint8_t, 2> buf{};
        feetech_driver::to_sts(&buf[0], &buf[1], std::stoi(param_it->second));
        const auto result = communication_protocol_->write(joint_ids_[i], address, buf);
        if (!result) {
          spdlog::error("FeetechHardwareInterface::configure_joints_ -> {}", result.error());
          return CallbackReturn::ERROR;
        }
      }
    }

    // Multi-turn position mode: MIN==MAX==0 (datasheet §7-13) selects absolute multi-turn (±7 rev),
    // removing the single-turn encoder-seam wall that otherwise jams a free-spinning joint. MODE
    // reg 33 is forced to 0 (position) so a servo left in wheel/step mode is recovered. EEPROM
    // writes are skipped when the register already holds the target value (write-cycle wear).
    if (multi_turn) {
      for (const auto address : {SMS_STS_MIN_ANGLE_LIMIT_L, SMS_STS_MAX_ANGLE_LIMIT_L}) {
        std::array<uint8_t, 2> current_buf{};
        if (communication_protocol_->read(joint_ids_[i], address, &current_buf) && current_buf[0] == 0 &&
            current_buf[1] == 0) {
          continue;  // already 0 — don't burn an EEPROM write
        }
        const std::array<uint8_t, 2> buf{0, 0};
        if (const auto result = communication_protocol_->write(joint_ids_[i], address, buf); !result) {
          spdlog::error("FeetechHardwareInterface::configure_joints_ multi_turn angle-limit -> {}", result.error());
          return CallbackReturn::ERROR;
        }
      }
      std::array<uint8_t, 1> mode_buf{};
      const bool already_position =
          communication_protocol_->read(joint_ids_[i], SMS_STS_MODE, &mode_buf) && mode_buf[0] == 0;
      if (!already_position) {
        if (const auto result =
                communication_protocol_->set_mode(joint_ids_[i], feetech_driver::OperationMode::kPosition);
            !result) {
          spdlog::error("FeetechHardwareInterface::configure_joints_ set_mode -> {}", result.error());
          return CallbackReturn::ERROR;
        }
      }
    }

    // Two-byte signed parameters (sign-magnitude encoding)
    for (const auto& [parameter_name, address, sign_bit] :
         {std::tuple{"homing_offset", SMS_STS_OFS_L, SMS_STS_SIGN_BIT_HOMING_OFFSET}}) {
      if (const auto param_it = merged_params.find(parameter_name); param_it != merged_params.end()) {
        const int target = std::stoi(param_it->second);
        // Read back what EEPROM currently holds before overwriting: a large
        // mismatch means something else rewrote the servo since our last run
        // (e.g. external calibration tooling with a different/stale file) and
        // the arm has been living in a different zero until now. Warn-only —
        // the config value is about to be (re)applied either way.
        std::array<uint8_t, 2> current_buf{};
        if (communication_protocol_->read(joint_ids_[i], address, &current_buf)) {
          const int current = feetech_driver::decode_sign_magnitude(
              feetech_driver::from_sts(
                  feetech_driver::WordBytes{.low = current_buf[0], .high = current_buf[1]}),
              sign_bit);
          if (current == target) {
            continue;  // EEPROM already holds the target — don't burn a write cycle
          }
          if (std::abs(current - target) > 20) {
            spdlog::warn(
                "Joint '{}': EEPROM {} was {} but config says {} (delta {} ticks ~ {:.1f} deg) — "
                "external tooling rewrote it since the last run? Writing the config value.",
                joint_name, parameter_name, current, target, current - target,
                std::abs(current - target) * 360.0 / 4096.0);
          }
        }
        std::array<uint8_t, 2> buf{};
        const int value = feetech_driver::encode_sign_magnitude(target, sign_bit);
        feetech_driver::to_sts(&buf[0], &buf[1], value);
        const auto result = communication_protocol_->write(joint_ids_[i], address, buf);
        if (!result) {
          spdlog::error("FeetechHardwareInterface::configure_joints_ -> {}", result.error());
          return CallbackReturn::ERROR;
        }
      }
    }

    // Lock EPROM after writing parameters (for all joints)
    if (const auto result = communication_protocol_->lock_eprom(joint_ids_[i]); !result) {
      spdlog::error("FeetechHardwareInterface::configure_joints_ lock_eprom -> {}", result.error());
      return CallbackReturn::ERROR;
    }

    // Only enable torque for joints with command interfaces (Follower Arm)
    if (!joint.command_interfaces.empty()) {
      // Sync the volatile Goal_Position register to the present position BEFORE
      // enabling torque. The goal survives across sessions while the bus stays
      // powered; if the homing offset was just rewritten the stale goal is
      // reinterpreted in the new frame and the servo JUMPS on torque-on (seen on
      // wrist_roll: ~49° lunge into a mechanical jam + overload cutoff).
      const auto present = communication_protocol_->read_position(joint_ids_[i]);
      if (!present) {
        spdlog::error("FeetechHardwareInterface::configure_joints_ read_position -> {}", present.error());
        return CallbackReturn::ERROR;
      }
      if (const auto result = communication_protocol_->write_position(joint_ids_[i], *present, 0, 0); !result) {
        spdlog::error("FeetechHardwareInterface::configure_joints_ goal sync -> {}", result.error());
        return CallbackReturn::ERROR;
      }
      if (const auto result = communication_protocol_->set_torque(joint_ids_[i], true); !result) {
        spdlog::error("FeetechHardwareInterface::configure_joints_ set_torque -> {}", result.error());
        return CallbackReturn::ERROR;
      }
    }
  }

  return CallbackReturn::SUCCESS;
}

CallbackReturn FeetechHardwareInterface::validate_model_series_() {
  const auto joint_model_series = joint_ids_ | ranges::views::transform([&](const auto id) {
                                    return communication_protocol_->read_model_number(id)
                                        .and_then(feetech_driver::get_model_name)
                                        .and_then(feetech_driver::get_model_series);
                                  });

  if (std::ranges::any_of(joint_model_series, [](const auto& series) { return !series.has_value(); })) {
    spdlog::error("FeetechHardware::validate_model_series_ [One of the joints has an error]. Input: {}",
                  ranges::views::zip(joint_ids_, joint_model_series));
    return CallbackReturn::ERROR;
  }

  const auto js = joint_model_series | ranges::views::transform([](const auto& series) { return series.value(); });

  if (ranges::any_of(js, [](const auto& series) { return series != feetech_driver::ModelSeries::kSts; })) {
    spdlog::error("FeetechHardware::validate_model_series_ [Only STS series is supported]. Input (id, series): {}",
                  ranges::views::zip(joint_ids_, js));
    return CallbackReturn::ERROR;
  }

  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> FeetechHardwareInterface::export_state_interfaces() {
  std::vector<hardware_interface::StateInterface> state_interfaces;
  state_hw_positions_.resize(info_.joints.size(), 0.0);
  state_hw_velocities_.resize(info_.joints.size(), 0.0);
  state_hw_efforts_.resize(info_.joints.size(), 0.0);
  for (uint i = 0; i < info_.joints.size(); i++) {
    state_interfaces.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_POSITION, &state_hw_positions_[i]);
    state_interfaces.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &state_hw_velocities_[i]);
    // Effort = Present_Load as a signed fraction of stall torque [-1, 1] (NOT N*m; see header).
    state_interfaces.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &state_hw_efforts_[i]);
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> FeetechHardwareInterface::export_command_interfaces() {
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  hw_positions_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  for (uint i = 0; i < info_.joints.size(); i++) {
    if (!info_.joints[i].command_interfaces.empty()) {
      command_interfaces.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_positions_[i]);
    }
  }

  return command_interfaces;
}

hardware_interface::return_type FeetechHardwareInterface::read(const rclcpp::Time& /* time */,
                                                               const rclcpp::Duration& /* period */) {
  // 8 contiguous bytes per servo (regs 56-63): position(2) + speed(2) + load(2) + voltage(1) +
  // temperature(1) — load/voltage/temp ride the SAME bus transaction as the position read
  // (~4 extra bytes per servo per cycle at 1 Mbps, negligible).
  std::vector<std::array<uint8_t, 8>> data;
  data.reserve(joint_ids_.size());
  std::vector<uint8_t> statuses;
  if (auto result = communication_protocol_->sync_read(joint_ids_, SMS_STS_PRESENT_POSITION_L, &data, &statuses);
      !result) {
    // Transient bus hiccups (a late/lost/corrupt response under load) self-heal: sync_read drops
    // stale bytes before each request, so one bad packet costs one cycle. Hold the last state and
    // retry next cycle — returning ERROR would latch the component until a node restart.
    ++consecutive_read_failures_;
    if (consecutive_read_failures_ <= 3 || consecutive_read_failures_ % 100 == 0) {
      spdlog::warn("FeetechHardwareInterface::read ({} consecutive failures) -> {}", consecutive_read_failures_,
                   result.error());
    }
    // Sustained staleness escalates to ERROR: the exported joint state has been FROZEN at the
    // last good read this whole time (deliberate anti-latch hold, see below) and every consumer —
    // path follower, behaviour, TF via robot_state_publisher — is acting on a stale arm pose
    // without any way to know. The log line is currently the ONLY staleness signal downstream.
    if (consecutive_read_failures_ % kStaleEscalateEvery == 0) {
      spdlog::error(
          "FeetechHardwareInterface::read: joint states STALE for {} cycles (~{:.1f} s) — bus "
          "unresponsive, consumers are tracking a frozen arm pose",
          consecutive_read_failures_, consecutive_read_failures_ * 0.01);
    }
    // An outage this long can be a brownout-reset servo: its volatile torque-enable register
    // cleared, the joint is limp until rewritten. Heal once the bus answers again.
    if (consecutive_read_failures_ == kTorqueRecoveryStreak) {
      needs_torque_recovery_ = true;
    }
    // A persistent outage can also be a wedged tty / USB hiccup: periodically reopen the port.
    if (consecutive_read_failures_ % kReconnectEvery == 0) {
      spdlog::error("FeetechHardwareInterface::read: bus unresponsive for {} cycles, reopening serial port",
                    consecutive_read_failures_);
      if (const auto reconnect_result = communication_protocol_->reconnect(); !reconnect_result) {
        spdlog::error("FeetechHardwareInterface::read reconnect -> {}", reconnect_result.error());
      }
    }
    return hardware_interface::return_type::OK;
  }
  if (consecutive_read_failures_ > 0) {
    spdlog::info("FeetechHardwareInterface::read: bus recovered after {} failed cycles", consecutive_read_failures_);
    consecutive_read_failures_ = 0;
  }
  if (needs_torque_recovery_) {
    recover_torque_();
  }
  report_servo_faults_(statuses);
  ranges::for_each(data | ranges::views::enumerate, [&](const auto& values) {
    const auto& [index, readings] = values;
    // Present position is sign-magnitude (BIT15 = sign). Single-turn keeps it in 0..4095 (sign
    // clear) so decoding is a no-op there; in multi-turn it goes negative just below -pi, where
    // skipping the decode would misread ~+47 rad and lunge. Mirrors the velocity decode below.
    state_hw_positions_[index] = feetech_driver::to_radians(
        feetech_driver::decode_sign_magnitude(
            feetech_driver::from_sts(feetech_driver::WordBytes{.low = readings[0], .high = readings[1]}),
            SMS_STS_SIGN_BIT_POSITION) -
        feetech_driver::kStsMidpoint);
    // Present speed is sign-magnitude (BIT15 = direction) — without decoding, reverse rotation
    // reads as ~+50 rad/s instead of negative.
    state_hw_velocities_[index] = feetech_driver::to_radians(feetech_driver::decode_sign_magnitude(
        feetech_driver::from_sts(feetech_driver::WordBytes{.low = readings[2], .high = readings[3]}),
        SMS_STS_SIGN_BIT_VELOCITY));
    // Present load: sign-magnitude (BIT10 = direction), magnitude in 0.1% of stall torque ->
    // exported as the effort state interface, a signed FRACTION of stall torque [-1, 1]. This is
    // the gravity/stall observability signal (joint_state_broadcaster publishes it for free).
    state_hw_efforts_[index] =
        static_cast<double>(feetech_driver::decode_sign_magnitude(
            feetech_driver::from_sts(feetech_driver::WordBytes{.low = readings[4], .high = readings[5]}),
            SMS_STS_SIGN_BIT_LOAD)) /
        1000.0;
    report_temperature_voltage_(index, readings[6], readings[7]);
  });
  return hardware_interface::return_type::OK;
}

void FeetechHardwareInterface::report_temperature_voltage_(std::size_t index, uint8_t voltage_raw,
                                                           uint8_t temperature_c) {
  // Early warning BEFORE the firmware protections trip (over-temp cutoff ~70 C, voltage window in
  // EEPROM): edge-triggered per level so a hot afternoon doesn't spam the log at 100 Hz.
  last_temp_level_.resize(joint_ids_.size(), 0);
  last_volt_level_.resize(joint_ids_.size(), 0);
  const double volts = 0.1 * voltage_raw;
  const uint8_t temp_level = (temperature_c >= 65) ? 2 : (temperature_c >= 55) ? 1 : 0;
  if (temp_level != last_temp_level_[index]) {
    if (temp_level == 2) {
      spdlog::error("Servo id={} ('{}') temperature {} C — overheat protection imminent (~70 C)",
                    joint_ids_[index], info_.joints[index].name, temperature_c);
    } else if (temp_level == 1) {
      spdlog::warn("Servo id={} ('{}') temperature {} C — running hot", joint_ids_[index],
                   info_.joints[index].name, temperature_c);
    } else {
      spdlog::info("Servo id={} ('{}') temperature back to {} C", joint_ids_[index],
                   info_.joints[index].name, temperature_c);
    }
    last_temp_level_[index] = temp_level;
  }
  const uint8_t volt_level = (volts < 6.0 || volts > 8.6) ? 1 : 0;
  if (volt_level != last_volt_level_[index]) {
    if (volt_level == 1) {
      spdlog::warn("Servo id={} ('{}') bus voltage {:.1f} V outside [6.0, 8.6] (7.4 V nominal)",
                   joint_ids_[index], info_.joints[index].name, volts);
    } else {
      spdlog::info("Servo id={} ('{}') bus voltage back to {:.1f} V", joint_ids_[index],
                   info_.joints[index].name, volts);
    }
    last_volt_level_[index] = volt_level;
  }
}

void FeetechHardwareInterface::report_servo_faults_(const std::vector<uint8_t>& statuses) {
  // Every response packet carries the servo's working-status byte for free (same bit layout as
  // the Servo Status register, addr 65). Bit 5 is the overload protection: when it trips, the
  // servo folds output to the "protection torque" (default 20% of max) — the joint sags but
  // still resists, distinguishable here from a brownout (comm outage + cleared torque switch).
  static constexpr std::array<const char*, 6> kFaultBits = {"voltage",     "sensor", "overheat",
                                                            "overcurrent", "angle",  "overload"};
  last_servo_statuses_.resize(statuses.size(), 0);
  for (size_t i = 0; i < statuses.size(); ++i) {
    if (statuses[i] == last_servo_statuses_[i]) {
      continue;
    }
    if (statuses[i] != 0) {
      std::string faults;
      for (size_t bit = 0; bit < kFaultBits.size(); ++bit) {
        if (statuses[i] & (1u << bit)) {
          faults += faults.empty() ? "" : "+";
          faults += kFaultBits[bit];
        }
      }
      spdlog::warn("Servo id={} ('{}') FAULT [{}] (status=0x{:02x})", joint_ids_[i], info_.joints[i].name, faults,
                   statuses[i]);
    } else {
      spdlog::info("Servo id={} ('{}') fault cleared", joint_ids_[i], info_.joints[i].name);
    }
    last_servo_statuses_[i] = statuses[i];
  }
}

hardware_interface::return_type FeetechHardwareInterface::write(const rclcpp::Time& /* time */,
                                                                const rclcpp::Duration& /* period */) {
  // Create vectors only for joints that have command interfaces
  std::vector<uint8_t> commanded_joint_ids;
  std::vector<int> commanded_positions;
  std::vector<int> commanded_speeds;
  std::vector<int> commanded_accelerations;

  for (uint i = 0; i < info_.joints.size(); i++) {
    // Only include joints with command interfaces
    if (!info_.joints[i].command_interfaces.empty()) {
      // hw_positions_ starts as NaN until the controller writes a setpoint; from_radians(NaN) ->
      // static_cast<int>(NaN) is UB and would emit a garbage goal. Skip until a real value arrives.
      if (std::isnan(hw_positions_[i])) {
        continue;
      }
      commanded_joint_ids.push_back(joint_ids_[i]);
      commanded_positions.push_back(to_commanded_tick(hw_positions_[i]));
      commanded_speeds.push_back(goal_speed_);
      commanded_accelerations.push_back(goal_acceleration_);
    }
  }

  // Only send commands if there are joints to command
  if (!commanded_joint_ids.empty()) {
    const auto write_result = communication_protocol_->sync_write_position(
        commanded_joint_ids, commanded_positions, commanded_speeds, commanded_accelerations);
    if (!write_result) {
      // Same policy as read(): a transient failure must not latch the component (sync_write is
      // fire-and-forget broadcast, so a failure here is port-level). Retry next cycle.
      ++consecutive_write_failures_;
      if (consecutive_write_failures_ <= 3 || consecutive_write_failures_ % 100 == 0) {
        spdlog::warn("FeetechHardwareInterface::write ({} consecutive failures) -> {}", consecutive_write_failures_,
                     write_result.error());
      }
      return hardware_interface::return_type::OK;
    }
    consecutive_write_failures_ = 0;
  }

  return hardware_interface::return_type::OK;
}

void FeetechHardwareInterface::recover_torque_() {
  // After a comm outage a servo may have brownout-reset: EEPROM config survives, but the volatile
  // torque-enable register cleared — the joint is limp until rewritten. Same goal-sync-then-
  // torque-on dance as configure_joints_ (writing torque-on with a stale Goal_Position would
  // lunge). No-op for servos that kept power. Any failure: keep the flag, retry next cycle.
  for (size_t i = 0; i < info_.joints.size(); ++i) {
    if (info_.joints[i].command_interfaces.empty()) {
      continue;
    }
    const auto present = communication_protocol_->read_position(joint_ids_[i]);
    if (!present) {
      spdlog::warn("FeetechHardwareInterface::recover_torque_ read_position(id={}) -> {}", joint_ids_[i],
                   present.error());
      return;
    }
    if (const auto result = communication_protocol_->write_position(joint_ids_[i], *present, 0, 0); !result) {
      spdlog::warn("FeetechHardwareInterface::recover_torque_ goal sync(id={}) -> {}", joint_ids_[i], result.error());
      return;
    }
    if (const auto result = communication_protocol_->set_torque(joint_ids_[i], true); !result) {
      spdlog::warn("FeetechHardwareInterface::recover_torque_ set_torque(id={}) -> {}", joint_ids_[i], result.error());
      return;
    }
  }
  needs_torque_recovery_ = false;
  spdlog::info("FeetechHardwareInterface: torque re-enabled on all commanded joints after comm outage");
}

CallbackReturn FeetechHardwareInterface::on_activate(const rclcpp_lifecycle::State& /* previous_state */) {
  // Time/Duration are not used
  read(rclcpp::Time{}, rclcpp::Duration::from_seconds(0));
  // Set the initial command to current joint positions
  hw_positions_ = state_hw_positions_;
  return CallbackReturn::SUCCESS;
}

CallbackReturn FeetechHardwareInterface::on_deactivate(const rclcpp_lifecycle::State& /* previous_state */) {
  // all joints torque off
  const auto torque_disable_parameters =
      std::vector(joint_ids_.size(), std::experimental::make_array(static_cast<uint8_t>(0)));
  if (const auto result =
          communication_protocol_->sync_write(joint_ids_, SMS_STS_TORQUE_ENABLE, torque_disable_parameters);
      !result) {
    spdlog::error("FeetechHardwareInterface::on_deactivate -> {}", result.error());
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

}  // namespace feetech_ros2_driver

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(feetech_ros2_driver::FeetechHardwareInterface, hardware_interface::SystemInterface)
