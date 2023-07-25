#include "diffdrive_arduino/diffbot_system.hpp"

#include "wheels/wheel.hpp"

namespace diffdrive_arduino
{
  class DiffDriveArduinoHardware : public hardware_interface::SystemInterface
  {
    public:
      hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info);
      std::vector<hardware_interface::StateInterface> export_state_interfaces();
      std::vector<hardware_interface::CommandInterface> export_command_interfaces();
      hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & /*previous_state*/);
      hardware_interface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State & /*previous_state*/);
      hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & /*previous_state*/);
      hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/);
      hardware_interface::return_type read(const rclcpp::Time & /*time*/, const rclcpp::Duration & period);
      hardware_interface::return_type write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/);

    private:
      std::unique_ptr<wheels::Wheel> left_back_wheel_;
      std::unique_ptr<wheels::Wheel> right_back_wheel_;
      std::unique_ptr<wheels::Wheel> left_front_wheel_;
      std::unique_ptr<wheels::Wheel> right_front_wheel_;

      struct Config
      {
        std::string left_back_wheel_name;
        std::string right_back_wheel_name;
        std::string left_front_wheel_name;
        std::string right_front_wheel_name;
        double loop_rate;
        std::string device;
        int baud_rate;
        int timeout_ms;
        int enc_counts_per_rev;
        int pid_p;
        int pid_d;
        int pid_i;
        int pid_o;
      } cfg_;

      // Motor control variables
      int motor_counts_per_loop_;
      double wheel_radius_;

      // Rest of the class variables...

      // Helper functions...
      void setupWheels(const hardware_interface::HardwareInfo &info);
      void calculateMotorCounts();

  };

  hardware_interface::CallbackReturn DiffDriveArduinoHardware::on_init(const hardware_interface::HardwareInfo & info)
  {
    if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS)
    {
      return hardware_interface::CallbackReturn::ERROR;
    }

    // Initialize configuration parameters
    cfg_.left_back_wheel_name = info_.hardware_parameters["left_back_wheel_name"];
    cfg_.right_back_wheel_name = info_.hardware_parameters["right_back_wheel_name"];
    cfg_.left_front_wheel_name = info_.hardware_parameters["left_front_wheel_name"];
    cfg_.right_front_wheel_name = info_.hardware_parameters["right_front_wheel_name"];
    cfg_.loop_rate = std::stof(info_.hardware_parameters["loop_rate"]);
    cfg_.device = info_.hardware_parameters["device"];
    cfg_.baud_rate = std::stoi(info_.hardware_parameters["baud_rate"]);
    cfg_.timeout_ms = std::stoi(info_.hardware_parameters["timeout_ms"]);
    cfg_.enc_counts_per_rev = std::stoi(info_.hardware_parameters["enc_counts_per_rev"]);

    if (info_.hardware_parameters.count("pid_p") > 0)
    {
      cfg_.pid_p = std::stoi(info_.hardware_parameters["pid_p"]);
      cfg_.pid_d = std::stoi(info_.hardware_parameters["pid_d"]);
      cfg_.pid_i = std::stoi(info_.hardware_parameters["pid_i"]);
      cfg_.pid_o = std::stoi(info_.hardware_parameters["pid_o"]);
    }
    else
    {
      RCLCPP_INFO(rclcpp::get_logger("DiffDriveArduinoHardware"), "PID values not supplied, using defaults.");
    }

    // Initialize wheels
    setupWheels(info);

    // Initialize motor control variables
    wheel_radius_ = 0.0; // Set the actual wheel radius
    calculateMotorCounts();

    // Rest of the initialization code...

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  std::vector<hardware_interface::StateInterface> DiffDriveArduinoHardware::export_state_interfaces()
  {
    std::vector<hardware_interface::StateInterface> state_interfaces;

    state_interfaces.emplace_back(hardware_interface::StateInterface(
      left_back_wheel_->name, hardware_interface::HW_IF_POSITION, &left_back_wheel_->pos));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      left_back_wheel_->name, hardware_interface::HW_IF_VELOCITY, &left_back_wheel_->vel));

    state_interfaces.emplace_back(hardware_interface::StateInterface(
      right_back_wheel_->name, hardware_interface::HW_IF_POSITION, &right_back_wheel_->pos));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      right_back_wheel_->name, hardware_interface::HW_IF_VELOCITY, &right_back_wheel_->vel));

    state_interfaces.emplace_back(hardware_interface::StateInterface(
      left_front_wheel_->name, hardware_interface::HW_IF_POSITION, &left_front_wheel_->pos));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      left_front_wheel_->name, hardware_interface::HW_IF_VELOCITY, &left_front_wheel_->vel));

    state_interfaces.emplace_back(hardware_interface::StateInterface(
      right_front_wheel_->name, hardware_interface::HW_IF_POSITION, &right_front_wheel_->pos));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      right_front_wheel_->name, hardware_interface::HW_IF_VELOCITY, &right_front_wheel_->vel));

    return state_interfaces;
  }

  std::vector<hardware_interface::CommandInterface> DiffDriveArduinoHardware::export_command_interfaces()
  {
    std::vector<hardware_interface::CommandInterface> command_interfaces;

    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      left_back_wheel_->name, hardware_interface::HW_IF_VELOCITY, &left_back_wheel_->cmd));

    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      right_back_wheel_->name, hardware_interface::HW_IF_VELOCITY, &right_back_wheel_->cmd));

    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      left_front_wheel_->name, hardware_interface::HW_IF_VELOCITY, &left_front_wheel_->cmd));

    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      right_front_wheel_->name, hardware_interface::HW_IF_VELOCITY, &right_front_wheel_->cmd));

    return command_interfaces;
  }

  hardware_interface::CallbackReturn DiffDriveArduinoHardware::on_configure(
    const rclcpp_lifecycle::State & /*previous_state*/)
  {
    RCLCPP_INFO(rclcpp::get_logger("DiffDriveArduinoHardware"), "Configuring ...please wait...");
    if (comms_.connected())
    {
      comms_.disconnect();
    }
    comms_.connect(cfg_.device, cfg_.baud_rate, cfg_.timeout_ms);
    RCLCPP_INFO(rclcpp::get_logger("DiffDriveArduinoHardware"), "Successfully configured!");

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::CallbackReturn DiffDriveArduinoHardware::on_cleanup(
    const rclcpp_lifecycle::State & /*previous_state*/)
  {
    RCLCPP_INFO(rclcpp::get_logger("DiffDriveArduinoHardware"), "Cleaning up ...please wait...");
    if (comms_.connected())
    {
      comms_.disconnect();
    }
    RCLCPP_INFO(rclcpp::get_logger("DiffDriveArduinoHardware"), "Successfully cleaned up!");

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::CallbackReturn DiffDriveArduinoHardware::on_activate(
    const rclcpp_lifecycle::State & /*previous_state*/)
  {
    RCLCPP_INFO(rclcpp::get_logger("DiffDriveArduinoHardware"), "Activating ...please wait...");
    if (!comms_.connected())
    {
      return hardware_interface::CallbackReturn::ERROR;
    }
    if (cfg_.pid_p > 0)
    {
      comms_.set_pid_values(cfg_.pid_p,cfg_.pid_d,cfg_.pid_i,cfg_.pid_o);
    }
    RCLCPP_INFO(rclcpp::get_logger("DiffDriveArduinoHardware"), "Successfully activated!");

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::CallbackReturn DiffDriveArduinoHardware::on_deactivate(
    const rclcpp_lifecycle::State & /*previous_state*/)
  {
    RCLCPP_INFO(rclcpp::get_logger("DiffDriveArduinoHardware"), "Deactivating ...please wait...");
    RCLCPP_INFO(rclcpp::get_logger("DiffDriveArduinoHardware"), "Successfully deactivated!");

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::return_type DiffDriveArduinoHardware::read(
    const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
  {
    if (!comms_.connected())
    {
      return hardware_interface::return_type::ERROR;
    }

    comms_.read_encoder_values(left_back_wheel_->enc, right_back_wheel_->enc);
    comms_.read_encoder_values(left_front_wheel_->enc, right_front_wheel_->enc);

    double delta_seconds = period.seconds();

    double pos_prev = left_back_wheel_->pos;
    left_back_wheel_->pos = left_back_wheel_->calc_enc_angle();
    left_back_wheel_->vel = (left_back_wheel_->pos - pos_prev) / delta_seconds;

    pos_prev = right_back_wheel_->pos;
    right_back_wheel_->pos = right_back_wheel_->calc_enc_angle();
    right_back_wheel_->vel = (right_back_wheel_->pos - pos_prev) / delta_seconds;

    pos_prev = left_front_wheel_->pos;
    left_front_wheel_->pos = left_front_wheel_->calc_enc_angle();
    left_front_wheel_->vel = (left_front_wheel_->pos - pos_prev) / delta_seconds;

    pos_prev = right_front_wheel_->pos;
    right_front_wheel_->pos = right_front_wheel_->calc_enc_angle();
    right_front_wheel_->vel = (right_front_wheel_->pos - pos_prev) / delta_seconds;

    return hardware_interface::return_type::OK;
  }

  hardware_interface::return_type DiffDriveArduinoHardware::write(
    const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
  {
    if (!comms_.connected())
    {
      return hardware_interface::return_type::ERROR;
    }

    // Calculate the average motor command for the back wheels
    int motor_l_counts_per_loop = (left_back_wheel_->cmd + right_back_wheel_->cmd) / 2.0 / left_back_wheel_->rads_per_count / cfg_.loop_rate;

    // Calculate the motor command for the front wheels
    int motor_r_counts_per_loop = (left_front_wheel_->cmd + right_front_wheel_->cmd) / 2.0 / left_front_wheel_->rads_per_count / cfg_.loop_rate;

    comms_.set_motor_values(motor_l_counts_per_loop, motor_r_counts_per_loop);
    return hardware_interface::return_type::OK;
  }

  void DiffDriveArduinoHardware::setupWheels(const hardware_interface::HardwareInfo &info)
  {
    left_back_wheel_ = std::make_unique<wheels::Wheel>(cfg_.left_back_wheel_name, cfg_.enc_counts_per_rev);
    right_back_wheel_ = std::make_unique<wheels::Wheel>(cfg_.right_back_wheel_name, cfg_.enc_counts_per_rev);
    left_front_wheel_ = std::make_unique<wheels::Wheel>(cfg_.left_front_wheel_name, cfg_.enc_counts_per_rev);
    right_front_wheel_ = std::make_unique<wheels::Wheel>(cfg_.right_front_wheel_name, cfg_.enc_counts_per_rev);
  }

  void DiffDriveArduinoHardware::calculateMotorCounts()
  {
    // Calculate motor counts per loop based on wheel radius and counts per revolution
    if (wheel_radius_ != 0.0)
    {
      // Motor counts per revolution = (2 * pi * wheel_radius) / counts_per_revolution
      motor_counts_per_loop_ = static_cast<int>((2 * M_PI * wheel_radius_) / cfg_.enc_counts_per_rev);
    }
    else
    {
      motor_counts_per_loop_ = 0;
    }
  }

}  // namespace diffdrive_arduino

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(diffdrive_arduino::DiffDriveArduinoHardware, hardware_interface::SystemInterface)
