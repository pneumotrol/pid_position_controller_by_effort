#ifndef PID_POSITION_CONTROLLER_BY_EFFORT__POSITION_CONTROLLER_HPP_
#define PID_POSITION_CONTROLLER_BY_EFFORT__POSITION_CONTROLLER_HPP_

#include <string>

#include "control_msgs/msg/multi_dof_command.hpp"
#include "control_msgs/msg/single_dof_state_stamped.hpp"
#include "controller_interface/controller_interface.hpp"
#include "pid_position_controller_by_effort/visibility_control.h"
#include "position_controller_parameters.hpp"
#include "realtime_tools/realtime_buffer.h"
#include "realtime_tools/realtime_publisher.h"
#include "std_srvs/srv/set_bool.hpp"

namespace pid_position_controller_by_effort {

class PositionController : public controller_interface::ControllerInterface {
 public:
  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  PositionController();

  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  controller_interface::CallbackReturn
  on_init() override;

  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  controller_interface::InterfaceConfiguration
  command_interface_configuration() const override;

  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  controller_interface::InterfaceConfiguration
  state_interface_configuration() const override;

  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  controller_interface::CallbackReturn
  on_configure(const rclcpp_lifecycle::State &previous_state) override;

  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  controller_interface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State &previous_state) override;

  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  controller_interface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State &previous_state) override;

  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  controller_interface::return_type
  update(const rclcpp::Time &time, const rclcpp::Duration &preiod) override;

  using TargetMsg = control_msgs::msg::MultiDOFCommand;
  using StateMsg = control_msgs::msg::SingleDOFStateStamped;

 protected:
  // parameters are defined in a yaml file,
  // and are libraryed by generate_parameter_library in CMakeLists.txt.
  std::shared_ptr<position_controller::ParamListener> param_listener_;
  position_controller::Params params_;

  // parameter
  realtime_tools::RealtimeBuffer<std::shared_ptr<TargetMsg>> target_;

  // target subscriber
  rclcpp::Subscription<TargetMsg>::SharedPtr target_subscriber_ = nullptr;

  // current state publisher
  using StatePublisher = realtime_tools::RealtimePublisher<StateMsg>;
  std::unique_ptr<StatePublisher> state_publisher_;

 private:
  // reference value subscribe callback
  TEMPLATES__ROS2_CONTROL__VISIBILITY_LOCAL
  void target_callback(const std::shared_ptr<TargetMsg> msg);
};

}  // namespace pid_position_controller_by_effort

#endif  // PID_POSITION_CONTROLLER_BY_EFFORT__POSITION_CONTROLLER_HPP_