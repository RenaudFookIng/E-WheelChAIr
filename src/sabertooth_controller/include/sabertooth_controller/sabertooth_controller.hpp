#ifndef SABERTOOTH_CONTROLLER__SABERTOOTH_CONTROLLER_HPP_
#define SABERTOOTH_CONTROLLER__SABERTOOTH_CONTROLLER_HPP_

#include <memory>
#include <chrono>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <serial_driver/serial_driver.hpp>
#include <serial_driver/serial_port.hpp>

namespace sabertooth_controller
{
class SabertoothController : public rclcpp::Node
{
public:
  SabertoothController();

private:
  void motorCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
  void sendCommand(int motor1, int motor2);
  void safetyTimerCallback();

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr motor_sub_;
  std::shared_ptr<drivers::serial_driver::SerialPort> serial_port_;
  std::unique_ptr<drivers::common::IoContext> io_context_;

  rclcpp::TimerBase::SharedPtr safety_timer_;
  rclcpp::Time last_command_time_;
};
}  // namespace sabertooth_controller

#endif  // SABERTOOTH_CONTROLLER__SABERTOOTH_CONTROLLER_HPP_
