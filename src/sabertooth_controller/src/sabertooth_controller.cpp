#include "sabertooth_controller/sabertooth_controller.hpp"

using namespace std::chrono_literals;
using namespace sabertooth_controller;

SabertoothController::SabertoothController() : Node("sabertooth_controller")
{
  motor_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
    "/motor/command", 10,
    std::bind(&SabertoothController::motorCallback, this, std::placeholders::_1));

  std::string port = "/dev/ttyS0";
  uint32_t baud_rate = 9600;

  try
  {
    io_context_ = std::make_unique<drivers::common::IoContext>();

    drivers::serial_driver::SerialPortConfig config(
      baud_rate,
      drivers::serial_driver::FlowControl::NONE,
      drivers::serial_driver::Parity::NONE,
      drivers::serial_driver::StopBits::ONE
    );

    serial_port_ = std::make_shared<drivers::serial_driver::SerialPort>(
      *io_context_, port, config);

    serial_port_->open();

    RCLCPP_INFO(this->get_logger(), "Port série %s ouvert à %u bauds.", port.c_str(), baud_rate);
  }
  catch (const std::exception &e)
  {
    RCLCPP_ERROR(this->get_logger(), "Erreur d'initialisation du port série : %s", e.what());
  }

  safety_timer_ = this->create_wall_timer(
    500ms, std::bind(&SabertoothController::safetyTimerCallback, this));

  last_command_time_ = this->now();
}

void SabertoothController::motorCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  int motor1 = static_cast<int>(msg->linear.x * 127.0);
  int motor2 = static_cast<int>(msg->angular.z * 127.0);
  sendCommand(motor1, motor2);
  last_command_time_ = this->now();
}

void SabertoothController::sendCommand(int motor1, int motor2)
{
  if (!serial_port_ || !serial_port_->is_open())
  {
    RCLCPP_ERROR(this->get_logger(), "Port série non ouvert.");
    return;
  }

  motor1 = std::max(-127, std::min(127, motor1));
  motor2 = std::max(-127, std::min(127, motor2));

  uint8_t command[4] = {
    static_cast<uint8_t>(0x80),
    static_cast<uint8_t>(motor1 & 0x7F),
    static_cast<uint8_t>(motor2 & 0x7F),
    static_cast<uint8_t>((0x80 + motor1 + motor2) & 0x7F)
  };

  std::vector<uint8_t> buffer(command, command + sizeof(command));

  try
  {
    serial_port_->send(buffer);
    RCLCPP_DEBUG(this->get_logger(), "Commande envoyée : motor1=%d, motor2=%d", motor1, motor2);
  }
  catch (const std::exception &e)
  {
    RCLCPP_ERROR(this->get_logger(), "Erreur d'envoi série : %s", e.what());
  }
}

void SabertoothController::safetyTimerCallback()
{
  auto now = this->now();
  if ((now - last_command_time_).seconds() > 1.0)
  {
    sendCommand(0, 0);
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                         "Aucune commande reçue depuis 1s, envoi d'arrêt de sécurité.");
    last_command_time_ = now;
  }
}

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SabertoothController>());
  rclcpp::shutdown();
  return 0;
}
