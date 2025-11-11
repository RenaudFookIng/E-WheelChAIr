#ifndef SABERTOOTH_CONTROLLER__SABERTOOTH_CONTROLLER_HPP_
#define SABERTOOTH_CONTROLLER__SABERTOOTH_CONTROLLER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <serial_driver/serial_driver.hpp>

namespace sabertooth_controller
{
    class SabertoothController : public rclcpp::Node
    {
    public:
        SabertoothController();

    private:
        void motorCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
        void sendCommand(int motor1, int motor2);

        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr motor_sub_;
        std::shared_ptr<serial_driver::SerialPort> serial_port_;
    };
} // namespace sabertooth_controller

#endif // SABERTOOTH_CONTROLLER__SABERTOOTH_CONTROLLER_HPP_
