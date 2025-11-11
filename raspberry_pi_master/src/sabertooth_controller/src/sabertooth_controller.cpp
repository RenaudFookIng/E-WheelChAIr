#include "sabertooth_controller/sabertooth_controller.hpp"

using namespace std::chrono_literals;
using namespace sabertooth_controller;

SabertoothController::SabertoothController() : Node("sabertooth_controller")
{
    motor_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "/motor/command", 10, std::bind(&SabertoothController::motorCallback, this, std::placeholders::_1));

    serial_port_ = std::make_shared<serial_driver::SerialPort>();
    serial_driver::SerialPortConfig config;
    config.port = "/dev/ttyS0";
    config.baud_rate = 9600;
    config.flow_control = serial_driver::FlowControl::NONE;
    config.parity = serial_driver::Parity::NONE;
    config.stop_bits = serial_driver::StopBits::ONE;

    try {
        serial_port_->open(config);
        RCLCPP_INFO(this->get_logger(), "Port série ouvert avec succès.");
    } catch (const std::exception &e) {
        RCLCPP_ERROR(this->get_logger(), "Erreur lors de l'ouverture du port série: %s", e.what());
    }
}

void SabertoothController::motorCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
    int motor1 = static_cast<int>(msg->linear.x * 127.0);
    int motor2 = static_cast<int>(msg->angular.z * 127.0);
    sendCommand(motor1, motor2);
}

void SabertoothController::sendCommand(int motor1, int motor2)
{
    if (!serial_port_->is_open()) {
        RCLCPP_ERROR(this->get_logger(), "Port série non ouvert.");
        return;
    }

    uint8_t command[4] = {
        static_cast<uint8_t>(0x80),
        static_cast<uint8_t>(motor1),
        static_cast<uint8_t>(motor2),
        static_cast<uint8_t>((0x80 + motor1 + motor2) & 0x7F)
    };

    try {
        serial_port_->write(command, 4);
        RCLCPP_INFO(this->get_logger(), "Commande envoyée: motor1=%d, motor2=%d", motor1, motor2);
    } catch (const std::exception &e) {
        RCLCPP_ERROR(this->get_logger(), "Erreur lors de l'envoi de la commande: %s", e.what());
    }
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SabertoothController>());
    rclcpp::shutdown();
    return 0;
}
