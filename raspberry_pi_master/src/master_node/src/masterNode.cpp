#include "master_node/masterNode.hh"

using namespace std::chrono_literals;
using namespace master_node;

MasterNode::MasterNode() : rclcpp::Node("master_node")  // Modified to explicitly inherit from rclcpp::Node
{
    // Subscriptions for Arduino data (joystick/ultrasonic)
    arduino_sub_ = create_subscription<custom_msgs::msg::ArduinoData>(
        "arduino/data", 10,  // Removed leading slash for ROS 2
        std::bind(&MasterNode::arduinoCallback, this, std::placeholders::_1));

    // Subscription for YOLO results
    yolo_sub_ = create_subscription<custom_msgs::msg::YoloResults>(
        "yolo/results", 10,  // Removed leading slash for ROS 2
        std::bind(&MasterNode::yoloCallback, this, std::placeholders::_1));

    // Publisher for motor commands
    motor_pub_ = create_publisher<geometry_msgs::msg::Twist>("motor/command", 10);  // Removed leading slash

    // Timer for publishing commands
    timer_ = create_wall_timer(
        100ms, [this]() {
            auto command = geometry_msgs::msg::Twist();
            command.linear.x = 0.0;
            command.angular.z = 0.0;
            motor_pub_->publish(command);
        });
}

void MasterNode::arduinoCallback(const custom_msgs::msg::ArduinoData::SharedPtr msg)
{
    RCLCPP_INFO(this->get_logger(), "Joystick: X=%d, Y=%d, Distance=%f", msg->x, msg->y, msg->distance);
    // Logique pour traiter les données du joystick et des ultrasons
}

void MasterNode::yoloCallback(const custom_msgs::msg::YoloResults::SharedPtr msg)
{
    RCLCPP_INFO(this->get_logger(), "YOLO: %zu objets détectés", msg->labels.size());
    // Logique pour traiter les résultats YOLO
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MasterNode>());
    rclcpp::shutdown();
    return 0;
}
