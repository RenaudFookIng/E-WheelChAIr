#ifndef MASTER_NODE__MASTER_NODE_HH_
#define MASTER_NODE__MASTER_NODE_HH_

#include <rclcpp/rclcpp.hpp>
#include <custom_msgs/msg/arduino_data.hpp>
#include <custom_msgs/msg/yolo_results.hpp>
#include <geometry_msgs/msg/twist.hpp>

namespace master_node
{
    class MasterNode : public rclcpp::Node
    {
    public:
        // Constructeur
        MasterNode();

    private:
        // Abonnements aux topics
        rclcpp::Subscription<custom_msgs::msg::ArduinoData>::SharedPtr arduino_sub_;
        rclcpp::Subscription<custom_msgs::msg::YoloResults>::SharedPtr yolo_sub_;

        // Publication des commandes moteur
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr motor_pub_;
        rclcpp::TimerBase::SharedPtr timer_;

        // Méthodes de callback pour traiter les données
        void arduinoCallback(const custom_msgs::msg::ArduinoData::SharedPtr msg);
        void yoloCallback(const custom_msgs::msg::YoloResults::SharedPtr msg);
    };
} // namespace master_node

#endif // MASTER_NODE__MASTER_NODE_HH_
