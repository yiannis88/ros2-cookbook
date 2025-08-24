/**
 *  A minimal ROS2 cpp node that logs a message every second.
 *
 *  Basic structure for ROS2 node:
 *  - Initialise rclpy
 *  - Create a node
 *  - Keep it alive with spin
 *  - Shutdown
 *
 *  Author: Ioannis Selinis <selinis.g@gmail.com> 2025
 */

#include <rclcpp/rclcpp.hpp>


class HelloNode : public rclcpp::Node
{
    public:
        HelloNode() : Node("hello_c_node")
        {
            m_timer = this->create_wall_timer(std::chrono::seconds(1),
                                              std::bind(&HelloNode::timerCallback, this));
            RCLCPP_INFO(this->get_logger(), "Node has started");
        }

    private:
        void timerCallback()
        {
            RCLCPP_INFO(this->get_logger(), "Hello counter: %d", m_counter);
            m_counter++;
        }
        rclcpp::TimerBase::SharedPtr m_timer;
        int m_counter = 0;
};


int main (int argc, char **argv)
{
    rclcpp::init(argc, argv);                   // Initialise rclcpp (ROS communications)
    auto node = std::make_shared<HelloNode>();  // Smart/Shared pointer
    rclcpp::spin(node);                         // Keep node alive
    rclcpp::shutdown();                         // Shutdown executor
}
