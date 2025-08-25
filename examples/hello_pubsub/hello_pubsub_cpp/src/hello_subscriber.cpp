/**
 *  A minimal ROS2 cpp node that subscribes to a topic.
 *
 *  Basic structure for ROS2 node:
 *  - Initialise rclcpp
 *  - Create a node
 *  - Create the subscriber
 *  - Keep it alive with spin
 *  - Shutdown
 *
 *  BEST_EFFORT and VOLATILE have been selected for the QOS_PROFILE, as the
 *  publisher uses them.
 *
 *  Author: Ioannis Selinis <selinis.g@gmail.com> 2025
 */

#include <rclcpp/rclcpp.hpp>
#include "example_interfaces/msg/string.hpp"


static const rmw_qos_profile_t rmw_qos_profile =
{
    RMW_QOS_POLICY_HISTORY_KEEP_LAST,
    1,
    RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
    RMW_QOS_POLICY_DURABILITY_VOLATILE,
    RMW_QOS_DEADLINE_DEFAULT,
    RMW_QOS_LIFESPAN_DEFAULT,
    RMW_QOS_POLICY_LIVELINESS_SYSTEM_DEFAULT,
    RMW_QOS_LIVELINESS_LEASE_DURATION_DEFAULT,
    false
};


class HelloSubscriber : public rclcpp::Node
{
    public:
        HelloSubscriber() : Node("hello_c_subscriber")
        {
            auto qos = rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile), rmw_qos_profile);
            m_subscriber = this->create_subscription<example_interfaces::msg::String>("hello_world",
                                                                                      qos,
                                                                                      std::bind(&HelloSubscriber::topicCallback, this, std::placeholders::_1));
            RCLCPP_INFO(this->get_logger(), "Node has started");
        }

    private:
        void topicCallback(const example_interfaces::msg::String::SharedPtr msg)
        {
            RCLCPP_INFO(this->get_logger(), "Received %s", msg->data.c_str());
        }
        rclcpp::Subscription<example_interfaces::msg::String>::SharedPtr m_subscriber;
};


int main (int argc, char **argv)
{
    rclcpp::init(argc, argv);                        // Initialise rclcpp (ROS communications)
    auto node = std::make_shared<HelloSubscriber>(); // Smart/Shared pointer
    rclcpp::spin(node);                              // Keep node alive
    rclcpp::shutdown();                              // Shutdown executor
}