/**
 *  A minimal ROS2 cpp node that publishes a message every second.
 *
 *  Basic structure for ROS2 node:
 *  - Initialise rclpy
 *  - Create a node
 *  - Create the publisher
 *  - Create a timer to publish at specific intervals
 *  - Keep it alive with spin
 *  - Shutdown
 *
 *  BEST_EFFORT and VOLATILE have been selected for the QOS_PROFILE, as the
 *  node keeps publishing at fixed intervals and losing messages it's fine.
 *
 *  * If you want reliability change BEST_EFFORT to RELIABLE.
 *  * If you want persisting samples for late-joining subscriptions,
 *    change VOLATILE to TRANSIENT_LOCAL.
 *
 *  Author: Ioannis Selinis <selinis.g@gmail.com> 2025
 */

#include <rclcpp/rclcpp.hpp>
#include "example_interfaces/msg/string.hpp"


static const rmw_qos_profile_t rmw_qos_profile_latch =
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


class HelloPublisher : public rclcpp::Node
{
    public:
        HelloPublisher() : Node("hello_c_publisher")
        {
            auto qos = rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_latch), rmw_qos_profile_latch);
            m_timer = this->create_wall_timer(std::chrono::seconds(1),
                                              std::bind(&HelloPublisher::timerCallback, this));
            m_publisher = this->create_publisher<example_interfaces::msg::String>("hello_world", qos);
            RCLCPP_INFO(this->get_logger(), "Node has started");
        }

    private:
        void timerCallback()
        {
            auto msg = example_interfaces::msg::String();
            msg.data = "Hello counter: " + std::to_string(m_counter);
            m_publisher->publish(msg);
            m_counter++;
        }
        rclcpp::TimerBase::SharedPtr m_timer;
        rclcpp::Publisher<example_interfaces::msg::String>::SharedPtr m_publisher;
        int m_counter = 0;
};


int main (int argc, char **argv)
{
    rclcpp::init(argc, argv);                       // Initialise rclcpp (ROS communications)
    auto node = std::make_shared<HelloPublisher>(); // Smart/Shared pointer
    rclcpp::spin(node);                             // Keep node alive
    rclcpp::shutdown();                             // Shutdown executor
}
