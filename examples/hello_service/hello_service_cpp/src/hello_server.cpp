/**
 *  A minimal ROS2 cpp node that creates a service.
 *
 *  Basic structure for ROS2 node:
 *  - Initialise rclcpp
 *  - Create a node
 *  - Create the service
 *  - Keep it alive with spin
 *  - Shutdown
 *
 *  RELIABLE and VOLATILE have been selected for the QOS_PROFILE. The reason with BEST_EFFORT,
 *  is that we won't be able to call the service from the terminal (CLI) and with TRANSIENT_LOCAL
 *  that can cause the service servers to receive requests from clients that have since terminated.
 *
 *  Author: Ioannis Selinis <selinis.g@gmail.com> 2025
 */

#include <rclcpp/rclcpp.hpp>
#include <example_interfaces/srv/add_two_ints.hpp>


static const rmw_qos_profile_t rmw_qos_profile =
{
    RMW_QOS_POLICY_HISTORY_KEEP_LAST,
    1,
    RMW_QOS_POLICY_RELIABILITY_RELIABLE,
    RMW_QOS_POLICY_DURABILITY_VOLATILE,
    RMW_QOS_DEADLINE_DEFAULT,
    RMW_QOS_LIFESPAN_DEFAULT,
    RMW_QOS_POLICY_LIVELINESS_SYSTEM_DEFAULT,
    RMW_QOS_LIVELINESS_LEASE_DURATION_DEFAULT,
    false
};


class HelloServer : public rclcpp::Node
{
    public:
        HelloServer() : Node("hello_c_server")
        {
            m_server = this->create_service<example_interfaces::srv::AddTwoInts>("add_two_ints",
                                                                                 std::bind(&HelloServer::serviceCallback, this, std::placeholders::_1, std::placeholders::_2),
                                                                                 rmw_qos_profile);
            RCLCPP_INFO(this->get_logger(), "Node has started");
        }

    private:
        void serviceCallback(const example_interfaces::srv::AddTwoInts::Request::SharedPtr request,
                             const example_interfaces::srv::AddTwoInts::Response::SharedPtr response)
        {
            response->sum = request->a + request->b;
            RCLCPP_INFO(this->get_logger(), "Request received with %d + %d = %d", (int) request->a, (int) request->b, (int) response->sum);
        }
        rclcpp::Service<example_interfaces::srv::AddTwoInts>::SharedPtr m_server;
};


int main (int argc, char **argv)
{
    rclcpp::init(argc, argv);                       // Initialise rclcpp (ROS communications)
    auto node = std::make_shared<HelloServer>();    // Smart/Shared pointer
    rclcpp::spin(node);                             // Keep node alive
    rclcpp::shutdown();                             // Shutdown executor
}
