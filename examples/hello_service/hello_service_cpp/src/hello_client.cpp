/**
 *  A minimal ROS2 cpp node that creates a service.
 *
 *  Basic structure for ROS2 node:
 *  - Initialise rclcpp
 *  - Create a node
 *  - Create the client for the service
 *  - Keep it alive with spin
 *  - Shutdown
 *
 *  RELIABLE and VOLATILE have been selected for the QOS_PROFILE to match with those
 *  defined in the service server.
 *
 *  Author: Ioannis Selinis <selinis.g@gmail.com> 2025
 */

#include <random>
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


class HelloClient : public rclcpp::Node
{
    public:
        HelloClient() : Node("hello_c_client")
        {
            m_rng = std::mt19937 (m_rd());
            m_distr = std::uniform_int_distribution<int>(0, 237);
            m_client = this->create_client<example_interfaces::srv::AddTwoInts>("add_two_ints", rmw_qos_profile);
            while (!m_client->wait_for_service(std::chrono::seconds(1))){
                RCLCPP_WARN(this->get_logger(), "Waiting for the server...");
            }
            m_timer = this->create_wall_timer(std::chrono::seconds(1),
                                              std::bind(&HelloClient::callService, this));
            RCLCPP_INFO(this->get_logger(), "Node is ready");
        }

    private:
        void callService()
        {
            auto request = std::make_shared<example_interfaces::srv::AddTwoInts::Request>();
            request->a = m_distr(m_rng);
            request->b = m_distr(m_rng);
            m_client->async_send_request(request,
                                         std::bind(&HelloClient::responseService, this, std::placeholders::_1));
        }
        void responseService(rclcpp::Client<example_interfaces::srv::AddTwoInts>::SharedFuture future)
        {
            auto response = future.get();
            RCLCPP_INFO(this->get_logger(), "Response sum: %d", (int) response->sum);
        }
        rclcpp::Client<example_interfaces::srv::AddTwoInts>::SharedPtr m_client;
        rclcpp::TimerBase::SharedPtr m_timer;
        std::random_device m_rd;
        std::mt19937 m_rng;
        std::uniform_int_distribution<int> m_distr;
};


int main (int argc, char **argv)
{
    rclcpp::init(argc, argv);                       // Initialise rclcpp (ROS communications)
    auto node = std::make_shared<HelloClient>();    // Smart/Shared pointer
    rclcpp::spin(node);                             // Keep node alive
    rclcpp::shutdown();                             // Shutdown executor
}
