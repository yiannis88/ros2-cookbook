/**
 *  A minimal ROS2 cpp node that logs a message every second.
 *
 *  Basic structure for ROS2 node:
 *  - Initialise rclcpp
 *  - Create a node
 *  - Declare the params
 *  - Keep it alive with spin
 *  - Shutdown
 *
 *  Author: Ioannis Selinis <selinis.g@gmail.com> 2025
 */

#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include "rcl_interfaces/msg/floating_point_range.hpp"
#include <rcl_interfaces/msg/set_parameters_result.hpp>


class HelloParams : public rclcpp::Node
{
    public:
        HelloParams() : Node("hello_c_params")
        {
            rcl_interfaces::msg::ParameterDescriptor timer_interval_descriptor;
            timer_interval_descriptor.description = "Set the interval for the timer";
            timer_interval_descriptor.additional_constraints = "Range [1.0, 13.0] Step 2.0";
            timer_interval_descriptor.read_only = false;
            // automatic constraints!
            timer_interval_descriptor.floating_point_range.resize(1);
            timer_interval_descriptor.floating_point_range[0].from_value = 1.0;
            timer_interval_descriptor.floating_point_range[0].to_value   = 13.0;
            timer_interval_descriptor.floating_point_range[0].step       = 2.0;
            this->declare_parameter("timer_interval", 1.0, timer_interval_descriptor);

            rcl_interfaces::msg::ParameterDescriptor hello_msg_descriptor;
            hello_msg_descriptor.description = "The msg name to print";
            this->declare_parameter("hello_msg", "Orcheas", hello_msg_descriptor);

            m_timer_interval = this->get_parameter("timer_interval").as_double();
            m_hello_msg = this->get_parameter("hello_msg").as_string();

            m_param_cb = this->add_on_set_parameters_callback(std::bind(&HelloParams::paramCb, this, std::placeholders::_1));

            m_timer = this->create_wall_timer(std::chrono::duration<double>(m_timer_interval),
                                              std::bind(&HelloParams::printMessage, this));
            RCLCPP_INFO(this->get_logger(), "Node has started");
        }

    private:
        void printMessage()
        {
            RCLCPP_INFO(this->get_logger(), "Hello from: %s (interval=%.1f)", m_hello_msg.c_str(), m_timer_interval);
        }
        rcl_interfaces::msg::SetParametersResult paramCb(const std::vector<rclcpp::Parameter> &params)
        {
            for (const auto &param : params){
                if (param.get_name() == "timer_interval"){
                    m_timer_interval = param.as_double();
                    if (m_timer){
                        m_timer->cancel();
                    }
                    m_timer = this->create_wall_timer(std::chrono::duration<double>(m_timer_interval),
                                                      std::bind(&HelloParams::printMessage, this));
                }
                else if (param.get_name() == "hello_msg"){
                    m_hello_msg = param.as_string();
                }
            }
            rcl_interfaces::msg::SetParametersResult result;
            result.successful = true;
            result.reason = "success";
            return result;
        }
        rclcpp::TimerBase::SharedPtr m_timer;
        std::string m_hello_msg;
        double m_timer_interval;
        OnSetParametersCallbackHandle::SharedPtr m_param_cb;
};


int main (int argc, char **argv)
{
    rclcpp::init(argc, argv);                   // Initialise rclcpp (ROS communications)
    auto node = std::make_shared<HelloParams>();// Smart/Shared pointer
    rclcpp::spin(node);                         // Keep node alive
    rclcpp::shutdown();                         // Shutdown executor
}
