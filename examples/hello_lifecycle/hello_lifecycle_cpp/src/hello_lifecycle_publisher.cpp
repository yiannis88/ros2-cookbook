/**
 *  A minimal ROS2 cpp node that publishes a message every second.
 *
 *  Basic structure for ROS2 node:
 *  - Initialise rclcpp
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
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include "example_interfaces/msg/string.hpp"
#include "example_interfaces/srv/set_bool.hpp"


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


class HelloLifecyclePublisher : public rclcpp_lifecycle::LifecycleNode
{
    public:
        HelloLifecyclePublisher() : LifecycleNode("hello_lifecycle_c_publisher")
        {
            m_counter = 0;
            m_executor = nullptr;
            RCLCPP_INFO(this->get_logger(), "Node has started");
        }

        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
        on_configure(const rclcpp_lifecycle::State &state) override
        {
            auto qos = rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile), rmw_qos_profile);
            m_current_state = "inactive";
            RCLCPP_INFO(get_logger(), "%s transitions from %s (%d) to %s - create publisher & service", get_name(), state.label().c_str(), state.id(), m_current_state.c_str());
            m_publisher = this->create_publisher<example_interfaces::msg::String>("hello_lifecycle", qos);

            m_server = this->create_service<example_interfaces::srv::SetBool>("lifecycle_activate",
                                                                              std::bind(&HelloLifecyclePublisher::serviceCallback,
                                                                                        this, std::placeholders::_1,
                                                                                        std::placeholders::_2));
            return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
        }
        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
        on_activate(const rclcpp_lifecycle::State &state) override
        {
            m_current_state = "active";
            RCLCPP_INFO(get_logger(), "%s transitions from %s (%d) to %s", get_name(), state.label().c_str(), state.id(), m_current_state.c_str());
            m_publisher->on_activate();
            m_timer = this->create_wall_timer(std::chrono::seconds(1),
                                              std::bind(&HelloLifecyclePublisher::timerCallback, this));
            return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
        }
        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
        on_deactivate(const rclcpp_lifecycle::State &state) override
        {
            m_current_state = "inactive";
            RCLCPP_INFO(get_logger(), "%s transitions from %s (%d) to %s", get_name(), state.label().c_str(), state.id(), m_current_state.c_str());
            m_publisher->on_deactivate();
            m_timer->reset();
            return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
        }
        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
        on_cleanup(const rclcpp_lifecycle::State &state) override
        {
            m_current_state = "unconfigured";
            RCLCPP_INFO(get_logger(), "%s transitions from %s (%d) to %s", get_name(), state.label().c_str(), state.id(), m_current_state.c_str());
            m_publisher->on_deactivate();
            m_timer->reset();
            return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
        }
        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
        on_shutdown(const rclcpp_lifecycle::State &state) override
        {
            m_current_state = "finalized";
            RCLCPP_INFO(get_logger(), "%s transitions from %s (%d) to %s", get_name(), state.label().c_str(), state.id(), m_current_state.c_str());
            m_publisher->on_deactivate();
            m_timer->reset();
            if (m_executor) {
                m_executor->cancel();
            }
            return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
        }
        void set_executor(rclcpp::executors::SingleThreadedExecutor* executor)
        {
            m_executor = executor;
        }
    private:
        void timerCallback()
        {
            auto msg = example_interfaces::msg::String();
            msg.data = "Lifecycle counter: " + std::to_string(m_counter);
            m_publisher->publish(msg);
            m_counter++;
        }
        void serviceCallback(const example_interfaces::srv::SetBool::Request::SharedPtr request,
                             const example_interfaces::srv::SetBool::Response::SharedPtr response)
        {
            RCLCPP_INFO(get_logger(), "Service callback triggered with %s", request->data ? "true" : "false");
            if (m_current_state == ""){
                response->success = false;
                response->message = "Unknown state";
                return;
            }
            response->success = true;
            if (m_current_state == "unconfigured"){
                if (request->data){
                    trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
                    response->message = "Node configured";
                } else{
                    trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_UNCONFIGURED_SHUTDOWN);
                    response->message = "Node finalized";
                }
            } else if (m_current_state == "inactive") {
                if (request->data) {
                    trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
                    response->message = "Node activated";
                } else {
                    trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CLEANUP);
                    response->message = "Node cleaned up";
                }
            } else if (m_current_state == "active") {
                if (request->data) {
                    trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVE_SHUTDOWN);
                    response->message = "Node finalized (shutdown)";
                } else {
                    trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE);
                    response->message = "Node deactivated";
                }
            }
        }
        rclcpp::executors::SingleThreadedExecutor* m_executor;
        rclcpp::TimerBase::SharedPtr m_timer;
        rclcpp_lifecycle::LifecyclePublisher<example_interfaces::msg::String>::SharedPtr m_publisher;
        rclcpp::Service<example_interfaces::srv::SetBool>::SharedPtr m_server;
        std::string m_current_state;
        int m_counter = 0;
};


int main (int argc, char **argv)
{
    try{
        rclcpp::init(argc, argv);
        rclcpp::executors::SingleThreadedExecutor executor;
        auto node = std::make_shared<HelloLifecyclePublisher>();
        node->set_executor(&executor);
        executor.add_node(node->get_node_base_interface());
        node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
        executor.spin();
        rclcpp::shutdown();                             // Shutdown executor
    } catch (...){
        RCLCPP_ERROR(rclcpp::get_logger("hello_lifecycle_publisher"), "Main loop crashed");
    }
}
