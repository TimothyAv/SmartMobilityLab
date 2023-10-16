#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <delivery_action_interfaces/action/order_delivery.hpp>

class OrderDeliveryServer : public rclcpp::Node
{
public:
    using OrderDelivery = delivery_action_interfaces::action::OrderDelivery;
    using GoalHandle = rclcpp_action::ServerGoalHandle<OrderDelivery>;

    OrderDeliveryServer()
        : Node("order_delivery_server"), server_(rclcpp_action::create_server<OrderDelivery>(
            this, "order_delivery", std::bind(&OrderDeliveryServer::handle_goal, this, std::placeholders::_1),
            std::bind(&OrderDeliveryServer::handle_cancel, this, std::placeholders::_1),
            std::bind(&OrderDeliveryServer::handle_accepted, this, std::placeholders::_1)))
    {
    }

private:
    rclcpp_action::Server<OrderDelivery>::SharedPtr server_;

    void handle_goal(const GoalHandle::SharedPtr goal_handle)
    {
        RCLCPP_INFO(get_logger(), "Received delivery order for order ID %d to destination: %s", goal_handle->get_goal()->order_id, goal_handle->get_goal()->destination);

        // Simulate a delivery process (you can replace this with your actual delivery logic)
        for (int i = 1; i <= 100; i++)
        {
            if (goal_handle->is_canceling())
            {
                goal_handle->canceled();
                RCLCPP_INFO(get_logger(), "Delivery canceled");
                return;
            }

            auto feedback = std::make_shared<OrderDelivery::Feedback>();
            feedback->progress = static_cast<float>(i);
            goal_handle->publish_feedback(feedback);

            rclcpp::Rate(1).sleep();  // Simulate delivery progress
        }

        auto result = std::make_shared<OrderDelivery::Result>();
        result->success = true;
        result->message = "Delivery successful";
        goal_handle->succeed(result);
    }

    void handle_cancel(const std::shared_ptr<GoalHandle> goal_handle)
    {
        RCLCPP_INFO(get_logger(), "Received cancel request for delivery order");
        goal_handle->canceled();
    }

    void handle_accepted(const std::shared_ptr<GoalHandle> goal_handle)
    {
        RCLCPP_INFO(get_logger(), "Accepted new delivery order");
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OrderDeliveryServer>());
    rclcpp::shutdown();
    return 0;
}


