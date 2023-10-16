#include <rclcpp/rclcpp.hpp>
#include <delivery_action_interfaces/action/order_delivery.hpp>

class OrderDeliveryClient : public rclcpp::Node
{
public:
    OrderDeliveryClient() : Node("order_delivery_client")
    {
        action_client_ = rclcpp_action::create_client<delivery_action_interfaces::action::OrderDelivery>(
            shared_from_this(), "order_delivery"
        );
        goal_handle_ = nullptr;
        send_goal();
    }

private:
    void send_goal()
    {
        if (!action_client_->wait_for_action_server(std::chrono::seconds(10)))
        {
            RCLCPP_ERROR(get_logger(), "Action server not available. Exiting.");
            return;
        }

        auto goal_msg = delivery_action_interfaces::action::OrderDelivery::Goal();
        goal_msg.order_id = 123;
        goal_msg.destination = "Customer's address";

        auto send_goal_options = rclcpp_action::Client<delivery_action_interfaces::action::OrderDelivery>::SendGoalOptions();
        send_goal_options.goal_response_callback = std::bind(&OrderDeliveryClient::goal_response_callback, this, std::placeholders::_1);
        send_goal_options.feedback_callback = std::bind(&OrderDeliveryClient::feedback_callback, this, std::placeholders::_1, std::placeholders::_2);
        send_goal_options.result_callback = std::bind(&OrderDeliveryClient::result_callback, this, std::placeholders::_1);
        goal_handle_ = action_client_->async_send_goal(goal_msg, send_goal_options);
    }

    void goal_response_callback(std::shared_future<rclcpp_action::ClientGoalHandle<delivery_action_interfaces::action::OrderDelivery>::SharedPtr> future)
    {
        goal_handle_ = future.get();
        if (!goal_handle_)
        {
            RCLCPP_ERROR(get_logger(), "Goal was rejected by the action server. Exiting.");
        }
        else
        {
            RCLCPP_INFO(get_logger(), "Goal accepted by the action server");
        }
    }

    void feedback_callback(
        rclcpp_action::ClientGoalHandle<delivery_action_interfaces::action::OrderDelivery>::SharedPtr,
        const std::shared_ptr<const delivery_action_interfaces::action::OrderDelivery::Feedback> feedback)
    {
        RCLCPP_INFO(get_logger(), "Received feedback: %.2f%%", feedback->progress);
    }

    void result_callback(const rclcpp_action::ClientGoalHandle<delivery_action_interfaces::action::OrderDelivery>::WrappedResult & result)
    {
        switch (result.code)
        {
            case rclcpp_action::ResultCode::SUCCEEDED:
                RCLCPP_INFO(get_logger(), "Delivery successful!");
                break;
            case rclcpp_action::ResultCode::ABORTED:
                RCLCPP_ERROR(get_logger(), "Delivery aborted");
                break;
            case rclcpp_action::ResultCode::CANCELED:
                RCLCPP_ERROR(get_logger(), "Delivery canceled");
                break;
            default:
                RCLCPP_ERROR(get_logger(), "Unknown result code");
                break;
        }
        rclcpp::shutdown();
    }

    rclcpp_action::Client<delivery_action_interfaces::action::OrderDelivery>::SharedPtr action_client_;
    rclcpp_action::ClientGoalHandle<delivery_action_interfaces::action::OrderDelivery>::SharedPtr goal_handle_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OrderDeliveryClient>());
    rclcpp::shutdown();
    return 0;
}

