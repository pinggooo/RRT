#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "Application.hpp"

Application* app;

class StartPosSubscriber : public rclcpp::Node {
public:
    StartPosSubscriber() : Node("start_position_subscriber") {
        auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(1));
        subscription_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
                "initialpose", qos_profile, std::bind(&StartPosSubscriber::topic_callback, this, std::placeholders::_1)
                );
    }

private:
    void topic_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) const {
        if (!app->isGotStartPos) {
            app->start_pos.x() = float(msg->pose.pose.position.x);
            app->start_pos.y() = float(msg->pose.pose.position.y);
            app->isGotStartPos = true;

            RCLCPP_INFO(this->get_logger(), "Start Position Selected!");
        }
    }

    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr subscription_;
};

class EndPosSubscriber : public rclcpp::Node {
public:
    EndPosSubscriber() : Node("end_position_subscriber") {
        auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(1));
        subscription_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
                "goal_pose", qos_profile, std::bind(&EndPosSubscriber::topic_callback, this, std::placeholders::_1)
        );
    }

private:
    void topic_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) const {
        if (!app->isGotEndPos) {
            app->end_pos.x() = float(msg->pose.position.x);
            app->end_pos.y() = float(msg->pose.position.y);
            app->isGotEndPos = true;

            RCLCPP_INFO(this->get_logger(), "End Position Selected!");
        }
    }

    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr subscription_;
};

int main(int argc, char** argv) {
    app = new Application();
    rclcpp::init(argc, argv);
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(std::make_shared<StartPosSubscriber>());
    executor.add_node(std::make_shared<EndPosSubscriber>());

    while (rclcpp::ok()) {
        if (app->initialize() && !app->isFinished) {
            app->run();
        }

        executor.spin_once();
    }

    rclcpp::shutdown();

    return 0;
}
