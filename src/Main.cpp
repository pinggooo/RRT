#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "Application.hpp"

Application app;

class StartPosSubscriber : public rclcpp::Node {
public:
    StartPosSubscriber() : Node("start_position_subscriber") {
        subscription_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
                "initialpose", 1, std::bind(&StartPosSubscriber::topic_callback, this, std::placeholders::_1)
                );
    }

private:
    void topic_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) const {
        if (!app.isGotStartPos) {
            app.start_pos.x() = float(msg->pose.pose.position.x);
            app.start_pos.y() = float(msg->pose.pose.position.y);
            app.isGotStartPos = true;
        }
    }

    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr subscription_;
};

class EndPosSubscriber : public rclcpp::Node {
public:
    EndPosSubscriber() : Node("end_position_subscriber") {
        subscription_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
                "goal_pose", 1, std::bind(&EndPosSubscriber::topic_callback, this, std::placeholders::_1)
        );
    }

private:
    void topic_callback(const geometry_msgs::msg::PointStamped::SharedPtr msg) const {
        if (!app.isGotEndPos) {
            app.end_pos.x() = float(msg->point.x);
            app.end_pos.y() = float(msg->point.y);
            app.isGotEndPos = true;
        }
    }

    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr subscription_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    while (rclcpp::ok() && app.initialize()) {
        app.run();
    }

    rclcpp::shutdown();

    return 0;
}
