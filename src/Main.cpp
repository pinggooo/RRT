#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "Application.hpp"

class StartEndPointSubscriber : public rclcpp::Node {
public:
    StartEndPointSubscriber() : Node("start_end_point_subscriber") {
        subscription_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
                "clicked_point", 2, std::bind(&StartEndPointSubscriber::topic_callback, this, std::placeholders::_1);
                )
    }

private:
    void topic_callback(const geometry_msgs::msg::PointStamped::SharedPtr msg) const {

    }

    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr subscription_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    Application app;

    rclcpp::shutdown();
}
