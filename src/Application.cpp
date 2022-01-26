#include "Application.hpp"

Application::Application() : Node("rrt_simulator"){
    this->isGotStartPos = false;
    this->isGotEndPos = false;
    this->isGotMapSize = false;
    this->isFinished = false;
    this->rrt = nullptr;

    rrt_path_pub_ = this->create_publisher<geometry_msgs::msg::PointStamped>("path_point", rclcpp::QoS(rclcpp::KeepAll()));
    start_pos_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "initialpose", rclcpp::QoS(rclcpp::KeepLast(1)), std::bind(&Application::startPosCallback_, this, std::placeholders::_1));
    end_pos_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "goal_pose", rclcpp::QoS(rclcpp::KeepLast(1)), std::bind(&Application::endPosCallback_, this, std::placeholders::_1));
    map_data_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "map", rclcpp::QoS(rclcpp::KeepLast(1)), std::bind(&Application::mapDataCallback_, this, std::placeholders::_1));
}

bool Application::initialize() {
    if (isGotStartPos && isGotEndPos && isGotMapSize) {
        if (rrt == nullptr) {
            this->rrt = new RRT(start_pos, end_pos, map_size);
            RCLCPP_INFO(this->get_logger(), "RRT is Initialized! Map Size: [%fx%f]", map_size.x(), map_size.y());
        }

        return true;
    }

    return false;
}

void Application::run() {
    for (int i = 0; i < rrt->getMaxLoopCount(); i++) {
        TreeNode* random_node = rrt->getRandomNode();

        if (random_node == nullptr) {
            continue;
        }

        rrt->addNode(random_node, rrt->getLastNode());

        auto point_ = geometry_msgs::msg::PointStamped();
        point_.header.frame_id = "map";
        point_.point.x = random_node->getPosition().x();
        point_.point.y = random_node->getPosition().y();
        point_.point.z = 0;
        this->rrt_path_pub_->publish(point_);
        rclcpp::sleep_for(std::chrono::nanoseconds(100000000));
        RCLCPP_INFO(this->get_logger(), "Node %d is made! (%f, %f)", random_node->getId(), random_node->getPosition().x(), random_node->getPosition().y());

        if (rrt->isReached()) {
            Eigen::Vector2f last_pos = rrt->getLastNode()->getPosition();
            RCLCPP_INFO(this->get_logger(), "RRT is reached at end position! (%f, %f)", last_pos.x(), last_pos.y());
            break;
        }
    }

    TreeNode* node = rrt->getLastNode();

    while (node != nullptr) {
        rrt->addPathNode(node);
        node = node->getParent();
    }

    isFinished = true;
}

void Application::startPosCallback_(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
    if (!this->isGotStartPos) {
        this->start_pos.x() = float(msg->pose.pose.position.x);
        this->start_pos.y() = float(msg->pose.pose.position.y);
        this->isGotStartPos = true;

        RCLCPP_INFO(this->get_logger(), "Start Position is Selected! (%f, %f)", this->start_pos.x(), this->start_pos.y());
    }
}

void Application::endPosCallback_(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    if (!this->isGotEndPos) {
        this->end_pos.x() = float(msg->pose.position.x);
        this->end_pos.y() = float(msg->pose.position.y);
        this->isGotEndPos = true;

        RCLCPP_INFO(this->get_logger(), "End Position is Selected! (%f, %f)", this->end_pos.x(), this->end_pos.y());
    }
}

void Application::mapDataCallback_(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
    if (!this->isGotMapSize) {
        this->map_size.x() = float(msg->info.width * msg->info.resolution);
        this->map_size.y() = float(msg->info.height * msg->info.resolution);
        this->isGotMapSize = true;

        RCLCPP_INFO(this->get_logger(), "Map is Loaded! Map Size: [%fx%f]", float(this->map_size.x()), float(this->map_size.y()));
    }
}