#include "Application.hpp"

Application::Application() : Node("rrt_simulator") {
    this->isGotStartPos = false;
    this->isGotEndPos = false;
    this->isGotMapSize = false;
    this->isFinished = false;
    this->isMaxLoopOver = true;
    this->rrt = nullptr;
    this->map = new Map();

    start_point_pub_ = this->create_publisher<geometry_msgs::msg::PointStamped>("start_point", rclcpp::QoS(rclcpp::KeepAll()));
    end_point_pub_ = this->create_publisher<geometry_msgs::msg::PointStamped>("end_point", rclcpp::QoS(rclcpp::KeepAll()));
    rrt_point_pub_ = this->create_publisher<geometry_msgs::msg::PointStamped>("path_point", rclcpp::QoS(rclcpp::KeepAll()).best_effort());
    rrt_line_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("path_line", rclcpp::QoS(rclcpp::KeepAll()).best_effort());
    rrt_path_pub_ = this->create_publisher<nav_msgs::msg::Path>("final_path", rclcpp::QoS(rclcpp::KeepAll()).best_effort());

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
            this->rrt = new RRT(map);
            RCLCPP_INFO(this->get_logger(), "RRT is Initialized! Map Size: [%fx%f]", map->getMapSize().x(), map->getMapSize().y());
        }

        return true;
    }

    return false;
}

void Application::run() {
    auto start_time = std::chrono::steady_clock::now();

    for (int i = 0; i < rrt->getMaxLoopCount(); i++) {
        TreeNode* random_node = rrt->getRandomNode();

        if (random_node == nullptr) {
            continue;
        }

        drawPathPoint_(random_node);

        if (rrt->isReached()) {
            this->isMaxLoopOver = false;
            rrt->updatePath(rrt->getLastNode());
            Eigen::Vector2f last_pos = rrt->getLastNode()->getPosition();
            RCLCPP_INFO(this->get_logger(), "RRT is reached at end position! (%f, %f)", last_pos.x(), last_pos.y());
            break;
        }
    }

    if (isMaxLoopOver) {
        RCLCPP_INFO(this->get_logger(), "Loop is over max loop count(%d)!", rrt->getMaxLoopCount());
    }

    rrt->refinePath();
    drawPathLine_(rrt->getPath());

    auto end_time = std::chrono::steady_clock::now();
    RCLCPP_INFO(this->get_logger(), "Elapsed Time(ms) : %d", std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time));
    isFinished = true;
}

void Application::drawPathPoint_(TreeNode* node) {
    geometry_msgs::msg::PointStamped point_;

    point_.header.frame_id = "map";
    point_.point.x = node->getPosition().x();
    point_.point.y = node->getPosition().y();
    point_.point.z = 0;

    this->rrt_point_pub_->publish(point_);

    visualization_msgs::msg::Marker marker_;

    marker_.id = node->getId();
    marker_.type = visualization_msgs::msg::Marker::LINE_STRIP;
    marker_.action = visualization_msgs::msg::Marker::ADD;
    marker_.header.frame_id = "map";
    marker_.scale.x = 0.01;
    marker_.color.a = 1.0;

    geometry_msgs::msg::Point line_point_;
    geometry_msgs::msg::Point line_parent_;

    line_point_.x = node->getPosition().x();
    line_point_.y = node->getPosition().y();
    line_point_.z = 0;
    line_parent_.x = node->getParent()->getPosition().x();
    line_parent_.y = node->getParent()->getPosition().y();
    line_parent_.z = 0;

    marker_.points.push_back(line_parent_);
    marker_.points.push_back(line_point_);

    rrt_line_pub_->publish(marker_);
    RCLCPP_INFO(this->get_logger(), "Node %d is made! (%f, %f)", node->getId(), node->getPosition().x(), node->getPosition().y());
}

void Application::drawPathLine_(const std::vector<TreeNode*>& path) {
    if (path.empty()) {
        RCLCPP_INFO(this->get_logger(), "Any path is not discovered...");
        return;
    }

    nav_msgs::msg::Path path_;
    path_.header.frame_id = "map";

    for (auto node : path) {
        geometry_msgs::msg::PoseStamped pose_;
        pose_.header.frame_id = "map";
        pose_.pose.position.x = node->getPosition().x();
        pose_.pose.position.y = node->getPosition().y();
        pose_.pose.position.z = 0;
        path_.poses.push_back(pose_);
    }

    rrt_path_pub_->publish(path_);
    RCLCPP_INFO(this->get_logger(), "The path is created!");
}

void Application::startPosCallback_(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
    if (!this->isGotStartPos) {
        Eigen::Vector2f start_pos(float(msg->pose.pose.position.x), float(msg->pose.pose.position.y));
        this->map->setStartPos(start_pos);
        this->isGotStartPos = true;

        geometry_msgs::msg::PointStamped start_point_;
        start_point_.header.frame_id = "map";
        start_point_.point.x = this->map->getStartPos().x();
        start_point_.point.y = this->map->getStartPos().y();
        start_point_.point.z = 0;

        start_point_pub_->publish(start_point_);
        RCLCPP_INFO(this->get_logger(), "Start Position is Selected! (%f, %f)", this->map->getStartPos().x(), this->map->getStartPos().y());
    }
}

void Application::endPosCallback_(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    if (!this->isGotEndPos) {
        Eigen::Vector2f end_pos(float(msg->pose.position.x), float(msg->pose.position.y));
        this->map->setEndPos(end_pos);
        this->isGotEndPos = true;

        geometry_msgs::msg::PointStamped end_point_;
        end_point_.header.frame_id = "map";
        end_point_.point.x = this->map->getEndPos().x();
        end_point_.point.y = this->map->getEndPos().y();
        end_point_.point.z = 0;

        end_point_pub_->publish(end_point_);
        RCLCPP_INFO(this->get_logger(), "End Position is Selected! (%f, %f)", this->map->getEndPos().x(), this->map->getEndPos().y());
    }
}

void Application::mapDataCallback_(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
    if (!this->isGotMapSize) {
        Eigen::Vector2f map_size(float(msg->info.width * msg->info.resolution), float(msg->info.height * msg->info.resolution));
        Eigen::Vector2f map_origin(msg->info.origin.position.x, msg->info.origin.position.y);
        this->map->setMapSize(map_size);
        this->map->setMapData(msg->data);
        this->map->setMapOrigin(map_origin);
        this->map->setMapResolution(msg->info.resolution);
        this->isGotMapSize = true;

        RCLCPP_INFO(this->get_logger(), "Map is Loaded! Map Size: [%.1fx%.1f]", float(this->map->getMapSize().x()), float(this->map->getMapSize().y()));
    }
}