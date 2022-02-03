#ifndef RRT_APPLICATION_HPP
#define RRT_APPLICATION_HPP

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/map_meta_data.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "RRT.hpp"

#include <chrono>

class Application : public rclcpp::Node {
public:
    Application();
    ~Application() = default;

    bool initialize();
    void run();

    bool isGotStartPos;
    bool isGotEndPos;
    bool isGotMapSize;
    bool isFinished;
    bool isMaxLoopOver;
    Eigen::Vector2f start_pos;
    Eigen::Vector2f end_pos;
    Eigen::Vector2f map_size;
    Eigen::Vector2f map_origin;
    std::vector<int8_t> map;
    float map_resolution;
private:
    void drawPathPoint_(TreeNode* node);
    void drawPathLine_(const std::vector<TreeNode*>& path);
    void publishWayPoints_(std::vector<TreeNode*> path);

    void startPosCallback_(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
    void endPosCallback_(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    void mapDataCallback_(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);

    RRT* rrt;
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr rrt_point_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr rrt_path_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr waypoint_pub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr start_pos_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr end_pos_sub_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_data_sub_;
};

#endif //RRT_APPLICATION_HPP
