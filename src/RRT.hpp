#ifndef RRT_RRT_HPP
#define RRT_RRT_HPP

#include <random>
#include "TreeNode.hpp"

class RRT {
public:
    RRT(const Eigen::Vector2f& start_pos, const Eigen::Vector2f& end_pos, const std::vector<int8_t>& map,
        const Eigen::Vector2f& map_size, const Eigen::Vector2f& map_origin, const float& map_resolution) {
        this->start_node = new TreeNode(nullptr, start_pos);
        this->end_node = new TreeNode(nullptr, end_pos);
        this->last_node = start_node;
        this->node_list.push_back(start_node);
        this->map = map;
        this->map_size = map_size;
        this->map_origin = map_origin;
        this->map_resolution = map_resolution;
        this->step_size = float(std::max(map_size.x(), map_size.y()) / 200.0);
        this->end_reach_threshold = float(step_size / 1.5);
    }

    ~RRT() = default;

    TreeNode* getRandomNode();
    float getDistance(TreeNode* a, TreeNode* b);
    TreeNode* getNearest(TreeNode* node);
    void addNode(TreeNode* node, TreeNode* parent);
    bool isObstacle(const Eigen::Vector2f& position);
    bool isReached();
    float getStepSize();
    void setStepSize(const float& step_size_);
    int getMaxLoopCount();
    void setGetMaxLoopCount(const int& max_loop_count_);
    TreeNode* getStartNode();
    TreeNode* getEndNode();
    TreeNode* getLastNode();
    std::vector<TreeNode*> getNodeList();
    void updatePath(TreeNode* node);
    std::vector<TreeNode*> getPath();

private:
    float step_size = 2;
    int max_loop_count = 20000;
    float end_reach_threshold = 1;
    TreeNode* start_node;
    TreeNode* end_node;
    TreeNode* last_node;
    std::vector<TreeNode*> node_list;
    std::vector<TreeNode*> path;
    std::vector<int8_t> map;
    Eigen::Vector2f map_size;
    Eigen::Vector2f map_origin;
    float map_resolution;
};

#endif //RRT_RRT_HPP
