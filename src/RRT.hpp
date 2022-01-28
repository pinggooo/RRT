#ifndef RRT_RRT_HPP
#define RRT_RRT_HPP

#include <random>
#include "TreeNode.hpp"

class RRT {
public:
    RRT(const Eigen::Vector2f& start_pos, const Eigen::Vector2f& end_pos, const std::vector<int8_t> map,
        const Eigen::Vector2f& map_size, const float& map_resolution) {
        this->start_node = new TreeNode(nullptr, start_pos);
        this->end_node = new TreeNode(nullptr, end_pos);
        this->last_node = start_node;
        this->node_list.push_back(start_node);
        this->map = map;
        this->map_size = map_size;
        this->map_resolution = map_resolution;
    }

    ~RRT() = default;

    TreeNode* getRandomNode();
    float getDistance(TreeNode* a, TreeNode* b);
    TreeNode* getNearest(TreeNode* node);
    void addNode(TreeNode* node, TreeNode* parent);
    bool isObstacle(const Eigen::Vector2f& position);
    bool isReached();
    int getStepSize();
    void setStepSize(const int& step_size_);
    int getMaxLoopCount();
    void setGetMaxLoopCount(const int& max_loop_count_);
    TreeNode* getStartNode();
    TreeNode* getEndNode();
    TreeNode* getLastNode();
    std::vector<TreeNode*> getNodeList();
    void updatePath(TreeNode* node);
    std::vector<TreeNode*> getPath();

private:
    int step_size = 2;
    int max_loop_count = 5000;
    float end_reach_threshold = 2;
    TreeNode* start_node;
    TreeNode* end_node;
    TreeNode* last_node;
    std::vector<TreeNode*> node_list;
    std::vector<TreeNode*> path;
    std::vector<int8_t> map;
    Eigen::Vector2f map_size;
    float map_resolution;
};

#endif //RRT_RRT_HPP
