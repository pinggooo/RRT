#ifndef RRT_RRT_HPP
#define RRT_RRT_HPP

#include <random>
#include "TreeNode.hpp"

#define END_REACH_THRESHOLD 0.1

class RRT {
public:
    RRT(const Eigen::Vector2f& start_pos, const Eigen::Vector2f& end_pos, const Eigen::Vector2f& map_size) {
        this->start_node = new TreeNode(nullptr, start_pos);
        this->end_node = new TreeNode(nullptr, end_pos);
        this->last_node = start_node;
        this->node_list.push_back(start_node);
        this->map_size = map_size;
    }

    RRT(const Eigen::Vector2f& start_pos, const Eigen::Vector2f& end_pos, const Eigen::Vector2f& map_size,
        int step_size, int max_loop_count) {
        this->start_node = new TreeNode(nullptr, start_pos);
        this->end_node = new TreeNode(nullptr, end_pos);
        this->step_size = step_size;
        this->max_loop_count = max_loop_count;
        this->last_node = start_node;
        this->node_list.push_back(start_node);
        this->map_size = map_size;
    }

    TreeNode* getRandomNode();
    float getDistance(TreeNode* a, TreeNode* b);
    TreeNode* getNearest(TreeNode* node);
    void addNode(TreeNode* node, TreeNode* parent);
    bool isReached();
    int getStepSize();
    void setStepSize(const int& step_size_);
    int getMaxLoopCount();
    void setGetMaxLoopCount(const int& max_loop_count_);
    TreeNode* getStartNode();
    TreeNode* getEndNode();
    TreeNode* getLastNode();
    std::vector<TreeNode*> getNodeList();
    void addPathNode(TreeNode* node);
    std::vector<TreeNode*> getPath();

private:
    int step_size = 5;
    int max_loop_count = 1000;
    TreeNode* start_node;
    TreeNode* end_node;
    TreeNode* last_node;
    std::vector<TreeNode*> node_list;
    std::vector<TreeNode*> path;
    Eigen::Vector2f map_size;
};

#endif //RRT_RRT_HPP
