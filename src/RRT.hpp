#ifndef RRT_RRT_HPP
#define RRT_RRT_HPP

#include <random>
#include "TreeNode.hpp"
#include "Map.hpp"

class RRT {
public:
    explicit RRT(Map* map) {
        this->start_node = new TreeNode(nullptr, map->getStartPos());
        this->end_node = new TreeNode(nullptr, map->getEndPos());
        this->last_node = start_node;
        this->node_list.push_back(start_node);
        this->map = map;
        this->step_size = float(std::max(map->getMapSize().x(), map->getMapSize().y()) / 150.0);
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
    std::vector<TreeNode*> getPath();
    void updatePath(TreeNode* node);
    void refinePath();
    bool checkRayCast(const Eigen::Vector2f& start, const Eigen::Vector2f& end);

private:
    float step_size;
    int max_loop_count = 20000;
    float end_reach_threshold;
    TreeNode* start_node;
    TreeNode* end_node;
    TreeNode* last_node;
    std::vector<TreeNode*> node_list;
    std::vector<TreeNode*> path;
    Map* map;
};

#endif //RRT_RRT_HPP
