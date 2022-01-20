#ifndef RRT_RRT_HPP
#define RRT_RRT_HPP

#include <random>
#include "TreeNode.hpp"

#define MAP_WIDTH 500.0
#define MAP_HEIGHT 500.0
#define START_POS_X 30.0
#define START_POS_Y 30.0
#define END_POS_X 470.0
#define END_POS_Y 470.0
#define END_REACH_THRESHOLD 1.0

class RRT {
public:
    RRT(const Eigen::Vector2f& start_pos, const Eigen::Vector2f& end_pos) {
        this->start_node = new Node(nullptr, start_pos);
        this->end_node = new Node(nullptr, end_pos);
        this->last_node = start_node;
        this->node_list.push_back(start_node);
    }

    RRT(const Eigen::Vector2f& start_pos, const Eigen::Vector2f& end_pos,
        int step_size, int max_loop_count) {
        this->start_node = new Node(nullptr, start_pos);
        this->end_node = new Node(nullptr, end_pos);
        this->step_size = step_size;
        this->max_loop_count = max_loop_count;
        this->last_node = start_node;
        this->node_list.push_back(start_node);
    }

    Node* getRandomNode();
    float getDistance(Node* a, Node* b);
    Node* getNearest(Node* node);
    void addNode(Node* node, Node* parent);
    bool isReached();
    int getStepSize();
    void setStepSize(const int& step_size_);
    int getMaxLoopCount();
    void setGetMaxLoopCount(const int& max_loop_count_);
    Node* getStartNode();
    Node* getEndNode();
    Node* getLastNode();
    std::vector<Node*> getNodeList();

private:
    int step_size = 5;
    int max_loop_count = 1000;
    Node* start_node;
    Node* end_node;
    Node* last_node;
    std::vector<Node*> node_list;
};

#endif //RRT_RRT_HPP
