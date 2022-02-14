#ifndef RRT_RRTCONNECT_HPP
#define RRT_RRTCONNECT_HPP

#include "RRT.hpp"
#include "RRTStar.hpp"

class RRTConnect {
public:
    explicit RRTConnect(Map* map, int max_loop_count = 50000) {
        this->start_rrt = new RRTStar(map);
        this->end_rrt = new RRTStar(map);
        this->activated_rrt = this->start_rrt;
        this->inactivated_rrt = this->end_rrt;
        this->max_loop_count = max_loop_count;
        this->connect_threshold = float(std::max(map->getMapSize().x(), map->getMapSize().y()) / 225.0);

        //* start_node <-> end_node *//
        Eigen::Vector2f temp_pos = this->end_rrt->getStartNode()->getPosition();
        this->end_rrt->getStartNode()->setPosition(this->end_rrt->getEndNode()->getPosition());
        this->end_rrt->getEndNode()->setPosition(temp_pos);
        this->end_rrt->setLastNode(this->end_rrt->getStartNode());
        this->end_rrt->getNodeList().clear();
        this->end_rrt->getNodeList().push_back(this->end_rrt->getStartNode());
    };

    ~RRTConnect() = default;

    bool isReached();
    void swapActivation();
    void updatePath();
    void refinePath();

    RRT* getActivatedRRT();
    int getMaxLoopCount();
    std::vector<TreeNode*> getPath();

private:
    RRT* start_rrt;
    RRT* end_rrt;
    RRT* activated_rrt;
    RRT* inactivated_rrt;
    int max_loop_count;
    TreeNode* connect_node;
    float connect_threshold;
};

#endif //RRT_RRTCONNECT_HPP
