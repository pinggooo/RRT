#include "RRTConnect.hpp"

void RRTConnect::initializeEndRRT() {
    //* start_node <-> end_node *//
    TreeNode* temp_node = this->end_rrt->getStartNode();
    this->end_rrt->setStartNode(this->end_rrt->getEndNode());
    this->end_rrt->setEndNode(temp_node);
    this->end_rrt->setLastNode(this->end_rrt->getStartNode());
    this->end_rrt->getNodeList().clear();
    this->end_rrt->getNodeList().push_back(this->end_rrt->getStartNode());
}

RRT* RRTConnect::getStartRRT() {
    return this->start_rrt;
}

RRT* RRTConnect::getEndRRT() {
    return this->end_rrt;
}

int RRTConnect::getMaxLoopCount() {
    return this->max_loop_count;
}
