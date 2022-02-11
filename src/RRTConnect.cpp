#include "RRTConnect.hpp"


bool RRTConnect::isReached() {
    if (this->activated_rrt->isReached()) {
        return true;
    }

    for (auto node : this->inactivated_rrt->getNodeList()) {
        if (Map::getDistance(this->activated_rrt->getLastNode()->getPosition(), node->getPosition()) <= connect_threshold) {
            connect_node = node;
            return true;
        }
    }

    return false;
}

void RRTConnect::swapActivation() {
    if (this->activated_rrt == this->start_rrt) {
        this->activated_rrt = this->end_rrt;
        this->inactivated_rrt = this->start_rrt;
    } else {
        this->activated_rrt = this->start_rrt;
        this->inactivated_rrt = this->end_rrt;
    }
}

void RRTConnect::updatePath() {
    if (this->activated_rrt->isReached()) {
        this->activated_rrt->updatePath(this->activated_rrt->getLastNode());
        return;
    }

    std::vector<TreeNode*> new_path;
    TreeNode* node = this->activated_rrt->getLastNode();

    while (node != nullptr) {
        new_path.push_back(node);
        node = node->getParent();
    }

    std::reverse(new_path.begin(), new_path.end());

    node = this->connect_node;

    while(node != nullptr) {
        new_path.push_back(node);
        node = node->getParent();
    }

    std::vector<TreeNode*> current_path = this->activated_rrt->getPath();

    if (current_path.empty() || (!current_path.empty() && new_path.size() < current_path.size())) {
        this->activated_rrt->setPath(new_path);
    }
}

void RRTConnect::refinePath() {
    this->activated_rrt->refinePath();
}

RRT* RRTConnect::getActivatedRRT() {
    return this->activated_rrt;
}

int RRTConnect::getMaxLoopCount() {
    return this->max_loop_count;
}

std::vector<TreeNode*> RRTConnect::getPath() {
    return this->activated_rrt->getPath();
}
