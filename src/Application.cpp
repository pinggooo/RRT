#include "Application.hpp"

bool Application::initialize() {
    if (isGotStartPos && isGotEndPos && isGotMapSize) {
        if (rrt == nullptr) {
            this->rrt = new RRT(start_pos, end_pos, map_size);
        }

        return true;
    }

    return false;
}

void Application::run() {
    for (int i = 0; i < rrt->getMaxLoopCount(); i++) {
        Node* random_node = rrt->getRandomNode();

        if (random_node == nullptr) {
            continue;
        }

        rrt->addNode(random_node, rrt->getLastNode());

        if (rrt->isReached()) {
            break;
        }
    }

    Node* node = rrt->getLastNode();

    while (node != nullptr) {
        rrt->addPathNode(node);
        node = node->getParent();
    }
}