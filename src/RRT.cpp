#include "RRT.hpp"

Node* RRT::getRandomNode() {
    std::random_device rd;
    std::default_random_engine eng(rd());
    std::uniform_real_distribution<float> urd(0, 1);

    Eigen::Vector2f random_pos(urd(eng) * MAP_WIDTH, urd(eng) * MAP_HEIGHT);
    Node* random_node = new Node(nullptr, random_pos);
    Node* nearest = getNearest(random_node);

    Eigen::Vector2f vector = random_node->getPosition() - nearest->getPosition();
    Eigen::Vector2f unit_vector = vector / vector.norm();

    random_pos = nearest->getPosition() + unit_vector * this->step_size;
    random_node->setPosition(random_pos);

    if (random_pos.x() < 0 || random_pos.x() > MAP_WIDTH) {
        delete random_node;
        return nullptr;
    }

    if (random_pos.y() < 0 || random_pos.y() > MAP_HEIGHT) {
        delete random_node;
        return nullptr;
    }

    return random_node;
}

float RRT::getDistance(Node* a, Node* b) {
    Eigen::Vector2f a_pos = a->getPosition();
    Eigen::Vector2f b_pos = b->getPosition();
    float x_diff = abs(a_pos.x() - b_pos.x());
    float y_diff = abs(a_pos.y() - b_pos.y());

    return sqrtf(x_diff * x_diff + y_diff * y_diff);
}

Node* RRT::getNearest(Node* node) {
    Node* nearest;
    float min_distance = sqrtf(MAP_WIDTH * MAP_WIDTH + MAP_HEIGHT * MAP_HEIGHT);

    for (auto& node_iter : this->node_list) {
        float distance = getDistance(node, &node_iter);

        if (distance < min_distance) {
            nearest = &node_iter;
            min_distance = distance;
        }
    }

    return nearest;
}

void RRT::addNode(Node* node, Node* parent) {
    node->setParent(parent);
    parent->getChildren().push_back(*node);
    this->node_list.push_back(*node);
    this->last_node = node;
}

bool RRT::isReached() {
    if (getDistance(last_node, end_node) <= END_REACH_THRESHOLD) {
        end_node = last_node;

        return true;
    }

    return false;
}