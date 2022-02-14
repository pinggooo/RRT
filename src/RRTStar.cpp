#include "RRTStar.hpp"


TreeNode* RRTStar::getRandomNode() {
    std::random_device rd;
    std::default_random_engine eng(rd());
    std::uniform_real_distribution<float> urd(0, 1);

    Eigen::Vector2f map_origin = map->getMapOrigin();
    Eigen::Vector2f map_size = map->getMapSize();

    Eigen::Vector2f random_pos(map_origin.x() + urd(eng) * map_size.x(), map_origin.y() + urd(eng) * map_size.y());
    TreeNode* random_node = new TreeNode(nullptr, random_pos);
    TreeNode* nearest = getNearest(random_node);

    Eigen::Vector2f vector = random_node->getPosition() - nearest->getPosition();
    Eigen::Vector2f unit_vector = vector / vector.norm();

    random_pos = nearest->getPosition() + unit_vector * this->step_size;
    random_node->setPosition(random_pos);

    if (!this->map->isValidPos(random_pos)) {
        return nullptr;
    }

    if (random_pos.x() < map_origin.x() || random_pos.x() > map_origin.x() + map_size.x()) {
        return nullptr;
    }

    if (random_pos.y() < map_origin.y() || random_pos.y() > map_origin.y() + map_size.y()) {
        return nullptr;
    }

    TreeNode* scope_optimum = getScopeOptimum(random_node);

    if (scope_optimum == nullptr) {
        return nullptr;
    }

    addNode(random_node, scope_optimum);

    return random_node;
}

TreeNode* RRTStar::getScopeOptimum(TreeNode* node) {
    TreeNode* scope_optimum;
    std::vector<TreeNode*> candidates;

    for (auto node_iter : this->node_list) {
        float distance = Map::getDistance(node->getPosition(), node_iter->getPosition());

        if (distance <= this->nearest_scope) {
            if (checkRayCast(node->getPosition(), node_iter->getPosition())) {
                candidates.push_back(node_iter);
            }
        }
    }

    if (candidates.empty()) {
        return nullptr;
    }

    int min_depth = int(this->node_list.size());

    for (auto candidate : candidates) {
        if (candidate->getDepth() < min_depth) {
            scope_optimum = candidate;
            min_depth = candidate->getDepth();
            continue;
        }

        if (candidate->getDepth() == min_depth) {
            float candidate_distance = Map::getDistance(candidate->getPosition(), map->getEndPos());
            float optimum_distance = Map::getDistance(scope_optimum->getPosition(), map->getEndPos());

            if (candidate_distance < optimum_distance) {
                scope_optimum = candidate;
            }
        }
    }

    candidates.erase(std::remove(candidates.begin(), candidates.end(), scope_optimum), candidates.end());
    this->reconstruct(scope_optimum, candidates);

    return scope_optimum;
}

void RRTStar::reconstruct(TreeNode* node, const std::vector<TreeNode*>& candidates) {
    for (auto candidate : candidates) {
        if (node->getDepth() < candidate->getParent()->getDepth()) {
            if (checkRayCast(node->getPosition(), candidate->getPosition())) {
                candidate->setParent(node);
            }
        }
    }
}