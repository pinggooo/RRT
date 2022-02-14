#include "RRT.hpp"
#include "catch2/catch_all.hpp"


TreeNode* RRT::getRandomNode() {
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

    addNode(random_node, nearest);

    return random_node;
}

TreeNode* RRT::getNearest(TreeNode* node) {
    Eigen::Vector2f map_size = map->getMapSize();
    TreeNode* nearest;
    float min_distance = sqrtf(map_size.x() * map_size.x() + map_size.y() * map_size.y());

    for (auto node_iter : this->node_list) {
        float distance = Map::getDistance(node->getPosition(), node_iter->getPosition());

        if (distance < min_distance) {
            nearest = node_iter;
            min_distance = distance;
        }
    }

    return nearest;
}

void RRT::addNode(TreeNode* node, TreeNode* parent) {
    node->setParent(parent);
    parent->addChild(node);
    this->node_list.push_back(node);
    this->last_node = node;
}

bool RRT::isReached() {
    if (Map::getDistance(last_node->getPosition(), end_node->getPosition()) <= end_reach_threshold) {
        return true;
    }

    return false;
}

void RRT::updatePath(TreeNode* node) {
    if (node == nullptr) {
        return;
    }

    std::vector<TreeNode*> new_path;

    while (node != nullptr) {
        new_path.push_back(node);
        node = node->getParent();
    }

    if (this->path.empty() || (!this->path.empty() && new_path.size() < this->path.size())) {
        this->path.clear();

        for (auto new_node : new_path) {
            this->path.push_back(new_node);
        }
    }
}

void RRT::refinePath() {
    if (this->path.empty()) {
        return;
    }

    int start = 0;
    int end = int(this->path.size()) - 1;
    std::vector<TreeNode*> removal_candidates;

    while (start < end - 1) {
        for (int i = end; i > start + 1; i--) {
            if (checkRayCast(this->path[start]->getPosition(), this->path[i]->getPosition())) {
                for (int j = start + 1; j < i; j++) {
                    removal_candidates.push_back(this->path[j]);
                }

                start = i;
                break;
            }
        }

        start++;
    }

    if (removal_candidates.empty()) {
        return;
    }

    std::vector<TreeNode*> new_path;

    for (auto node : this->path) {
        if (std::find(removal_candidates.begin(), removal_candidates.end(), node) == removal_candidates.end()) {
            new_path.push_back(node);
        }
    }

    this->path.clear();

    for (auto node : new_path) {
        this->path.push_back(node);
    }
}

bool RRT::checkRayCast(const Eigen::Vector2f& start, const Eigen::Vector2f& end) {
    Eigen::Vector2f vector = end - start;
    int iter_size = int(std::ceil(std::max(std::abs(vector.x()), std::abs(vector.y())) / this->map->getMapResolution()));
    Eigen::Vector2f iter_vector = vector / iter_size;

    for (int i = 1; i < iter_size; i++) {
        if (!this->map->isValidPos(start + iter_vector * i)) {
            return false;
        }
    }

    return true;
}

float RRT::getStepSize() {
    return this->step_size;
}

int RRT::getMaxLoopCount() {
    return this->max_loop_count;
}

TreeNode* RRT::getStartNode() {
    return this->start_node;
}

TreeNode* RRT::getEndNode() {
    return this->end_node;
}

TreeNode* RRT::getLastNode() {
    return this->last_node;
}

std::vector<TreeNode*> RRT::getNodeList() {
    return this->node_list;
}

std::vector<TreeNode*> RRT::getPath() {
    return this->path;
}

void RRT::setStepSize(const float& step_size_) {
    this->step_size = step_size_;
    this->end_reach_threshold = float(step_size_ / 1.5);
}

void RRT::setGetMaxLoopCount(const int& max_loop_count_) {
    this->max_loop_count = max_loop_count_;
}

void RRT::setStartNode(TreeNode* start_node_) {
    this->start_node = start_node_;
}

void RRT::setEndNode(TreeNode* end_node_) {
    this->end_node = end_node_;
}

void RRT::setLastNode(TreeNode* last_node_) {
    this->last_node = last_node_;
}

void RRT::setPath(std::vector<TreeNode*> path_) {
    this->path = path_;
}


//****************************************************//
//                    UNIT TESTING                    //
//****************************************************//
/*
int main(int argc, char** argv) {
    Catch::Session session;
    bool isTestMode = false;
    auto cli = session.cli() | Catch::Clara::Opt(isTestMode)["--test"]("Run unit tests");
    session.cli(cli);

    int returnCode = session.applyCommandLine(argc, argv);

    if (returnCode != 0) {
        return returnCode;
    }

    if (isTestMode) {
        int numFailed = session.run();

        return numFailed;
    } else {
        std::cout << "This executable file is for Testing. Please add --test command." << std::endl;
    }

}

TEST_CASE("RRT TEST", "[rrt]") {
    Eigen::Vector2f start_pos(START_POS_X, START_POS_Y);
    Eigen::Vector2f end_pos(END_POS_X, END_POS_Y);
    Eigen::Vector2f map_size(500, 500);

    RRT rrt(start_pos, end_pos, map_size);

    REQUIRE(rrt.getStepSize() == 5);
    REQUIRE(rrt.getMaxLoopCount() == 1000);
    REQUIRE(rrt.getNodeList().size() == 1);
    REQUIRE(rrt.getStartNode()->getId() == 1);
    REQUIRE(rrt.getEndNode()->getId() == 2);
    REQUIRE(rrt.getStartNode()->getParent() == nullptr);
    REQUIRE(rrt.getEndNode()->getParent() == nullptr);
    REQUIRE(rrt.getStartNode()->getChildren().empty() == true);
    REQUIRE(rrt.getEndNode()->getChildren().empty() == true);
    REQUIRE(rrt.getStartNode()->getPosition().x() == 30);
    REQUIRE(rrt.getStartNode()->getPosition().y() == 30);
    REQUIRE(rrt.getEndNode()->getPosition().x() == 470);
    REQUIRE(rrt.getEndNode()->getPosition().y() == 470);

    rrt.setStepSize(1);
    TreeNode* random = rrt.getRandomNode();
    float distance = rrt.getDistance(rrt.getStartNode(), random);

    REQUIRE(random->getId() == 3);
    REQUIRE((distance >= rrt.getStepSize() - 0.01 && distance <= rrt.getStepSize() + 0.01) == true);
    std::cout << "Node is made at " << random->getPosition().x() <<
                ", " << random->getPosition().y() << std::endl;

    rrt.addNode(random, rrt.getStartNode());

    REQUIRE(random->getParent() == rrt.getStartNode());
    REQUIRE(rrt.getLastNode() == random);
    REQUIRE(random->getChildren().empty() == true);
    REQUIRE(rrt.getStartNode()->getChildren().size() == 1);
}
*/