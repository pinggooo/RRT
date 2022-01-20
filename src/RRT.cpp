#include "RRT.hpp"
#include "catch2/catch_all.hpp"
#include <memory>

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
        return nullptr;
    }

    if (random_pos.y() < 0 || random_pos.y() > MAP_HEIGHT) {
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
        float distance = getDistance(node, node_iter);

        if (distance < min_distance) {
            nearest = node_iter;
            min_distance = distance;
        }
    }

    return nearest;
}

void RRT::addNode(Node* node, Node* parent) {
    node->setParent(parent);
    parent->addChild(node);
    this->node_list.push_back(node);
    this->last_node = node;
}

bool RRT::isReached() {
    if (getDistance(last_node, end_node) <= END_REACH_THRESHOLD) {
        end_node = last_node;

        return true;
    }

    return false;
}

int RRT::getStepSize() {
    return this->step_size;
}

void RRT::setStepSize(const int& step_size_) {
    this->step_size = step_size_;
}

int RRT::getMaxLoopCount() {
    return this->max_loop_count;
}

void RRT::setGetMaxLoopCount(const int& max_loop_count_) {
    this->max_loop_count = max_loop_count_;
}

Node* RRT::getStartNode() {
    return this->start_node;
}

Node* RRT::getEndNode() {
    return this->end_node;
}

Node* RRT::getLastNode() {
    return this->last_node;
}

std::vector<Node*> RRT::getNodeList() {
    return this->node_list;
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

    RRT rrt(start_pos, end_pos);

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
    Node* random = rrt.getRandomNode();
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