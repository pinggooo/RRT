#include "TreeNode.hpp"
#include "catch2/catch_all.hpp"

int Node::count = 1;

int Node::getId() {
    return this->id;
}

std::vector<Node> Node::getChildren() {
    return this->children;
}

void Node::addChild(const Node& node) {
    for (auto& iter : this->children) {
        if (iter.id == node.id) {
            return;
        }
    }

    this->children.push_back(node);
}

void Node::removeChild(const Node& node) {
    this->children.erase(std::remove(this->children.begin(), this->children.end(), node),
                         this->children.end());
}

Node* Node::getParent() {
    return this->parent;
}

void Node::setParent(Node* node) {
    this->parent = node;
}

void Node::removeParent() {
    this->parent = nullptr;
}

Eigen::Vector2f Node::getPosition() {
    return this->position;
}

void Node::setPosition(const Eigen::Vector2f& position) {
    this->position = position;
}

//****************************************************//
//                    UNIT TESTING                    //
//****************************************************//

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

TEST_CASE("Node TEST", "[node]") {
    Node parent;
    Eigen::Vector2f position(77, 777);
    Node node(&parent, position);

    REQUIRE(parent.getId() == 0);
    REQUIRE(parent.getChildren().size() == 1);
    REQUIRE(node.getId() == 1);
    REQUIRE(node.getParent()->getId() == 0);
    REQUIRE(node.getChildren().empty() == true);
    REQUIRE(node.getPosition().x() == 77);
    REQUIRE(node.getPosition().y() == 777);


    Eigen::Vector2f new_position(11, 111);
    Node new_parent(&parent, new_position);
    node.setParent(&new_parent);

    REQUIRE(new_parent.getId() == 2);
    REQUIRE(node.getParent()->getId() == 2);
    REQUIRE(node.getParent()->getPosition().x() == 11);
    REQUIRE(node.getParent()->getPosition().y() == 111);


    Eigen::Vector2f new_position1(22, 222);
    Eigen::Vector2f new_position2(33, 333);
    Node child1(&node, new_position1);
    Node child2(&node, new_position2);

    REQUIRE(node.getChildren().size() == 2);
    REQUIRE(child1.getId() == 3);
    REQUIRE(child2.getId() == 4);
    REQUIRE(node.getChildren().at(0) == child1);
    REQUIRE(node.getChildren().at(1) == child2);
}