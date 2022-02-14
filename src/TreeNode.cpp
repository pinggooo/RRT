#include "TreeNode.hpp"
#include "catch2/catch_all.hpp"

int TreeNode::count = 1;

void TreeNode::addChild(TreeNode* node) {
    for (auto& iter : this->children) {
        if (iter->getId() == node->getId()) {
            return;
        }
    }

    this->children.push_back(node);
}

void TreeNode::removeChild(TreeNode* node) {
    this->children.erase(std::remove(this->children.begin(), this->children.end(), node),
                         this->children.end());
}

void TreeNode::removeParent() {
    this->parent = nullptr;
    this->depth = 0;
}

int TreeNode::getId() {
    return this->id;
}

std::vector<TreeNode*> TreeNode::getChildren() {
    return this->children;
}

TreeNode* TreeNode::getParent() {
    return this->parent;
}

Eigen::Vector2f TreeNode::getPosition() {
    return this->position;
}

int TreeNode::getDepth() {
    return this->depth;
}

void TreeNode::setParent(TreeNode* node) {
    this->parent = node;
    this->depth = this->parent->getDepth() + 1;
}

void TreeNode::setPosition(const Eigen::Vector2f& position_) {
    this->position = position_;
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

TEST_CASE("TreeNode TEST", "[tree_node]") {
    TreeNode parent;
    Eigen::Vector2f position(77, 777);
    TreeNode node(&parent, position);

    REQUIRE(parent.getId() == 0);
    REQUIRE(parent.getChildren().size() == 1);
    REQUIRE(node.getId() == 1);
    REQUIRE(node.getParent()->getId() == 0);
    REQUIRE(node.getChildren().empty() == true);
    REQUIRE(node.getPosition().x() == 77);
    REQUIRE(node.getPosition().y() == 777);


    Eigen::Vector2f new_position(11, 111);
    TreeNode new_parent(&parent, new_position);
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
    REQUIRE(node.getChildren().at(0) == &child1);
    REQUIRE(node.getChildren().at(1) == &child2);
}
*/