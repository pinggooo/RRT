#ifndef RRT_TREENODE_HPP
#define RRT_TREENODE_HPP

#include <iostream>
#include <vector>
#include <eigen3/Eigen/Dense>

class Node {
public:
    Node() = default;

    Node(Node* parent, const Eigen::Vector2f& position) {
        this->parent = parent;
        this->position = position;
        this->id = count++;
        parent->addChild(*this);
    }

    int getId();
    std::vector<Node> getChildren();
    void addChild(const Node& node);
    void removeChild(const Node& node);
    Node* getParent();
    void setParent(Node* node);
    void removeParent();
    Eigen::Vector2f getPosition();

    bool operator == (const Node& object) const {
        if (this->id == object.id) {
            return true;
        }

        return false;
    }

    ~Node() = default;

private:
    int id{};
    Node* parent{};
    std::vector<Node> children;
    Eigen::Vector2f position;
    static int count;
};

#endif //RRT_TREENODE_HPP
