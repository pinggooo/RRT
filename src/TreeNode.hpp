#ifndef RRT_TREENODE_HPP
#define RRT_TREENODE_HPP

#include <iostream>
#include <vector>
#include <eigen3/Eigen/Dense>

class TreeNode {
public:
    TreeNode() = default;

    TreeNode(TreeNode* parent, const Eigen::Vector2f& position) {
        this->parent = parent;
        this->position = position;
        this->id = count++;

        if (this->parent != nullptr) {
            this->parent->addChild(this);
        }
    }

    int getId();
    std::vector<TreeNode*> getChildren();
    void addChild(TreeNode* node);
    void removeChild(TreeNode* node);
    TreeNode* getParent();
    void setParent(TreeNode* node);
    void removeParent();
    Eigen::Vector2f getPosition();
    void setPosition(const Eigen::Vector2f& position);

    bool operator == (const TreeNode& object) const {
        if (this->id == object.id) {
            return true;
        }

        return false;
    }

    ~TreeNode() = default;

private:
    int id{};
    TreeNode* parent{};
    std::vector<TreeNode*> children;
    Eigen::Vector2f position;
    static int count;
};

#endif //RRT_TREENODE_HPP
