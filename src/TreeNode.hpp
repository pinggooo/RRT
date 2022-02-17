#ifndef RRT_TREENODE_HPP
#define RRT_TREENODE_HPP

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
            this->depth = this->parent->getDepth() + 1;
            this->parent->addChild(this);
        }
    }

    void addChild(TreeNode* node);
    void removeChild(TreeNode* node);
    void removeParent();

    int getId();
    TreeNode* getParent();
    std::vector<TreeNode*> getChildren();
    Eigen::Vector2f getPosition();
    int getDepth();

    void setParent(TreeNode* node);
    void setPosition(const Eigen::Vector2f& position_);

    bool operator == (const TreeNode& object) const {
        if (this->id == object.id) {
            return true;
        }

        return false;
    }

    ~TreeNode() = default;

private:
    int id{};
    int depth{};
    TreeNode* parent{};
    std::vector<TreeNode*> children;
    Eigen::Vector2f position;
    static int count;
};

#endif //RRT_TREENODE_HPP
