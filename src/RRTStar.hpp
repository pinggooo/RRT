#ifndef RRT_RRTSTAR_HPP
#define RRT_RRTSTAR_HPP

#include "RRT.hpp"


class RRTStar : public RRT {
public:
    explicit RRTStar(Map* map, int max_loop_count = 50000) : RRT(map, max_loop_count) {
        this->nearest_scope = 5 * step_size;
        this->max_loop_count = max_loop_count;
    }

    ~RRTStar() = default;

    TreeNode* getRandomNode() override;
    TreeNode* getScopeOptimum(TreeNode* node);

private:
    void reconstruct(TreeNode* node, const std::vector<TreeNode*>& candidates);

    float nearest_scope;
};

#endif //RRT_RRTSTAR_HPP
