#ifndef RRT_RRTCONNECT_HPP
#define RRT_RRTCONNECT_HPP

#include "RRT.hpp"

class RRTConnect {
public:
    explicit RRTConnect(Map* map, int max_loop_count = 30000) {
        this->start_rrt = new RRT(map);
        this->end_rrt = new RRT(map);
        this->max_loop_count = max_loop_count;
        this->connect_threshold = float(std::max(map->getMapSize().x(), map->getMapSize().y()) / 225.0);

        initializeEndRRT();
    };

    ~RRTConnect() = default;

    RRT* getStartRRT();
    RRT* getEndRRT();
    int getMaxLoopCount();

private:
    void initializeEndRRT();

    RRT* start_rrt;
    RRT* end_rrt;
    int max_loop_count;
    float connect_threshold;
    std::vector<TreeNode*> path;
};

#endif //RRT_RRTCONNECT_HPP
