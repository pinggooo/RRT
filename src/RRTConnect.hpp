#ifndef RRT_RRTCONNECT_HPP
#define RRT_RRTCONNECT_HPP

#include "RRT.hpp"

class RRTConnect {
public:
    explicit RRTConnect(Map* map) {
        this->start_rrt = new RRT(map);
        this->end_rrt = new RRT(map);
    };

    ~RRTConnect() = default;

private:
    RRT* start_rrt;
    RRT* end_rrt;
};

#endif //RRT_RRTCONNECT_HPP
