#ifndef RRT_APPLICATION_HPP
#define RRT_APPLICATION_HPP

#include "nav2_map_server/map_server.hpp"
#include "RRT.hpp"

class Application {
public:
    Application() = default;
    ~Application() = default;

    void initialize();
private:
    nav2_map_server::MapServer* map_server;
    RRT* rrt;
};

#endif //RRT_APPLICATION_HPP
