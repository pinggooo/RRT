#ifndef RRT_APPLICATION_HPP
#define RRT_APPLICATION_HPP

#include "RRT.hpp"

class Application {
public:
    Application() {
        this->isGotStartPos = false;
        this->isGotEndPos = false;
        this->isGotMapSize = false;
        this->isFinished = false;
        this->rrt = nullptr;
    };

    ~Application() = default;

    bool initialize();
    void run();

    bool isGotStartPos;
    bool isGotEndPos;
    bool isGotMapSize;
    bool isFinished;
    Eigen::Vector2f start_pos;
    Eigen::Vector2f end_pos;
    Eigen::Vector2f map_size;
private:
    RRT* rrt;
};

#endif //RRT_APPLICATION_HPP
