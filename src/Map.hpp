#ifndef RRT_MAP_HPP
#define RRT_MAP_HPP

#include <iostream>
#include <vector>
#include <eigen3/Eigen/Dense>

class Map {
public:
    Map() = default;
    ~Map() = default;

    void inflateData(int filter_size = 7);
    bool isValidPos(const Eigen::Vector2f& position);
    static float getDistance(const Eigen::Vector2f& a, const Eigen::Vector2f& b);

    std::vector<int8_t> getMapData();
    Eigen::Vector2f getMapSize();
    Eigen::Vector2f getMapOrigin();
    float getMapResolution();
    Eigen::Vector2f getStartPos();
    Eigen::Vector2f getEndPos();

    void setMapData(std::vector<int8_t> map_data);
    void setMapSize(Eigen::Vector2f map_size);
    void setMapOrigin(Eigen::Vector2f map_origin);
    void setMapResolution(float map_resolution);
    void setStartPos(Eigen::Vector2f position);
    void setEndPos(Eigen::Vector2f position);

private:
    Eigen::MatrixXi convolve(Eigen::MatrixXi matrix, Eigen::MatrixXi filter);

    std::vector<int8_t> data;
    Eigen::Vector2f size;
    Eigen::Vector2f origin;
    float resolution;
    Eigen::Vector2f start_pos;
    Eigen::Vector2f end_pos;
};

#endif //RRT_MAP_HPP
