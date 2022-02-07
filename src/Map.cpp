#include "Map.hpp"

std::vector<int8_t> Map::getMapData() {
    return this->data;
}

Eigen::Vector2f Map::getMapSize() {
    return this->size;
}

Eigen::Vector2f Map::getMapOrigin() {
    return this->origin;
}

float Map::getMapResolution() {
    return this->resolution;
}

Eigen::Vector2f Map::getStartPos() {
    return this->start_pos;
}

Eigen::Vector2f Map::getEndPos() {
    return this->end_pos;
}

void Map::setMapData(std::vector<int8_t> map_data) {
    this->data = map_data;
}

void Map::setMapSize(Eigen::Vector2f map_size) {
    this->size = map_size;
}

void Map::setMapOrigin(Eigen::Vector2f map_origin) {
    this->origin = map_origin;
}

void Map::setMapResolution(float map_resolution) {
    this->resolution = map_resolution;
}

void Map::setStartPos(Eigen::Vector2f position) {
    this->start_pos = position;
}

void Map::setEndPos(Eigen::Vector2f position) {
    this->end_pos = position;
}
