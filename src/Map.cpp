#include "Map.hpp"

void Map::inflateData(int filter_size) {
    if (filter_size % 2 == 0) {
        return;
    }

    int size_x = int(this->size.x() / this->resolution);
    int size_y = int(this->size.y() / this->resolution);
    Eigen::MatrixXi filter(filter_size, filter_size);
    Eigen::MatrixXi map(size_x + 2 * int(filter_size / 2), size_y + 2 * int(filter_size / 2));
    map.setZero();

    for (int y = 0; y < size_y; y++) {
        for (int x = 0; x < size_x; x++) {
            map(y + int(filter_size / 2), x + int(filter_size / 2)) = data[size_x * y + x];
        }
    }

    map = convolve(map, filter);

    for (int y = 0; y < size_y; y++) {
        for (int x = 0; x < size_x; x++) {
            data[size_x * y + x] = map(y + int(filter_size / 2), x + int(filter_size / 2));
        }
    }
}

Eigen::MatrixXi Map::convolve(Eigen::MatrixXi matrix, Eigen::MatrixXi filter) {
    Eigen::MatrixXi convolved_matrix(matrix.rows(), matrix.cols());

    for (int y = int(filter.rows() / 2); y < matrix.rows() - int(filter.rows() / 2); y++) {
        for (int x = int(filter.cols() / 2); x < matrix.cols() - int(filter.cols() / 2); x++) {
            if (matrix(y, x) == 0) {
                continue;
            }

            filter.setConstant(matrix(y, x));

            for (int i = -int(filter.rows() / 2); i <= int(filter.rows() / 2); i++) {
               for (int j = -int(filter.cols() / 2); j <= int(filter.cols() / 2); j++) {
                   int filter_value = filter(i + int(filter.rows() / 2), j + int(filter.cols() / 2));

                   if (filter_value == -1 || matrix(y + i, x + j) < filter_value) {
                       convolved_matrix(y + i, x + j) = filter_value;
                   }
               }
           }
        }
    }

    return convolved_matrix;
}

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
