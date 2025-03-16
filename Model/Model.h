//
// Created by kristof on 2025.03.16..
//

#ifndef MODEL_H
#define MODEL_H

#include <vector>
#include <algorithm>

#include "PointCloudShapes.h"

template<typename PointT>
class Model {
    private:
        std::pmr::vector<PointCloudShape<PointT>> clouds;
    public:
        Model();
        void addCloud(const PointCloudShape<PointT>&);
        void updateCloud(const PointCloudShape<PointT>&);
        void removeCloud(const PointCloudShape<PointT>&);
};

#endif //MODEL_H
