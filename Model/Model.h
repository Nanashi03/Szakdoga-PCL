//
// Created by kristof on 2025.03.16..
//

#ifndef MODEL_H
#define MODEL_H

#include <vector>
#include <memory>
#include <algorithm>

#include "PointCloudShapes.h"

class Model {
    private:
        std::pmr::vector<std::shared_ptr<IPointCloudShape>> clouds;
    public:
        Model();
        void addCloud(std::shared_ptr<IPointCloudShape>);
        void updateCloud(std::shared_ptr<IPointCloudShape>);
        void removeCloud(std::shared_ptr<IPointCloudShape>);
};

#endif //MODEL_H
