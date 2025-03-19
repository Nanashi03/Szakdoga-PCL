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
        std::pmr::vector<IPointCloudShape> clouds;
    public:
        Model();
        void addCloud(const IPointCloudShape&);
        void updateCloud(const IPointCloudShape&);
        void removeCloud(const IPointCloudShape&);
};

#endif //MODEL_H
