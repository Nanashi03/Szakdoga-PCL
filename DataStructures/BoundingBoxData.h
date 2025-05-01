//
// Created by kristof on 4/15/25.
//

#ifndef BOUNDINGBOXDATA_H
#define BOUNDINGBOXDATA_H

#include <pcl/common/common.h>

struct BoundingBoxData
{
    std::string NAME = "BBOX";
    float minX, minY, minZ;
    float maxX, maxY, maxZ;
};

#endif //BOUNDINGBOXDATA_H
