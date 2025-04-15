//
// Created by kristof on 4/15/25.
//

#ifndef BOUNDINGBOXDATA_H
#define BOUNDINGBOXDATA_H

#include <pcl/common/common.h>

struct BoundingBoxData
{
    Eigen::Quaternionf bboxQuaternion;
    Eigen::Vector3f bboxTransform;
    double width, height, depth;
    const std::string NAME = "BBOX";
};

#endif //BOUNDINGBOXDATA_H
