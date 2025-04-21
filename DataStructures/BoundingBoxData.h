//
// Created by kristof on 4/15/25.
//

#ifndef BOUNDINGBOXDATA_H
#define BOUNDINGBOXDATA_H

#include <pcl/common/common.h>

struct BoundingBoxData
{
    std::string NAME = "BBOX";
    Eigen::Affine3f bboxQuaternion = Eigen::Affine3f::Identity(); //must reset after scaling/regenerating point cloud
    Eigen::Vector3f bboxTransform;
    double width, height, depth;
};

#endif //BOUNDINGBOXDATA_H
