//
// Created by kristof on 2025.03.13..
//

#ifndef POINTCLOUDSHAPES_H
#define POINTCLOUDSHAPES_H

#inclide "PointTypes.h"

#include <pcl/io/pcd_io.h>

template <typename PointT>
class PointCloudShape {
protected:
    pcl::PointCloud<PointT> shape;
    std::string id;
    PointCloudShape(const std::string&);
public:
    virtual void generateShape();
    pcl::PointCloud<PointT> getShape();
};

template <typename PointT>
class ImportedPointCloudShape : public PointCloudShape<PointT> {
public:
    ImportedPointCloudShape(const std::string&);
    void generateShape(const std::string&);
};

#endif //POINTCLOUDSHAPES_H