//
// Created by kristof on 2025.03.13..
//

#ifndef POINTCLOUDSHAPES_H
#define POINTCLOUDSHAPES_H

#include <iostream>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

template <typename PointT>
class PointCloudShape {
protected:
    pcl::PointCloud<pcl::PCLPointField> shape;
    std::string id;
    PointCloudShape(const std::string&);
public:
    virtual void generateShape();
    PointT getShape();
};

template <typename PointT>
class ImportedPointCloudShape : public PointCloudShape<PointT> {
public:
    ImportedPointCloudShape(const std::string&);
    void generateShape(const std::string&);
};

#endif //POINTCLOUDSHAPES_H