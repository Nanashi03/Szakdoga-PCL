//
// Created by kristof on 2025.03.13..
//

#include "PointCloudShapes.h"

template<typename PointT>
PointCloudShape<PointT>::PointCloudShape(const std::string& id_) :
    id {id_}
{ }

template<typename PointT>
PointT PointCloudShape<PointT>::getShape() {
    return shape;
}

template<typename PointT>
ImportedPointCloudShape<PointT>::ImportedPointCloudShape(const std::string & id) :
    PointCloudShape<PointT>(id) {}


template<typename PointT>
void ImportedPointCloudShape<PointT>::generateShape(const std::string& filePath) {
    if (pcl::io::loadPCDFile<PointT> (filePath, this->shape) == -1) //* load the file
    {
        PCL_ERROR("Couldn't read file test_pcd.pcd \n");
    }
}

