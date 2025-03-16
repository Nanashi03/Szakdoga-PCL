//
// Created by kristof on 2025.03.16..
//

#include "Controller.h"

template<typename PointT>
Controller<PointT>::Controller() {}

template<typename PointT>
void Controller<PointT>::importCloud(const std::string& filePath) {
    ImportedPointCloudShape<PointT> cloud("id");
    cloud.generateShape(filePath);
}

