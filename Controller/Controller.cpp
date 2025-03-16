//
// Created by kristof on 2025.03.16..
//

#include "Controller.h"

Controller::Controller() {}

void Controller::importCloud(const std::string& filePath) {
    std::shared_ptr<ImportedPointCloudShape<pcl::PointXYZ>> cloud = std::make_shared<ImportedPointCloudShape<pcl::PointXYZ>>("id1", filePath);
    cloud->generateShape();
    model.addCloud(cloud);
}

