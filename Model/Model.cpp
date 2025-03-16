//
// Created by kristof on 2025.03.16..
//

#include "Model.h"

Model::Model() {}

void Model::addCloud(std::shared_ptr<IPointCloudShape> cloud_shape) {
    if (std::ranges::find(clouds.begin(), clouds.end(), cloud_shape) == clouds.end()) return; //RAISE ERROR

    clouds.push_back(cloud_shape);
}

void Model::updateCloud(std::shared_ptr<IPointCloudShape> cloud_shape) {
    return;
}

void Model::removeCloud(std::shared_ptr<IPointCloudShape> cloud_shape) {
    return;
}