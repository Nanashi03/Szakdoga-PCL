//
// Created by kristof on 2025.03.16..
//

#include "Model.h"

Model::Model() {}

void Model::addCloud(const IPointCloudShape& cloud_shape) {
    //if (std::ranges::find(clouds.begin(), clouds.end(), cloud_shape) == clouds.end()) return; //RAISE ERROR

    clouds.push_back(cloud_shape);
}

void Model::updateCloud(const IPointCloudShape& cloud_shape) {
    return;
}

void Model::removeCloud(const IPointCloudShape& cloud_shape) {
    return;
}