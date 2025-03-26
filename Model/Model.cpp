//asd
// Created by kristof on 2025.03.16..
//

#include "Model.h"

Model::Model() :
    selectedCloud {-1}
{}

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

void Model::colorCloud(pcl::RGB color, int index) {
    clouds[index].setColor({0,0,0});
}

IPointCloudShape Model::selectCloud(const string& name) {
    for (IPointCloudShape cloud : clouds) {
        if (cloud.getId() == name) {
            return cloud;
        }
    }
}