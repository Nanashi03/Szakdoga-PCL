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
    clouds[index].setColor(color);
}

PointCloudT::ConstPtr Model::selectCloud(const string& name) {
    for (int i = 0; i < clouds.size(); i++) {
        if (clouds[i].getId() == name) {
            colorCloud({255,0,0}, i);
            return clouds[i].getShape();
        }
    }
}
