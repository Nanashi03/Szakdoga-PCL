//
// Created by kristof on 2025.03.16..
//

#include "Model.h"

template<typename PointT>
Model<PointT>::Model() {}

template<typename PointT>
void Model<PointT>::addCloud(const PointCloudShape<PointT>& cloud_shape) {
    if (std::find(clouds.begin(), clouds.end(), cloud_shape) == clouds.end()) return; //RAISE ERROR

    clouds.push_back(cloud_shape);
}

template<typename PointT>
void Model<PointT>::updateCloud(const PointCloudShape<PointT>& cloud_shape) {
    return;
}

template<typename PointT>
void Model<PointT>::removeCloud(const PointCloudShape<PointT>& cloud_shape) {
    return;
}