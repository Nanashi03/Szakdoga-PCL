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

PointCloudT::ConstPtr Model::deSelectCloud() {
    if (selectedCloud == -1) return nullptr;
    clouds[selectedCloud].setColor({255,255,255});
    int tmp = selectedCloud;
    selectedCloud = -1;
    return clouds[tmp].getShape();
}

PointCloudT::ConstPtr Model::selectCloud(const string& name) {
    for (int i = 0; i < clouds.size(); i++) {
        if (clouds[i].getId() == name) {
            selectedCloud = i;
            break;
        }
    }
    colorCloud({255,0,0}, selectedCloud);
    return clouds[selectedCloud].getShape();
}

PointCloudT::ConstPtr Model::translateSelectedCloud(float x, float y, float z) {
    if (selectedCloud == -1) return nullptr;

    Eigen::Affine3f transform_2 = Eigen::Affine3f::Identity();
    transform_2.translation() << x, y, z;
    PointCloudT::Ptr translatedCloud (new PointCloudT);
    pcl::transformPointCloud(*clouds[selectedCloud].getShape(), *translatedCloud, transform_2);

    clouds[selectedCloud].setShape(translatedCloud);
    return clouds[selectedCloud].getShape();
}

PointCloudT::ConstPtr Model::rotateSelectedCloud(float angle, char axis) {
    if (selectedCloud == -1) return nullptr;
    float angleRad = angle * M_PI / 180;

    Eigen::Affine3f transform_2 = Eigen::Affine3f::Identity();
    if (axis == 'x') transform_2.rotate (Eigen::AngleAxisf (angleRad, Eigen::Vector3f::UnitX()));
    else if (axis == 'y') transform_2.rotate (Eigen::AngleAxisf (angleRad, Eigen::Vector3f::UnitY()));
    else if (axis == 'z') transform_2.rotate (Eigen::AngleAxisf (angleRad, Eigen::Vector3f::UnitZ()));

    PointCloudT::Ptr rotatedCloud (new PointCloudT);
    pcl::transformPointCloud(*clouds[selectedCloud].getShape(), *rotatedCloud, transform_2);

    clouds[selectedCloud].setShape(rotatedCloud);
    return clouds[selectedCloud].getShape();
}

string Model::getSelectedCloudName()
{
    if (selectedCloud == -1) return "";
    return clouds[selectedCloud].getId();
}

bool Model::isCloudSelected()
{
    return selectedCloud != -1;
}