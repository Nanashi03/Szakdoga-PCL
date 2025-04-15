//asd
// Created by kristof on 2025.03.16..
//

#include "Model.h"

Model::Model() :
    selectedCloud {-1}
{}

void Model::addCloud(const shared_ptr<IPointCloudShape>& cloud_shape) {
    for (int i = 0; i < clouds.size(); i++) {
        if (clouds[i]->getId() == cloud_shape->getId() || clouds[i]->getNormalId() == cloud_shape->getId())
            throw std::runtime_error("ID already exists or conflicts with to be generated ones!");
    }

    clouds.push_back(cloud_shape);
}

void Model::updateSelectedCloudDimensions(float x, float y, float z) {
    if (selectedCloud == -1) return;

    clouds[selectedCloud]->setDimensions(x,y,z);
    clouds[selectedCloud]->generateShape();
}

void Model::updateSelectedCloudDensity(int density) {
    if (selectedCloud == -1) return;

    clouds[selectedCloud]->setDensity(density);
    clouds[selectedCloud]->generateShape();
}

void Model::updateSelectedCloudIsFilled(bool isFilled) {
    if (selectedCloud == -1) return;

    clouds[selectedCloud]->setIsFilled(isFilled);
    clouds[selectedCloud]->generateShape();
}

void Model::updateSelectedCloudAreNormalsPresent(bool areNormalsPresent) {
    if (selectedCloud == -1) return;

    clouds[selectedCloud]->setAreNormalsPresent(areNormalsPresent);
}

void Model::removeSelectedCloud() {
    if (selectedCloud == -1) return;
    clouds.erase(clouds.begin() + selectedCloud);
    deSelectCloud();
}

void Model::generateNormalsForSelectedCloud() {
    if (selectedCloud == -1 || clouds[selectedCloud]->getAreNormalsPresent()) return;
    clouds[selectedCloud]->calculateNormals();
}

void Model::colorSelectedCloud(pcl::RGB color) {
    if (selectedCloud == -1) return;
    clouds[selectedCloud]->setColor(color);
}

void Model::deSelectCloud() {
    selectedCloud = -1;
}

void Model::selectCloud(const string& name) {
    for (int i = 0; i < clouds.size(); i++) {
        if (clouds[i]->getId() == name) {
            selectedCloud = i;
            break;
        }
    }
}

BoundingBoxData Model::getBoundingBoxDataAroundSelectedCloud()
{
    if (selectedCloud == -1) return BoundingBoxData();

    PointCloudT::Ptr cloudPCAprojection (new PointCloudT);
    pcl::PCA<PointType> pca;
    pca.setInputCloud(clouds[selectedCloud]->getShape());
    pca.project(*clouds[selectedCloud]->getShape(), *cloudPCAprojection);

    Eigen::Matrix3f eigenVectorsPCA = pca.getEigenVectors();
    eigenVectorsPCA.col(2) = eigenVectorsPCA.col(0).cross(eigenVectorsPCA.col(1));

    Eigen::Matrix4f projectionTransform(Eigen::Matrix4f::Identity());
    projectionTransform.block<3,3>(0,0) = eigenVectorsPCA.transpose();
    projectionTransform.block<3,1>(0,3) = -1.f * (projectionTransform.block<3,3>(0,0) * pca.getEigenValues().head<3>());

    PointCloudT::Ptr cloudPointsProjected (new PointCloudT);
    pcl::transformPointCloud(*clouds[selectedCloud]->getShape(), *cloudPointsProjected, projectionTransform);

    PointType minPoint, maxPoint;
    pcl::getMinMax3D(*cloudPointsProjected, minPoint, maxPoint);
    const Eigen::Vector3f meanDiagonal = 0.5f*(maxPoint.getVector3fMap() + minPoint.getVector3fMap());

    BoundingBoxData bboxData;
    bboxData.bboxQuaternion = eigenVectorsPCA;
    bboxData.bboxTransform = eigenVectorsPCA * meanDiagonal + pca.getEigenValues().head<3>();
    bboxData.width = maxPoint.x - minPoint.x;
    bboxData.height = maxPoint.y - minPoint.y;
    bboxData.depth = maxPoint.z - minPoint.z;

    return bboxData;
}

PointCloudT::ConstPtr Model::translateSelectedCloud(float x, float y, float z) {
    if (selectedCloud == -1) return nullptr;

    Eigen::Affine3f transform = Eigen::Affine3f::Identity();
    transform.translation() << x, y, z;
    PointCloudT::Ptr translatedCloud (new PointCloudT);
    pcl::transformPointCloud(*clouds[selectedCloud]->getShape(), *translatedCloud, transform);

    clouds[selectedCloud]->setShape(translatedCloud);
    return clouds[selectedCloud]->getShape();
}

PointCloudT::ConstPtr Model::rotateSelectedCloud(float angle, char axis) {
    if (selectedCloud == -1) return nullptr;
    float angleRad = angle * M_PI / 180;

    Eigen::Affine3f transform_2 = Eigen::Affine3f::Identity();
    if (axis == 'x') transform_2.rotate (Eigen::AngleAxisf (angleRad, Eigen::Vector3f::UnitX()));
    else if (axis == 'y') transform_2.rotate (Eigen::AngleAxisf (angleRad, Eigen::Vector3f::UnitY()));
    else if (axis == 'z') transform_2.rotate (Eigen::AngleAxisf (angleRad, Eigen::Vector3f::UnitZ()));

    PointCloudT::Ptr rotatedCloud (new PointCloudT);
    pcl::transformPointCloud(*clouds[selectedCloud]->getShape(), *rotatedCloud, transform_2);

    clouds[selectedCloud]->setShape(rotatedCloud);
    return clouds[selectedCloud]->getShape();
}

EditCloudData Model::getEditCloudData()
{
    if (selectedCloud == -1) return EditCloudData();

    EditCloudData data;
    data.name = getSelectedCloudName();
    data.rgb = { clouds[selectedCloud]->getColor().r, clouds[selectedCloud]->getColor().g, clouds[selectedCloud]->getColor().b };
    data.isFilled = clouds[selectedCloud]->getIsFilled();
    data.areNormalsPresent = clouds[selectedCloud]->getAreNormalsPresent();
    data.dim = clouds[selectedCloud]->getDimensions();
    data.labels = clouds[selectedCloud]->getLabels();
    data.density = clouds[selectedCloud]->getDensity();

    data.showFilledEdit = clouds[selectedCloud]->getIsFillable();
    data.showColorEdit = clouds[selectedCloud]->getIsColorable();
    data.showDensityEdit = clouds[selectedCloud]->getIsDensitable();
    data.showLabels = clouds[selectedCloud]->getShowLabels();

    return data;
}

PointCloudT::ConstPtr Model::getSelectedCloudShape() {
    if (selectedCloud == -1) return nullptr;
    return clouds[selectedCloud]->getShape();
}

string Model::getSelectedCloudName() {
    if (selectedCloud == -1) return "";
    return clouds[selectedCloud]->getId();
}

string Model::getSelectedCloudNormalsName() {
    if (selectedCloud == -1) return "";
    return clouds[selectedCloud]->getNormalId();
}

bool Model::isCloudSelected() {
    return selectedCloud != -1;
}

bool Model::getSelectedCloudAreNormalsPresent() {
    if (selectedCloud == -1) return false;
    return clouds[selectedCloud]->getAreNormalsPresent();
}
