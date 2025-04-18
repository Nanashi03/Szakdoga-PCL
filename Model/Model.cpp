//asd
// Created by kristof on 2025.03.16..
//

#include "Model.h"

Model::Model() :
    selectedCloud {-1}
{}

void Model::addCloud(const shared_ptr<IPointCloudShape>& cloud_shape) {
    if (cloud_shape->getId() == "BBOX") throw runtime_error("ID already exists or conflicts with to be generated ones!");
    for (int i = 0; i < clouds.size(); i++) {
        if (clouds[i]->getId() == cloud_shape->getId() || clouds[i]->getNormalId() == cloud_shape->getId())
            throw runtime_error("ID already exists or conflicts with to be generated ones!");
    }

    clouds.push_back(cloud_shape);
}

void Model::exportClouds(const string& newFilePath) {
    PointCloudT::Ptr mergedCloud  { make_shared<PointCloudT>() };
    for (int i = 0; i < clouds.size(); i++) {
        *mergedCloud += *clouds[i]->getShape();
    }
    mergedCloud->width = mergedCloud->points.size();
    mergedCloud->height = 1;
    mergedCloud->is_dense = true; //?????????????????????????????????????????????no NaN's

    if (pcl::io::savePCDFileASCII(newFilePath, *mergedCloud) == -1)
        throw runtime_error("Failed to save file :" + newFilePath);
}

void Model::updateSelectedCloudDimensions(float x, float y, float z) {
    if (selectedCloud == -1) return;

    clouds[selectedCloud]->scale(x,y,z);
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

void Model::updateSelectedCloudAreNormalsShown(bool areNormalsShown) {
    if (selectedCloud == -1) return;

    clouds[selectedCloud]->setAreNormalsShown(areNormalsShown);
}

void Model::removeSelectedCloud() {
    if (selectedCloud == -1) return;
    clouds.erase(clouds.begin() + selectedCloud);
    deSelectCloud();
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
        if (clouds[i]->getId() == name || clouds[i]->getNormalId() == name) {
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

void Model::translateSelectedCloud(float x, float y, float z, Eigen::Affine3f& transform) {
    if (selectedCloud == -1) return;

    transform = Eigen::Affine3f::Identity();
    transform.translation() << x, y, z;
    pcl::transformPointCloud(*clouds[selectedCloud]->getShape(), *clouds[selectedCloud]->getShape(), transform);

    clouds[selectedCloud]->addToTranslationValues({x, y, z});
}

void Model::rotateSelectedCloud(int angle, char axis, Eigen::Affine3f& fullTransform) {
    if (selectedCloud == -1) return;

    Eigen::Affine3f rotation { Eigen::Affine3f::Identity() };
    float offsetRad = 0.0f;
    if (axis == 'x') {
        offsetRad = (float)(angle - clouds[selectedCloud]->getRotationAt(0)) * M_PI / 180.0f;
        rotation.rotate(Eigen::AngleAxisf(offsetRad, Eigen::Vector3f::UnitX()));
        clouds[selectedCloud]->setRotationAt(0, angle);
    } else if (axis == 'y') {
        offsetRad = (float)(angle - clouds[selectedCloud]->getRotationAt(1)) * M_PI / 180.0f;
        rotation.rotate(Eigen::AngleAxisf(offsetRad, Eigen::Vector3f::UnitY()));
        clouds[selectedCloud]->setRotationAt(1, angle);
    } else if (axis == 'z') {
        offsetRad = (float)(angle - clouds[selectedCloud]->getRotationAt(2)) * M_PI / 180.0f;
        rotation.rotate(Eigen::AngleAxisf(offsetRad, Eigen::Vector3f::UnitZ()));
        clouds[selectedCloud]->setRotationAt(2, angle);
    }

    Eigen::Affine3f toOrigin = Eigen::Affine3f::Identity();
    toOrigin.translate(-clouds[selectedCloud]->getTranslationValues());
    Eigen::Affine3f backToCentroid = Eigen::Affine3f::Identity();
    backToCentroid.translate(clouds[selectedCloud]->getTranslationValues());

    fullTransform = backToCentroid * rotation * toOrigin;

    pcl::transformPointCloud(*clouds[selectedCloud]->getShape(), *clouds[selectedCloud]->getShape(), fullTransform);

    clouds[selectedCloud]->addToRotationMatrix(rotation);
}

EditCloudData Model::getEditCloudData()
{
    if (selectedCloud == -1) return EditCloudData();

    EditCloudData data;
    data.name = getSelectedCloudName();
    data.rgb = { clouds[selectedCloud]->getColor().r, clouds[selectedCloud]->getColor().g, clouds[selectedCloud]->getColor().b };
    data.isFilled = clouds[selectedCloud]->getIsFilled();
    data.areNormalsShown = clouds[selectedCloud]->getAreNormalsShown();
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

bool Model::getSelectedCloudAreNormalsShown() {
    if (selectedCloud == -1) return false;
    return clouds[selectedCloud]->getAreNormalsShown();
}
