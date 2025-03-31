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

void Model::createBoundingBoxAround(int index, BoundingBoxData& bboxData)
{
    /*Eigen::Vector4f pcaCentroid;
    pcl::compute3DCentroid(*clouds[index].getShape(), pcaCentroid);
    Eigen::Matrix3f covariance;
    computeCovarianceMatrixNormalized(*clouds[index].getShape(), pcaCentroid, covariance);
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance, Eigen::ComputeEigenvectors);
    Eigen::Matrix3f eigenVectorsPCA = eigen_solver.eigenvectors();
    eigenVectorsPCA.col(2) = eigenVectorsPCA.col(0).cross(eigenVectorsPCA.col(1));*/

    PointCloudT::Ptr cloudPCAprojection (new PointCloudT);
    pcl::PCA<PointType> pca;
    pca.setInputCloud(clouds[index].getShape());
    pca.project(*clouds[index].getShape(), *cloudPCAprojection);
    std::cerr << std::endl << "EigenVectors: " << pca.getEigenVectors() << std::endl;
    std::cerr << std::endl << "EigenValues: " << pca.getEigenValues() << std::endl;

    Eigen::Matrix3f eigenVectorsPCA = pca.getEigenVectors();
    eigenVectorsPCA.col(2) = eigenVectorsPCA.col(0).cross(eigenVectorsPCA.col(1));

    Eigen::Matrix4f projectionTransform(Eigen::Matrix4f::Identity());
    projectionTransform.block<3,3>(0,0) = eigenVectorsPCA.transpose();
    projectionTransform.block<3,1>(0,3) = -1.f * (projectionTransform.block<3,3>(0,0) * pca.getEigenValues().head<3>());

    PointCloudT::Ptr cloudPointsProjected (new PointCloudT);
    pcl::transformPointCloud(*clouds[index].getShape(), *cloudPointsProjected, projectionTransform);

    PointType minPoint, maxPoint;
    pcl::getMinMax3D(*cloudPointsProjected, minPoint, maxPoint);
    const Eigen::Vector3f meanDiagonal = 0.5f*(maxPoint.getVector3fMap() + minPoint.getVector3fMap());

    bboxData.bboxQuaternion = eigenVectorsPCA;
    bboxData.bboxTransform = eigenVectorsPCA * meanDiagonal + pca.getEigenValues().head<3>();
    bboxData.width = maxPoint.x - minPoint.x;
    bboxData.height = maxPoint.y - minPoint.y;
    bboxData.depth = maxPoint.z - minPoint.z;
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