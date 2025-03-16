#include "PointCloudShapes.h"

template<typename PointT>
ImportedPointCloudShape<PointT>::ImportedPointCloudShape(const std::string & id) : IPointCloudShape(id) {}

template<typename PointT>
pcl::PointCloud<PointT> ImportedPointCloudShape<PointT>::getShape() {
    return shape;
}

template<typename PointT>
void ImportedPointCloudShape<PointT>::generateShape(const std::string& filePath) {
    if (pcl::io::loadPCDFile<PointT> (filePath, this->shape) == -1) //* load the file
    {
        PCL_ERROR("Couldn't read file test_pcd.pcd \n");
    }
}

