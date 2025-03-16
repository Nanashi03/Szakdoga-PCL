#ifndef POINTCLOUDSHAPES_H
#define POINTCLOUDSHAPES_H

#include <pcl/io/pcd_io.h>

class IPointCloudShape {
protected:
    std::string id;
    explicit IPointCloudShape(const std::string& id_) : id(id_) { }
public:
    std::string getId() { return id; }
    virtual void generateShape() { }
    virtual ~IPointCloudShape() = default;
};

template <typename PointT>
class ImportedPointCloudShape : public IPointCloudShape {
private:
    pcl::PointCloud<PointT> shape;
    std::string filePath;
public:
    ImportedPointCloudShape(const std::string& id, const std::string& filePath) : IPointCloudShape(id), filePath {filePath} {}
    pcl::PointCloud<PointT> getShape() { return shape; }
    void generateShape() override {
        if (pcl::io::loadPCDFile<PointT> (filePath, this->shape) == -1) //* load the file
        {
            PCL_ERROR("Couldn't read file test_pcd.pcd \n");
        }
    }
};

#endif //POINTCLOUDSHAPES_H