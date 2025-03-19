#ifndef POINTCLOUDSHAPES_H
#define POINTCLOUDSHAPES_H

#include <pcl/io/pcd_io.h>

class IPointCloudShape {
protected:
    std::string id;
    pcl::PointCloud<pcl::PointXYZRGBNormal> shape;
    explicit IPointCloudShape(const std::string& id_) : id(id_) { }
public:
    std::string getId() { return id; }
    virtual pcl::PointCloud<pcl::PointXYZRGBNormal> getShape() { return shape; }
    virtual void generateShape() { }
    virtual ~IPointCloudShape() = default;
};

class ImportedPointCloudShape : public IPointCloudShape {
private:
    std::string filePath;
public:
    ImportedPointCloudShape(const std::string& id, const std::string& filePath) : IPointCloudShape(id), filePath {filePath} {}
    void generateShape() override {
        pcl::PointCloud<pcl::PointXYZ> tmp;
        if (pcl::io::loadPCDFile<pcl::PointXYZ> (filePath, tmp) == -1) //* load the file
        {
            PCL_ERROR("Couldn't read file test_pcd.pcd \n");
        }
        pcl::copyPointCloud(tmp, shape);
    }
};

#endif //POINTCLOUDSHAPES_H