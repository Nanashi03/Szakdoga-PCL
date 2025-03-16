#ifndef POINTCLOUDSHAPES_H
#define POINTCLOUDSHAPES_H

#include <pcl/io/pcd_io.h>

class IPointCloudShape {
protected:
    std::string id;
    explicit IPointCloudShape(const std::string& id_) : id(id_) {}
public:
    std::string getId() { return id; }
    virtual void generateShape();
    virtual ~IPointCloudShape() = default;
};

template <typename PointT>
class ImportedPointCloudShape : public IPointCloudShape {
private:
    pcl::PointCloud<PointT> shape;
public:
    explicit ImportedPointCloudShape(const std::string&);
    void generateShape(const std::string&);
    pcl::PointCloud<PointT> getShape();
};

#endif //POINTCLOUDSHAPES_H