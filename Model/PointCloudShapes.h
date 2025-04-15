#ifndef POINTCLOUDSHAPES_H
#define POINTCLOUDSHAPES_H

#include <pcl/io/pcd_io.h>
#include <pcl/features/normal_3d.h>
#include <pcl/PCLPointCloud2.h>
#include <cmath>

using PointType = pcl::PointXYZRGBNormal;
using PointCloudT = pcl::PointCloud<PointType>;

using namespace std;

class IPointCloudShape {
protected:
    PointCloudT::Ptr shapePtr;
    string id;
    float density;

    bool isFilled;
    bool areNormalsPresent;

    bool isColorable, isFillable, isDensitable;
    vector<string> labels;
    vector<bool> showLabels;
    vector<float> dimensions;

    IPointCloudShape(const string&,bool,float);
public:
    virtual void generateShape();
    void calculateNormals();

    void setColor(pcl::RGB);
    void setShape(PointCloudT::Ptr);
    virtual void setDimensions(float,float,float);

    bool getAreNormalsPresent() const;
    bool getIsFilled() const;
    string getId() const;
    string getNormalId() const;
    PointCloudT::Ptr getShape() const;

    float getDensity() const;
    bool getIsColorable() const;
    bool getIsFillable() const;
    bool getIsDensitable() const;
    vector<bool> getShowLabels() const;
    vector<string> getLabels() const;
    vector<float> getDimensions() const;
    pcl::RGB getColor() const;

    virtual ~IPointCloudShape() = default;
};

class ImportedPointCloudShape : public IPointCloudShape {
private:
    string filePath;
public:
    ImportedPointCloudShape(const string&, const string&);
    void generateShape() override;
};

class RectanglePointCloudShape : public IPointCloudShape {
private:
    float width, height;
public:
    RectanglePointCloudShape(const string&, bool,float,float,float);
    void generateShape() override;
    void setDimensions(float,float,float) override;
};

class CuboidPointCloudShape : public IPointCloudShape {
private:
    float width, height, length;
public:
    CuboidPointCloudShape(const string&, bool,float,float,float,float);
    void generateShape() override;
    void setDimensions(float,float,float) override;
};

class CirclePointCloudShape : public IPointCloudShape {
private:
    float radius;
public:
    CirclePointCloudShape(const string&, bool,float,float);
    void generateShape() override;
    void setDimensions(float,float,float) override;
};

class SpherePointCloudShape : public IPointCloudShape {
private:
    float radius;
public:
    SpherePointCloudShape(const string&, bool,float,float);
    void generateShape() override;
    void setDimensions(float,float,float) override;
};

class CylinderPointCloudShape : public IPointCloudShape {
private:
    float radius, height;
public:
    CylinderPointCloudShape(const string&, bool,float,float,float);
    void generateShape() override;
    void setDimensions(float,float,float) override;
};

class ConePointCloudShape : public IPointCloudShape {
private:
    float radius, height;
public:
    ConePointCloudShape(const string&, bool,float,float,float);
    void generateShape() override;
    void setDimensions(float,float,float) override;
};

#endif //POINTCLOUDSHAPES_H