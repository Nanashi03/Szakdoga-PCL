#ifndef POINTCLOUDSHAPES_H
#define POINTCLOUDSHAPES_H

#include <pcl/io/pcd_io.h>
#include <pcl/features/normal_3d.h>
#include <pcl/common/transforms.h>
#include <pcl/PCLPointCloud2.h>
#include <cmath>

using PointType = pcl::PointXYZRGBNormal;
using PointCloudT = pcl::PointCloud<PointType>;

using namespace std;

class IPointCloudShape {
protected:
    PointCloudT::Ptr shapePtr;
    Eigen::Vector3f translationValues;

    string id;
    float density;
    int rotation[3];

    bool isFilled;
    bool areNormalsPresent;

    bool isColorable, isFillable, isDensitable;
    vector<string> labels;
    vector<bool> showLabels;
    vector<float> dimensions;

    IPointCloudShape(const string&,bool,float);
    void transformPointCloudToCenter();
public:
    virtual void generateShape();
    void calculateNormals();

    void setColor(pcl::RGB);
    void setShape(PointCloudT::Ptr);
    void addToTranslationValues(const Eigen::Vector3f&);
    void setIsFilled(bool);
    void setAreNormalsPresent(bool);
    void setDensity(int);
    void setRotationAt(int,int);
    virtual void setDimensions(float,float,float);

    vector<bool> getShowLabels() const;
    vector<string> getLabels() const;
    vector<float> getDimensions() const;
    PointCloudT::Ptr getShape() const;
    const Eigen::Vector3f& getTranslationValues() const;
    pcl::RGB getColor() const;
    string getId() const;
    string getNormalId() const;
    float getDensity() const;
    int getRotationAt(int) const;
    bool getIsColorable() const;
    bool getIsFillable() const;
    bool getIsDensitable() const;
    bool getAreNormalsPresent() const;
    bool getIsFilled() const;

    virtual ~IPointCloudShape() = default;
};

class ImportedPointCloudShape : public IPointCloudShape {
private:
    string filePath;
public:
    ImportedPointCloudShape(const string&, string );
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