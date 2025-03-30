#ifndef POINTCLOUDSHAPES_H
#define POINTCLOUDSHAPES_H

#include <pcl/io/pcd_io.h>
#include <cmath>

typedef pcl::PointXYZRGBNormal PointType;
typedef pcl::PointCloud<PointType> PointCloudT;

using namespace std;

class IPointCloudShape {
    protected:
        string id;
        bool isFilled;
        PointCloudT::Ptr shapePtr;
        float intensity;
        IPointCloudShape(const string&,bool,float);
    public:
        string getId();
        PointCloudT::Ptr getShape();
        void setColor(pcl::RGB);
        virtual void generateShape();
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
};

class CuboidPointCloudShape : public IPointCloudShape {
    private:
        float width, height, length;
    public:
        CuboidPointCloudShape(const string&, bool,float,float,float,float);
        void generateShape() override;
};

class CirclePointCloudShape : public IPointCloudShape {
    private:
        float radius;
    public:
        CirclePointCloudShape(const string&, bool,float,float);
        void generateShape() override;
};

class SpherePointCloudShape : public IPointCloudShape {
    private:
        float radius;
    public:
        SpherePointCloudShape(const string&, bool,float,float);
        void generateShape() override;
};

class CylinderPointCloudShape : public IPointCloudShape {
private:
    float radius, height;
public:
    CylinderPointCloudShape(const string&, bool,float,float,float);
    void generateShape() override;
};

class ConePointCloudShape : public IPointCloudShape {
private:
    float radius, height;
public:
    ConePointCloudShape(const string&, bool,float,float,float);
    void generateShape() override;
};

#endif //POINTCLOUDSHAPES_H