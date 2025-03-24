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
        PointCloudT shape;
        float intensity;
        IPointCloudShape(const string&);
        IPointCloudShape(const string&,float);
    public:
        string getId();
        PointCloudT getShape();
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
        RectanglePointCloudShape(const string&, float,float,float);
        void generateShape() override;
};

class CuboidPointCloudShape : public IPointCloudShape {
    private:
        float width, height, length;
    public:
        CuboidPointCloudShape(const string&, float,float,float,float);
        void generateShape() override;
};

class CirclePointCloudShape : public IPointCloudShape {
    private:
        float radius;
    public:
        CirclePointCloudShape(const string&, float,float);
        void generateShape() override;
};

class SpherePointCloudShape : public IPointCloudShape {
    private:
        float radius;
    public:
        SpherePointCloudShape(const string&, float,float);
        void generateShape() override;
};

#endif //POINTCLOUDSHAPES_H