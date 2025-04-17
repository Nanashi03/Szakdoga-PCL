//
// Created by kristof on 2025.03.23..
//

#include "PointCloudShapes.h"

#include <oneapi/tbb/task_arena.h>

#include <utility>

IPointCloudShape::IPointCloudShape(const string& id_, bool iF = false, float d = 1.0f) :
    id{id_},
    shapePtr { make_shared<PointCloudT>() },
    translationValues { Eigen::Vector3f::Zero() },
    isFilled { iF },
    areNormalsPresent { false },
    density { d },
    rotation { 0, 0, 0 },
    isColorable { true },
    isDensitable { true },
    isFillable { true }
{ }

void IPointCloudShape::transformPointCloudToCenter() {
    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(*shapePtr, centroid);
    Eigen::Affine3f translation = Eigen::Affine3f::Identity();
    translation.translation() << -centroid[0], -centroid[1], -centroid[2];
    pcl::transformPointCloud(*shapePtr, *shapePtr, translation);
}

void IPointCloudShape::calculateNormals()
{
    areNormalsPresent = true;

    for (int i=0; i<shapePtr->points.size(); i++)
    {
        cout << shapePtr->points[i].x << " " << shapePtr->points[i].y << " " << shapePtr->points[i].z << endl;
    }

    cout << endl;

    pcl::NormalEstimation<PointType, PointType> ne;
    ne.setInputCloud (shapePtr);
    pcl::search::KdTree<PointType>::Ptr tree { new pcl::search::KdTree<PointType> () };
    ne.setSearchMethod (tree);
    ne.setRadiusSearch (100);
    ne.compute (*shapePtr);

    for (int i=0; i<shapePtr->size(); ++i)
    {
        cout << shapePtr->points[i].normal_x << " " << shapePtr->points[i].normal_y << " " << shapePtr->points[i].normal_z << endl;
    }
}

bool IPointCloudShape::getAreNormalsPresent() const { return areNormalsPresent; }

bool IPointCloudShape::getIsFilled() const { return isFilled; }

string IPointCloudShape::getId() const { return id; }

string IPointCloudShape::getNormalId() const { return id + "_normals"; }

PointCloudT::Ptr IPointCloudShape::getShape() const { return shapePtr; }

const Eigen::Vector3f& IPointCloudShape::getTranslationValues() const { return translationValues; }

float IPointCloudShape::getDensity() const { return 1/density; }

int IPointCloudShape::getRotationAt(int ind) const { return rotation[ind]; }

bool IPointCloudShape::getIsColorable() const { return isColorable; }

bool IPointCloudShape::getIsFillable() const { return isFillable; }

bool IPointCloudShape::getIsDensitable() const { return isDensitable; }

vector<bool> IPointCloudShape::getShowLabels() const { return showLabels; }

vector<string> IPointCloudShape::getLabels() const { return labels; }

vector<float> IPointCloudShape::getDimensions() const { return dimensions; }

pcl::RGB IPointCloudShape::getColor() const
{
    if (shapePtr->size() > 0) return { shapePtr->points[0].r, shapePtr->points[0].g, shapePtr->points[0].b };
    return {0,0,0};
}

void IPointCloudShape::generateShape() { }

void IPointCloudShape::setColor(pcl::RGB color) {
    for (int i=0; i<shapePtr->size(); i++) {
        shapePtr->points[i].r = color.r;
        shapePtr->points[i].g = color.g;
        shapePtr->points[i].b = color.b;
    }
}

void IPointCloudShape::setIsFilled(bool isFilled) { this->isFilled = isFilled; }

void IPointCloudShape::setAreNormalsPresent(bool areNormalsPresent) { this->areNormalsPresent = areNormalsPresent; }

void IPointCloudShape::setDensity(int density) { this->density = 1/static_cast<float>(density); }

void IPointCloudShape::setRotationAt(int ind, int value) { this->rotation[ind] = value; }

void IPointCloudShape::setShape(PointCloudT::Ptr shape) { this->shapePtr = std::move(shape); }

void IPointCloudShape::addToTranslationValues(const Eigen::Vector3f& offSet) { translationValues += offSet; }

void IPointCloudShape::setDimensions(float x, float y, float z) { }
/*********************************************IMPORTED_CLOUD***********************************************************/
ImportedPointCloudShape::ImportedPointCloudShape(const string& id, string  filePath) :
    IPointCloudShape(id),
    filePath {std::move(filePath)}
{
    this->isFillable = false;
    this->isDensitable = false;
    this->labels = {};
    this->showLabels = {false, false, false};
    this->dimensions = {};
}

void ImportedPointCloudShape::generateShape() {
    pcl::PCLPointCloud2 cloud2;
    pcl::PCDReader reader;
    if (reader.readHeader(filePath, cloud2) != 0) {
        throw runtime_error("Error reading file: " + filePath);
    }
    bool constainRGB { false };
    bool constainNormals {false};
    for (auto field : cloud2.fields) {
        if (field.name == "rgb") constainRGB = true;
        if (field.name == "normal_x") constainNormals = true;
    }

    this->isColorable = !constainRGB;

    if (constainRGB && constainNormals) {
        pcl::io::loadPCDFile<PointType>(filePath, *shapePtr);
    } else if (constainRGB) {
        pcl::PointCloud<pcl::PointXYZRGB> tmp;
        pcl::io::loadPCDFile<pcl::PointXYZRGB>(filePath, tmp);
        pcl::copyPointCloud(tmp, *shapePtr);
    } else {
        cout << "IM HERE" << endl;
        pcl::PointCloud<pcl::PointXYZ> tmp;
        pcl::io::loadPCDFile<pcl::PointXYZ> (filePath, tmp);
        pcl::copyPointCloud(tmp, *shapePtr);
        setColor({ 255, 255, 255 });
    }

    transformPointCloudToCenter();
}
/******************************************RECTANGLE*******************************************************************/
RectanglePointCloudShape::RectanglePointCloudShape(const string& id, bool iF, float w, float h, float d) :
    IPointCloudShape(id, iF, 1/d),
    width {w},
    height {h}
{
    this->labels = {"Width", "Height"};
    this->showLabels = {true, true, false};
    this->dimensions = {w,h};
}

void RectanglePointCloudShape::generateShape() {
    shapePtr->points.clear();

    cout << "width: " << width << endl;
    cout << "height: " << height << endl;

    for (float i = 0; i <= width; i += density) {
        for (float j = 0; j <= height; j += density) {
            if (!isFilled && (i==0 || i>width-density || j==0 || j>height-density)) {
                PointType point {i, j, 0, 255, 255, 255};
                shapePtr->points.push_back(point);
            } else if (isFilled) {
                PointType point {i, j, 0, 255, 255, 255};
                shapePtr->points.push_back(point);
            }
        }
    }

    transformPointCloudToCenter();
}

void RectanglePointCloudShape::setDimensions(float width, float height, float z) {
    this->width = width;
    this->height = height;
}
/*********************************************CUBOID*******************************************************************/
CuboidPointCloudShape::CuboidPointCloudShape(const string& id, bool iF, float w, float h, float l, float d) :
    IPointCloudShape(id, iF, 1/d),
    width {w},
    height {h},
    length {l}
{
    this->labels = {"Width", "Height", "Length"};
    this->showLabels = {true, true, true};
    this->dimensions = {w,h,l};
}

void CuboidPointCloudShape::generateShape() {
    shapePtr->points.clear();

    for (float i = 0; i <= width; i += density) {
        for (float j = 0; j <= height; j += density) {
            for (float k = 0; k <= length; k += density) {
                if (i==0 || i>width-density || j==0 || j>height-density || k==0 || k>length-density) {
                    PointType point {i, j, k, 255, 255, 255};
                    shapePtr->points.push_back(point);
                } else if (isFilled) {
                    PointType point {i, j, k, 255, 255, 255};
                    shapePtr->points.push_back(point);
                }
            }
        }
    }

    transformPointCloudToCenter();
}

void CuboidPointCloudShape::setDimensions(float width, float height, float length) {
    this->width = width;
    this->height = height;
    this->length = length;
}
/*********************************************CIRCLE*******************************************************************/
CirclePointCloudShape::CirclePointCloudShape(const string& id, bool iF, float r, float d) :
    IPointCloudShape(id, iF, 1/d),
    radius {r}
{
    this->labels = {"Radius"};
    this->showLabels = {true, false, false};
    this->dimensions = {r};
}

void CirclePointCloudShape::generateShape() {
    shapePtr->points.clear();

    float degreeIntensity = 360*density / 20;

    if (fmodf(radius, density) > 0.0001)
        radius = density*static_cast<int>(radius / density);

    for (float i = 0; i <= 360.0f; i += degreeIntensity) {
        float rad = i * (M_PI / 180.0f);
        PointType point {radius*cos(rad), radius*sin(rad), 0, 255, 255, 255};
        shapePtr->points.push_back(point);
        if (isFilled) {
            for (float j = 0; j < radius; j += density) {
                PointType point {j*cos(rad), j*sin(rad), 0, 255, 255, 255};
                shapePtr->points.push_back(point);
            }
        }
    }

    transformPointCloudToCenter();
}

void CirclePointCloudShape::setDimensions(float radius, float y, float z) {
    this->radius = radius;
}
/*********************************************SPHERE*******************************************************************/
SpherePointCloudShape::SpherePointCloudShape(const string& id, bool iF, float r, float d) :
    IPointCloudShape(id,iF,1/d),
    radius {r}
{
    this->labels = {"Radius"};
    this->showLabels = {true, false, false};
    this->dimensions = {r};
}

void SpherePointCloudShape::generateShape() {
    shapePtr->points.clear();

    float degreeIntensity = 360*density / 20;
    for (float i = 0; i <= 180.0f; i += degreeIntensity) {
        for (float j = 0; j < 360.0f; j += degreeIntensity) {
            float radI = i * (M_PI / 180.0f), radJ = j * (M_PI / 180.0f);
            PointType point {radius*sin(radI)*cos(radJ), radius*sin(radI)*sin(radJ), radius*cos(radI), 255, 255, 255};
            shapePtr->points.push_back(point);
            if (isFilled) {
                for (float k = 0; k < radius; k += density) {
                    PointType point {k*sin(radI)*cos(radJ), k*sin(radI)*sin(radJ), k*cos(radI), 255, 255, 255};
                    shapePtr->points.push_back(point);
                }
            }
        }
    }

    transformPointCloudToCenter();
}

void SpherePointCloudShape::setDimensions(float radius, float y, float z) {
    this->radius = radius;
}
/*********************************************CYLINDER*****************************************************************/
CylinderPointCloudShape::CylinderPointCloudShape(const string& id, bool iF, float r, float h, float d) :
    IPointCloudShape(id,iF,1/d),
    radius {r},
    height {h}
{
    this->labels = {"Radius", "Height"};
    this->showLabels = {true, true, false};
    this->dimensions = {r,h};
}

void CylinderPointCloudShape::generateShape() {
    shapePtr->points.clear();

    float degreeIntensity = 360*density / 20;

    if (fmodf(radius, density) > 0.0001)
        radius = density*static_cast<int>(radius / density);

    for (float i = 0; i <= 360.0f; i += degreeIntensity) {
        float rad = i * (M_PI / 180.0f);
        for (float j = 0; j <= radius; j += density) {
            PointType point = {j*cos(rad), j*sin(rad), 0, 255, 255, 255};
            shapePtr->points.push_back(point);
            point.z = height;
            shapePtr->points.push_back(point);
        }
    }

    for (float i = 0; i <= 360.0f; i += degreeIntensity) {
        for (float j = density; j < height; j += density) {
            float rad = i * (M_PI / 180.0f);
            PointType point {radius*cos(rad), radius*sin(rad), j, 255, 255, 255};
            shapePtr->points.push_back(point);
            if (isFilled) {
                for (float k = 0; k < radius; k += density) {
                    PointType point {k*cos(rad), k*sin(rad), j, 255, 255, 255};
                    shapePtr->points.push_back(point);
                }
            }
        }
    }

    transformPointCloudToCenter();
}

void CylinderPointCloudShape::setDimensions(float radius, float height, float z) {
    this->radius = radius;
    this->height = height;
}
/***********************************************CONE*******************************************************************/
ConePointCloudShape::ConePointCloudShape(const string& id, bool iF, float r, float h, float d) :
    IPointCloudShape(id,iF,1/d),
    radius {r},
    height {h}
{
    this->labels = {"Radius", "Height"};
    this->showLabels = {true, true, false};
    this->dimensions = {r,h};
}

void ConePointCloudShape::generateShape() {
    shapePtr->points.clear();

    float degreeIntensity = 360*density / 20;

    if (fmodf(radius, density) > 0.0001)
        radius = density*static_cast<int>(radius / density);

    if (fmodf(height, density) > 0.0001)
        height = density*static_cast<int>(height / density);

    for (float i = 0; i <= 360.0f; i += degreeIntensity) {
        float rad = i * (M_PI / 180.0f);
        for (float j = 0; j <= radius; j += density) {
            PointType point = {j*cos(rad), j*sin(rad), 0, 255, 255, 255};
            shapePtr->points.push_back(point);
        }
    }

    for (float i = 0; i < 360.0f; i += degreeIntensity)
    {
        float radI = i * (M_PI / 180.0f);
        for (float u = density; u <= height; u += density)
        {
            float x = (height - u) / height * radius * cos(radI);
            float y = (height - u) / height * radius * sin(radI);
            PointType point {x, y, u, 255, 255, 255};
            shapePtr->points.push_back(point);
            if (isFilled) {
                for (float k = 0; k < radius; k += density) {
                    x = (height - u) / height * k * cos(radI);
                    y = (height - u) / height * k * sin(radI);
                    PointType point {x, y, u, 255, 255, 255};
                    shapePtr->points.push_back(point);
                }
            }
        }
    }

    transformPointCloudToCenter();
}

void ConePointCloudShape::setDimensions(float radius, float height, float z) {
    this->radius = radius;
    this->height = height;
}