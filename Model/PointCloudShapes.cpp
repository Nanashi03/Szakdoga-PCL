#include "PointCloudShapes.h"

IPointCloudShape::IPointCloudShape(const string& id_, bool iF = false, float d = 1.0f) :
    id{id_},
    shapePtr { make_shared<PointCloudT>() },
    translationValues { Eigen::Vector3f::Zero() },
    rotationValues { Eigen::Vector3i::Zero() },
    currentRotation { Eigen::Affine3f::Identity() },
    color { 255, 255, 255 },
    isFilled { iF },
    areNormalsShown { false },
    isBoundingBoxDataCalculated { false },
    density { d },
    isColorable { true },
    isDensitable { true },
    isFillable { true }
{  }

void IPointCloudShape::transformPointCloudToCenter() {
    Eigen::Vector4f minPt, maxPt;
    pcl::getMinMax3D(*shapePtr, minPt, maxPt);
    Eigen::Vector3f center = (minPt.head<3>() + maxPt.head<3>()) * 0.5f;
    Eigen::Affine3f translation = Eigen::Affine3f::Identity();
    translation.translation() << -center[0], -center[1], -center[2];
    cout << "TRANSLATION BACK TO ZERO BY: (" << -center[0] << ", " << -center[1] << ", " << -center[2] << ")" << endl;
    pcl::transformPointCloud(*shapePtr, *shapePtr, translation);
}

void IPointCloudShape::transformPointCloudBackToOriginal() {
    Eigen::Affine3f backToCentroid = Eigen::Affine3f::Identity();
    backToCentroid.translate(getTranslationValues());
    cout << "TRANSLATION BACK TO ORIGINAL BY: (" << translationValues[0] << ", " << translationValues[1] << ", " << translationValues[2] << ")" << endl;
    Eigen::Affine3f fullTransform = backToCentroid * currentRotation;
    pcl::transformPointCloud(*shapePtr, *shapePtr, fullTransform);
}

void IPointCloudShape::calculateNormals() {
    cout << "CALCULATING NORMALS..." << endl;
    pcl::NormalEstimation<PointType, pcl::Normal> ne;
    pcl::search::KdTree<PointType>::Ptr tree(new pcl::search::KdTree<PointType>());
    tree->setInputCloud(shapePtr);
    ne.setInputCloud(shapePtr);
    ne.setSearchMethod(tree);
    ne.setKSearch(20);

    auto resultCloud(make_shared<pcl::PointCloud<pcl::Normal>>());
    ne.compute(*resultCloud);
    cout << "CALCULATING NORMALS DONE" << endl;
    for (int i = 0; i < resultCloud->size(); i++) {
        shapePtr->points[i].normal_x = resultCloud->points[i].normal_x;
        shapePtr->points[i].normal_y = resultCloud->points[i].normal_y;
        shapePtr->points[i].normal_z = resultCloud->points[i].normal_z;
    }
}

void IPointCloudShape::calculateBoundingBoxData() {
    if (isBoundingBoxDataCalculated) return;
    Eigen::Vector4f minPt, maxPt;
    cout << "CALCULATING BOUNDING BOX DATA..." << endl;
    pcl::getMinMax3D(*shapePtr, minPt, maxPt);

    boundingBoxData.minX = minPt.x();
    boundingBoxData.minY = minPt.y();
    boundingBoxData.minZ = minPt.z();
    boundingBoxData.maxX = maxPt.x();
    boundingBoxData.maxY = maxPt.y();
    boundingBoxData.maxZ = maxPt.z();

    cout << "CALCULATING BOUNDING BOX DATA DONE" << endl;
    isBoundingBoxDataCalculated = true;
}

bool IPointCloudShape::getAreNormalsShown() const { return areNormalsShown; }

bool IPointCloudShape::getIsFilled() const { return isFilled; }

const string& IPointCloudShape::getId() const { return id; }

string IPointCloudShape::getNormalId() const { return id + "_normals"; }

PointCloudT::Ptr IPointCloudShape::getShape() const { return shapePtr; }

const BoundingBoxData& IPointCloudShape::getBoundingBoxData() {
    calculateBoundingBoxData();
    return boundingBoxData;
}

const Eigen::Vector3f& IPointCloudShape::getTranslationValues() const { return translationValues; }

const Eigen::Affine3f& IPointCloudShape::getCurrentRotation() const { return currentRotation; }

float IPointCloudShape::getDensity() const { return 1/density; }

int IPointCloudShape::getRotationAt(int ind) const { return rotationValues[ind]; }

bool IPointCloudShape::getIsColorable() const { return isColorable; }

bool IPointCloudShape::getIsFillable() const { return isFillable; }

bool IPointCloudShape::getIsDensitable() const { return isDensitable; }

vector<bool> IPointCloudShape::getShowLabels() const { return showLabels; }

vector<string> IPointCloudShape::getLabels() const { return labels; }

vector<float> IPointCloudShape::getDimensions() const { return dimensions; }

pcl::RGB IPointCloudShape::getColor() const { return color; }

void IPointCloudShape::generateShape() { }

void IPointCloudShape::setColor(pcl::RGB color) {
    if (!isColorable) return;
    this->color = color;
    for (int i=0; i<shapePtr->size(); i++) {
        shapePtr->points[i].r = color.r;
        shapePtr->points[i].g = color.g;
        shapePtr->points[i].b = color.b;
    }
}

void IPointCloudShape::setIsFilled(bool isFilled) { this->isFilled = isFilled; }

void IPointCloudShape::setAreNormalsShown(bool areNormalsShown) { this->areNormalsShown = areNormalsShown; }

void IPointCloudShape::setDensity(int density) { this->density = 1/static_cast<float>(density); }

void IPointCloudShape::setRotationAt(int ind, int value) { this->rotationValues[ind] = value; }

void IPointCloudShape::setShape(PointCloudT::Ptr shape) {
    shapePtr->clear();  pcl::copyPointCloud(*shape, *shapePtr);
    calculateBoundingBoxData();
}

void IPointCloudShape::addToTranslationValues(const Eigen::Vector3f& offSet) {
    translationValues += offSet;

    boundingBoxData.minX += offSet.x();
    boundingBoxData.minY += offSet.y();
    boundingBoxData.minZ += offSet.z();
    boundingBoxData.maxX += offSet.x();
    boundingBoxData.maxY += offSet.y();
    boundingBoxData.maxZ += offSet.z();
}

void IPointCloudShape::addToRotationMatrix(const Eigen::Affine3f& deltaRotation) {
    currentRotation = deltaRotation * currentRotation;
    isBoundingBoxDataCalculated = false;
}

void IPointCloudShape::scale(float x, float y, float z) { }
/*********************************************IMPORTED_CLOUD***********************************************************/
ImportedPointCloudShape::ImportedPointCloudShape(const string& id, string  filePath) :
    IPointCloudShape(id),
    filePath {std::move(filePath)}
{
    this->isFillable = false;
    this->isDensitable = false;
    this->isColorable = false;
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
        if (pcl::io::loadPCDFile<PointType>(filePath, *shapePtr) != 0)
            throw runtime_error("Error reading file: " + filePath);
    } else if (constainRGB) {
        pcl::PointCloud<pcl::PointXYZRGB> tmp;
        if (pcl::io::loadPCDFile<pcl::PointXYZRGB>(filePath, tmp) != 0)
            throw runtime_error("Error reading file: " + filePath);
        pcl::copyPointCloud(tmp, *shapePtr);
    } else {
        pcl::PointCloud<pcl::PointXYZ> tmp;
        if (pcl::io::loadPCDFile<pcl::PointXYZ>(filePath, tmp) != 0)
            throw runtime_error("Error reading file: " + filePath);
        pcl::copyPointCloud(tmp, *shapePtr);
        setColor({ color.r, color.g, color.b });
    }

    if (!constainNormals) calculateNormals();
    calculateBoundingBoxData();
}

void ImportedPointCloudShape::scale(float x,float y,float z) {
    return;
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

    for (float i = 0; i <= width; i += density) {
        for (float j = 0; j <= height; j += density) {
            if (!isFilled && (i==0 || i>width-density || j==0 || j>height-density)) {
                PointType point {i, j, 0, color.r, color.g, color.b};
                shapePtr->points.push_back(point);
            } else if (isFilled) {
                PointType point {i, j, 0, color.r, color.g, color.b};
                shapePtr->points.push_back(point);
            }
        }
    }

    transformPointCloudToCenter();
    transformPointCloudBackToOriginal();
    calculateNormals();
    calculateBoundingBoxData();
}

void RectanglePointCloudShape::scale(float width, float height, float z) {
    this->width = width;
    this->height = height;
    this->isBoundingBoxDataCalculated = false;
    this->generateShape();
    this->dimensions = {this->width,this->height};
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
                    PointType point {i, j, k, color.r, color.g, color.b};
                    shapePtr->points.push_back(point);
                } else if (isFilled) {
                    PointType point {i, j, k, color.r, color.g, color.b};
                    shapePtr->points.push_back(point);
                }
            }
        }
    }

    transformPointCloudToCenter();
    transformPointCloudBackToOriginal();
    calculateNormals();
    calculateBoundingBoxData();
}

void CuboidPointCloudShape::scale(float width, float height, float length) {
    this->width = width;
    this->height = height;
    this->length = length;
    this->isBoundingBoxDataCalculated = false;
    this->generateShape();
    this->dimensions = {this->width,this->height,this->length};
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
        PointType point {radius*cos(rad), radius*sin(rad), 0, color.r, color.g, color.b};
        shapePtr->points.push_back(point);
        if (isFilled) {
            for (float j = 0; j < radius; j += density) {
                PointType point {j*cos(rad), j*sin(rad), 0, color.r, color.g, color.b};
                shapePtr->points.push_back(point);
            }
        }
    }

    transformPointCloudToCenter();
    transformPointCloudBackToOriginal();
    calculateNormals();
    calculateBoundingBoxData();
}

void CirclePointCloudShape::scale(float radius, float y, float z) {
    this->radius = radius;
    this->isBoundingBoxDataCalculated = false;
    this->generateShape();
    this->dimensions = {this->radius};
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

void SpherePointCloudShape::generateShape()
{
    shapePtr->points.clear();
    float degreeIntensity = 360*density / 20;
    for (float i = 0; i <= 180.0f; i += degreeIntensity) {
        for (float j = 0; j < 360.0f; j += degreeIntensity) {
            float radI = i * (M_PI / 180.0f), radJ = j * (M_PI / 180.0f);
            PointType point {radius*sin(radI)*cos(radJ), radius*sin(radI)*sin(radJ), radius*cos(radI), color.r, color.g, color.b};
            shapePtr->points.push_back(point);
            if (isFilled) {
                for (float k = 0; k < radius; k += density) {
                    PointType point {k*sin(radI)*cos(radJ), k*sin(radI)*sin(radJ), k*cos(radI), color.r, color.g, color.b};
                    shapePtr->points.push_back(point);
                }
            }
        }
    }

    transformPointCloudToCenter();
    transformPointCloudBackToOriginal();
    calculateNormals();
    calculateBoundingBoxData();
}

void SpherePointCloudShape::scale(float radius, float y, float z) {
    this->radius = radius;
    this->isBoundingBoxDataCalculated = false;
    this->generateShape();
    this->dimensions = {this->radius};
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
            PointType point = {j*cos(rad), j*sin(rad), 0, color.r, color.g, color.b};
            shapePtr->points.push_back(point);
            point.z = height;
            shapePtr->points.push_back(point);
        }
    }

    for (float i = 0; i <= 360.0f; i += degreeIntensity) {
        for (float j = density; j < height; j += density) {
            float rad = i * (M_PI / 180.0f);
            PointType point {radius*cos(rad), radius*sin(rad), j, color.r, color.g, color.b};
            shapePtr->points.push_back(point);
            if (isFilled) {
                for (float k = 0; k < radius; k += density) {
                    PointType point {k*cos(rad), k*sin(rad), j, color.r, color.g, color.b};
                    shapePtr->points.push_back(point);
                }
            }
        }
    }

    transformPointCloudToCenter();
    transformPointCloudBackToOriginal();
    calculateNormals();
    calculateBoundingBoxData();
}

void CylinderPointCloudShape::scale(float radius, float height, float z) {
    this->radius = radius;
    this->height = height;
    this->isBoundingBoxDataCalculated = false;
    this->generateShape();
    this->dimensions = {this->radius, this->height};
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
            PointType point = {j*cos(rad), j*sin(rad), 0, color.r, color.g, color.b};
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
            PointType point {x, y, u, color.r, color.g, color.b};
            shapePtr->points.push_back(point);
            if (isFilled) {
                for (float k = 0; k < radius; k += density) {
                    x = (height - u) / height * k * cos(radI);
                    y = (height - u) / height * k * sin(radI);
                    PointType point {x, y, u, color.r, color.g, color.b};
                    shapePtr->points.push_back(point);
                }
            }
        }
    }

    transformPointCloudToCenter();
    transformPointCloudBackToOriginal();
    calculateNormals();
    calculateBoundingBoxData();
}

void ConePointCloudShape::scale(float radius, float height, float z) {
    this->radius = radius;
    this->height = height;
    this->isBoundingBoxDataCalculated = false;
    this->generateShape();
    this->dimensions = {this->radius, this->height};
}