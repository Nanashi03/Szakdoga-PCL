#include "Model.h"

Model::Model() :
    selectedCloud {-1}
{}

shared_ptr<IPointCloudShape> Model::createPointCloudShape(const string& type, const string& name, bool isFilled, float x, float y, float z, float d) {
    if (type == "RectanglePointCloudShape")
        return make_shared<RectanglePointCloudShape>(name, isFilled, x, y, d);
    else if (type == "CuboidPointCloudShape")
        return make_shared<CuboidPointCloudShape>(name, isFilled, x, y, z, d);
    else if (type == "CirclePointCloudShape")
        return make_shared<CirclePointCloudShape>(name, isFilled, x, d);
    else if (type == "SpherePointCloudShape")
        return make_shared<SpherePointCloudShape>(name, isFilled, x, d);
    else if (type == "CylinderPointCloudShape")
        return make_shared<CylinderPointCloudShape>(name, isFilled, x, y, d);
    else if (type == "ConePointCloudShape")
        return make_shared<ConePointCloudShape>(name, isFilled, x, y, d);
    else if (type == "ImportedPointCloudShape")
        return make_shared<ImportedPointCloudShape>(name, "");
    else
        throw std::runtime_error("Unknown point cloud shape type: " + type);
}


void Model::addCloud(const shared_ptr<IPointCloudShape>& cloud_shape) {
    if (cloud_shape->getId() == "BBOX") throw runtime_error("ID already exists or conflicts with to be generated ones!");
    for (int i = 0; i < clouds.size(); i++) {
        if (clouds[i]->getId() == cloud_shape->getId() || clouds[i]->getNormalId() == cloud_shape->getId())
            throw runtime_error("ID already exists or conflicts with to be generated ones!");
    }
    clouds.push_back(cloud_shape);
}

void Model::importProject(const string& filePath) {
    Database database { filePath, "import" };
    for (const string& cloudName : database.getPointCloudNamesFromDatabase()) {
        EditCloudData data { database.getPointCloudPropertiesFromDatabase(cloudName) };
        shared_ptr<IPointCloudShape> shape {
            createPointCloudShape(database.getPointCloudTypeByName(cloudName), cloudName, data.isFilled, data.dim[0], data.dim[1], data.dim[2], data.density)
        };
        shape->setRotationAt(0, data.rotation[0]);
        shape->setRotationAt(1, data.rotation[1]);
        shape->setRotationAt(2, data.rotation[2]);
        shape->setAreNormalsShown(data.areNormalsShown);
        if (shape->getIsColorable() && data.rgb.size() == 3)
            shape->setColor({static_cast<std::uint8_t>(data.rgb[0]), static_cast<std::uint8_t>(data.rgb[1]), static_cast<std::uint8_t>(data.rgb[2])});

        tuple<Eigen::Vector3f, Eigen::Affine3f> transFormation = database.getPointCloudTransformationFromDatabase(cloudName);
        shape->addToTranslationValues(get<0>(transFormation));
        shape->addToRotationMatrix(get<1>(transFormation));
        shape->setShape(database.getPointCloudFromDatabase(cloudName));

        clouds.push_back(shape);
    }
}

void Model::exportClouds(const string& newFilePath) {
    PointCloudT::Ptr mergedCloud  { make_shared<PointCloudT>() };
    for (int i = 0; i < clouds.size(); i++) {
        *mergedCloud += *clouds[i]->getShape();
    }
    mergedCloud->width = mergedCloud->points.size();
    mergedCloud->height = 1;
    mergedCloud->is_dense = true;

    if (pcl::io::savePCDFileASCII(newFilePath, *mergedCloud) == -1)
        throw runtime_error("Failed to save file :" + newFilePath);
}

void Model::exportProject(const string& newFilePath) {
    Database database { newFilePath, "export" };
    for (shared_ptr cloud : clouds) {
        string typeName = typeid(*cloud).name();
        database.addPointCloudNameToDatabase(cloud->getId(), typeName.substr(2));
        database.addPointCloudToDatabase(cloud->getId(), cloud->getShape());
        database.addPointCloudPropertiesToDatabase(cloud->getId(), getEditCloudData(cloud->getId()));
        database.addPointCloudTransformationToDatabase(cloud->getId(), cloud->getTranslationValues(), cloud->getCurrentRotation());
    }
}

void Model::updateSelectedCloudDimensions(float x, float y, float z) {
    if (selectedCloud == -1) return;
    clouds[selectedCloud]->scale(x,y,z);
}

void Model::updateSelectedCloudDensity(int density) {
    if (selectedCloud == -1 || !clouds[selectedCloud]->getIsDensitable()) return;
    clouds[selectedCloud]->setDensity(density);
    clouds[selectedCloud]->generateShape();
}

void Model::updateSelectedCloudIsFilled(bool isFilled) {
    if (selectedCloud == -1 || !clouds[selectedCloud]->getIsFillable()) return;
    clouds[selectedCloud]->setIsFilled(isFilled);
    clouds[selectedCloud]->generateShape();
}

void Model::updateSelectedCloudAreNormalsShown(bool areNormalsShown) {
    if (selectedCloud == -1) return;
    clouds[selectedCloud]->setAreNormalsShown(areNormalsShown);
}

void Model::removeSelectedCloud() {
    if (selectedCloud == -1) return;
    clouds.erase(clouds.begin() + selectedCloud);
    deSelectCloud();
}

void Model::colorSelectedCloud(pcl::RGB color) {
    if (selectedCloud == -1 || !clouds[selectedCloud]->getIsColorable()) return;
    clouds[selectedCloud]->setColor(color);
}

void Model::deSelectCloud() {
    selectedCloud = -1;
}

void Model::selectCloud(const string& name) {
    for (int i = 0; i < clouds.size(); i++) {
        if (clouds[i]->getId() == name || clouds[i]->getNormalId() == name) {
            selectedCloud = i;
            break;
        }
    }
}

BoundingBoxData Model::getBoundingBoxDataAroundSelectedCloud() const {
    if (selectedCloud == -1) return BoundingBoxData();
    return clouds[selectedCloud]->getBoundingBoxData();
}

void Model::translateSelectedCloud(float x, float y, float z, Eigen::Affine3f& transform) {
    if (selectedCloud == -1) return;

    transform = Eigen::Affine3f::Identity();
    transform.translation() << x, y, z;
    pcl::transformPointCloud(*clouds[selectedCloud]->getShape(), *clouds[selectedCloud]->getShape(), transform);

    clouds[selectedCloud]->addToTranslationValues({x, y, z});
}

void Model::rotateSelectedCloud(int angle, char axis, Eigen::Affine3f& fullTransform) {
    if (selectedCloud == -1) return;

    Eigen::Affine3f rotation { Eigen::Affine3f::Identity() };
    float offsetRad = 0.0f;
    if (axis == 'x') {
        offsetRad = (float)(angle - clouds[selectedCloud]->getRotationAt(0)) * M_PI / 180.0f;
        rotation.rotate(Eigen::AngleAxisf(offsetRad, Eigen::Vector3f::UnitX()));
        clouds[selectedCloud]->setRotationAt(0, angle);
    } else if (axis == 'y') {
        offsetRad = (float)(angle - clouds[selectedCloud]->getRotationAt(1)) * M_PI / 180.0f;
        rotation.rotate(Eigen::AngleAxisf(offsetRad, Eigen::Vector3f::UnitY()));
        clouds[selectedCloud]->setRotationAt(1, angle);
    } else if (axis == 'z') {
        offsetRad = (float)(angle - clouds[selectedCloud]->getRotationAt(2)) * M_PI / 180.0f;
        rotation.rotate(Eigen::AngleAxisf(offsetRad, Eigen::Vector3f::UnitZ()));
        clouds[selectedCloud]->setRotationAt(2, angle);
    }
    clouds[selectedCloud]->addToRotationMatrix(rotation);

    Eigen::Affine3f toOrigin = Eigen::Affine3f::Identity();
    toOrigin.translate(-clouds[selectedCloud]->getTranslationValues());
    Eigen::Affine3f backToCentroid = Eigen::Affine3f::Identity();
    backToCentroid.translate(clouds[selectedCloud]->getTranslationValues());

    fullTransform = backToCentroid * rotation * toOrigin;

    pcl::transformPointCloud(*clouds[selectedCloud]->getShape(), *clouds[selectedCloud]->getShape(), fullTransform);
}

EditCloudData Model::getEditCloudData(const string& name) {
    int i = 0;
    for (i = 0; i < clouds.size(); i++)
        if (clouds[i]->getId() == name) break;
    if (i == clouds.size()) return EditCloudData();

    EditCloudData data;
    data.name = clouds[i]->getId();
    data.rgb = { clouds[i]->getColor().r, clouds[i]->getColor().g, clouds[i]->getColor().b };
    data.rotation = { clouds[i]->getRotationAt(0), clouds[i]->getRotationAt(1), clouds[i]->getRotationAt(2) };
    data.isFilled = clouds[i]->getIsFilled();
    data.areNormalsShown = clouds[i]->getAreNormalsShown();
    data.dim = clouds[i]->getDimensions();
    data.density = clouds[i]->getDensity();

    data.labels = clouds[i]->getLabels();
    data.showFilledEdit = clouds[i]->getIsFillable();
    data.showColorEdit = clouds[i]->getIsColorable();
    data.showDensityEdit = clouds[i]->getIsDensitable();
    data.showLabels = clouds[i]->getShowLabels();
    return data;
}

vector<string> Model::getCloudNames() const {
    vector<string> names;
    for (int i = 0; i < clouds.size(); i++)
        names.emplace_back(clouds[i]->getId());
    return names;
}

vector<shared_ptr<IPointCloudShape>> Model::getClouds() { return clouds; }

PointCloudT::ConstPtr Model::getSelectedCloudShape() const {
    if (selectedCloud == -1) return nullptr;
    return clouds[selectedCloud]->getShape();
}

string Model::getSelectedCloudName() const {
    if (selectedCloud == -1) return "";
    return clouds[selectedCloud]->getId();
}

string Model::getSelectedCloudNormalsName() const {
    if (selectedCloud == -1) return "";
    return clouds[selectedCloud]->getNormalId();
}

bool Model::isCloudSelected() const {
    return selectedCloud != -1;
}

bool Model::getSelectedCloudAreNormalsShown() const {
    if (selectedCloud == -1) return false;
    return clouds[selectedCloud]->getAreNormalsShown();
}