//asdasd
// Created by kristof on 2025.03.16..
//

#include "Controller.h"

Controller::Controller() {
    Viewer::cloudSelectedEventListener = [this](const string& name) { this->selectCloud(name); };
    Viewer::selectedCloudTranslateLeftEventListener = [this](float x, float y, float z) { this->translate(x,y,z); };
    MainWindow::eventImportListener = [this](const string& id, const string& fileName) { this->importCloud(id, fileName); };
}

void Controller::start() {
    mainWindow.show();
}

void Controller::tmp() {
    CylinderPointCloudShape cloud1 {"id", false, 5, 10, 1};
    CylinderPointCloudShape cloud2 {"idd", false, 5, 10, 1};
    cloud1.generateShape();
    //cloud1.calculateNormals();

    cloud2.generateShape();
    //cloud2.calculateNormals();

    model.addCloud(cloud1);
    mainWindow.pclEditorView.addCloud(cloud1.getId(), cloud1.getShape());

    //model.addCloud(cloud2);
    //viewer.addCloud(cloud2.getId(), cloud2.getShape());

    //mainWindow.pclEditorView.addNormals(cloud1.getNormalId(), cloud1.getShape());
}

void Controller::selectCloud(const string& cloudName) {
    cout << "I SELECTED A CLOUD: " << cloudName << endl;

    if (model.isCloudSelected() && cloudName == model.getSelectedCloudName())
    {
        model.deSelectCloud();
        mainWindow.pclEditorView.removeBoundingBoxCube();
    } else if (model.isCloudSelected())
    {
        model.deSelectCloud();
        mainWindow.pclEditorView.removeBoundingBoxCube();

        BoundingBoxData bboxData;
        model.selectCloud(cloudName, bboxData);
        mainWindow.pclEditorView.addBoundingBoxCube(bboxData.bboxTransform, bboxData.bboxQuaternion, bboxData.width, bboxData.height, bboxData.depth);
    } else
    {
        BoundingBoxData bboxData;
        model.selectCloud(cloudName, bboxData);
        mainWindow.pclEditorView.addBoundingBoxCube(bboxData.bboxTransform, bboxData.bboxQuaternion, bboxData.width, bboxData.height, bboxData.depth);
    }
}

void Controller::importCloud(const string& id, const string& filePath) {
    try {
        ImportedPointCloudShape cloud{id, filePath};
        cloud.generateShape();

        model.addCloud(cloud);
        mainWindow.pclEditorView.addCloud(cloud.getId(), cloud.getShape());
    } catch (const std::exception& e) {
        mainWindow.showErrorMessageBox(e.what());
    }
}

void Controller::generateRectangle(const string& id, bool isFilled, float width, float height, float density) {
    RectanglePointCloudShape cloud {id, isFilled, width, height, density};
    cloud.generateShape();

    model.addCloud(cloud);
    mainWindow.pclEditorView.addCloud(cloud.getId(), cloud.getShape());
}

void Controller::generateCircle(const string& id, bool isFilled, float radius, float density) {
    CirclePointCloudShape cloud {id, isFilled, radius, density};
    cloud.generateShape();

    model.addCloud(cloud);
    mainWindow.pclEditorView.addCloud(cloud.getId(), cloud.getShape());
}

void Controller::generateCube(const string & id, bool isFilled, float width, float height, float length, float density) {
    CuboidPointCloudShape cloud {id, isFilled, width, height, length, density};
    cloud.generateShape();

    model.addCloud(cloud);
    mainWindow.pclEditorView.addCloud(cloud.getId(), cloud.getShape());
}

void Controller::generateSphere(const string& id, bool isFilled, float radius, float density) {
    SpherePointCloudShape cloud {id, isFilled, radius, density};
    cloud.generateShape();

    model.addCloud(cloud);
    mainWindow.pclEditorView.addCloud(cloud.getId(), cloud.getShape());
}

void Controller::generateCylinder(const string& id, bool isFilled, float radius, float height, float density) {
    CylinderPointCloudShape cloud {id, isFilled, radius, height, density};
    cloud.generateShape();

    model.addCloud(cloud);
    mainWindow.pclEditorView.addCloud(cloud.getId(), cloud.getShape());
}

void Controller::generateCone(const string& id, bool isFilled, float radius, float height, float density) {
    ConePointCloudShape cloud {id, isFilled, radius, height, density};
    cloud.generateShape();

    model.addCloud(cloud);
    mainWindow.pclEditorView.addCloud(cloud.getId(), cloud.getShape());
}

void Controller::translate(float x, float y, float z) {
    if (model.isCloudSelected()) {
        mainWindow.pclEditorView.updateCloud(model.getSelectedCloudName(), model.getSelectedCloudAreNormalsPresent(), model.translateSelectedCloud(x,y,z));
        mainWindow.pclEditorView.translateBoundingBoxCube(x,y,z);
    }
}

void Controller::rotate(float angle, char axis) {
    if (model.isCloudSelected())
        mainWindow.pclEditorView.updateCloud(model.getSelectedCloudName(), model.getSelectedCloudAreNormalsPresent(), model.rotateSelectedCloud(angle, axis));
}



