//asdasd
// Created by kristof on 2025.03.16..
//

#include "Controller.h"

Controller::Controller() {
    Viewer::cloudSelectedEventListener = [this](const string& name) { this->selectCloud(name); };
    Viewer::selectedCloudTranslateLeftEventListener = [this](float x, float y, float z) { this->translate(x,y,z); };
    MainWindow::importEventListener = [this](const string& id, const string& fileName) { this->importCloud(id, fileName); };
    MainWindow::addSquareEventListener = [this](const string& id, bool isFilled, float side) { this->generateRectangle(id, isFilled, side, side, 1.0f); };
    MainWindow::addCubeEventListener = [this](const string& id, bool isFilled, float side) { this->generateCube(id, isFilled, side, side, side, 1.0f); };
    MainWindow::addCircleEventListener = [this](const string& id, bool isFilled, float r) { this->generateCircle(id, isFilled, r, 1.0f); };
    MainWindow::addSphereEventListener = [this](const string& id, bool isFilled, float r) { this->generateSphere(id, isFilled, r, 1.0f); };
    MainWindow::addRectangleEventListener = [this](const string& id, bool isFilled, float w, float h) { this->generateRectangle(id, isFilled, w, h, 1.0f); };
    MainWindow::addCylinderEventListener = [this](const string& id, bool isFilled, float r, float h) { this->generateCylinder(id, isFilled, r, h, 1.0f); };
    MainWindow::addConeEventListener = [this](const string& id, bool isFilled, float r, float h) { this->generateCone(id, isFilled, r, h, 1.0f); };
    MainWindow::addCuboidEventListener = [this](const string& id, bool isFilled, float w, float h, float d) { this->generateCube(id, isFilled, w, h, d, 1.0f); };
}

void Controller::start() {
    mainWindow.show();
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
    try {
        RectanglePointCloudShape cloud {id, isFilled, width, height, density};
        cloud.generateShape();

        model.addCloud(cloud);
        mainWindow.pclEditorView.addCloud(cloud.getId(), cloud.getShape());
    } catch (const std::exception& e) {
        mainWindow.showErrorMessageBox(e.what());
    }
}

void Controller::generateCircle(const string& id, bool isFilled, float radius, float density) {
    try {
        CirclePointCloudShape cloud {id, isFilled, radius, density};
        cloud.generateShape();

        model.addCloud(cloud);
        mainWindow.pclEditorView.addCloud(cloud.getId(), cloud.getShape());
    } catch (const std::exception& e) {
        mainWindow.showErrorMessageBox(e.what());
    }
}

void Controller::generateCube(const string & id, bool isFilled, float width, float height, float length, float density) {
    try {
        CuboidPointCloudShape cloud {id, isFilled, width, height, length, density};
        cloud.generateShape();

        model.addCloud(cloud);
        mainWindow.pclEditorView.addCloud(cloud.getId(), cloud.getShape());
    } catch (const std::exception& e) {
        mainWindow.showErrorMessageBox(e.what());
    }
}

void Controller::generateSphere(const string& id, bool isFilled, float radius, float density) {
    try {
        SpherePointCloudShape cloud {id, isFilled, radius, density};
        cloud.generateShape();

        model.addCloud(cloud);
        mainWindow.pclEditorView.addCloud(cloud.getId(), cloud.getShape());
    } catch (const std::exception& e) {
        mainWindow.showErrorMessageBox(e.what());
    }
}

void Controller::generateCylinder(const string& id, bool isFilled, float radius, float height, float density) {
    try {
        CylinderPointCloudShape cloud {id, isFilled, radius, height, density};
        cloud.generateShape();

        model.addCloud(cloud);
        mainWindow.pclEditorView.addCloud(cloud.getId(), cloud.getShape());
    } catch (const std::exception& e) {
        mainWindow.showErrorMessageBox(e.what());
    }
}

void Controller::generateCone(const string& id, bool isFilled, float radius, float height, float density) {
    try {
        ConePointCloudShape cloud {id, isFilled, radius, height, density};
        cloud.generateShape();

        model.addCloud(cloud);
        mainWindow.pclEditorView.addCloud(cloud.getId(), cloud.getShape());
    } catch (const std::exception& e) {
        mainWindow.showErrorMessageBox(e.what());
    }
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



