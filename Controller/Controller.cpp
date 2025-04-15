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
    MainWindow::densityChangedEventListener = [this](int d) { this->updateSelectedCloudDensity(d); };
    MainWindow::colorChangedEventListener = [this](int r, int g, int b) { this->changeSelectedCloudColor(r,g,b); };
    MainWindow::shapeChangedEventListener = [this](float x, float y, float z) { this->updateSelectedCloudDimensions(x,y,z); };
    MainWindow::isFilledChangedEventListener = [this](bool isFilled) { this->updateSelectedCloudIsFilled(isFilled); };
    MainWindow::areNormalsPresentChangedEventListener = [this](bool areNormalsPresent) { this->updateSelectedCloudNormals(areNormalsPresent); };
    MainWindow::removeCloudEventListener = [this]() { this->removeSelectedCloud(); };
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
        mainWindow.changeToAddShapeWidget();
    } else if (model.isCloudSelected())
    {
        model.deSelectCloud();
        mainWindow.pclEditorView.removeBoundingBoxCube();

        model.selectCloud(cloudName);
        mainWindow.pclEditorView.addBoundingBoxCube(model.getBoundingBoxDataAroundSelectedCloud());
        mainWindow.changeToEditShapeWidget(model.getEditCloudData());
    } else
    {
        model.selectCloud(cloudName);
        mainWindow.pclEditorView.addBoundingBoxCube(model.getBoundingBoxDataAroundSelectedCloud());
        mainWindow.changeToEditShapeWidget(model.getEditCloudData());
    }
}
/**********************************************GENERATION**************************************************************/
void Controller::importCloud(const string& id, const string& filePath) {
    try {
        shared_ptr<ImportedPointCloudShape> cloud {make_shared<ImportedPointCloudShape>(id, filePath)};
        cloud->generateShape();

        model.addCloud(cloud);
        mainWindow.pclEditorView.addCloud(cloud->getId(), cloud->getShape());
    } catch (const std::exception& e) {
        mainWindow.showErrorMessageBox(e.what());
    }
}

void Controller::generateRectangle(const string& id, bool isFilled, float width, float height, float density) {
    try {
        shared_ptr<RectanglePointCloudShape> cloud { make_shared<RectanglePointCloudShape>(id, isFilled, width, height, density) };
        cloud->generateShape();

        model.addCloud(cloud);
        mainWindow.pclEditorView.addCloud(cloud->getId(), cloud->getShape());
    } catch (const std::exception& e) {
        mainWindow.showErrorMessageBox(e.what());
    }
}

void Controller::generateCircle(const string& id, bool isFilled, float radius, float density) {
    try {
        shared_ptr<CirclePointCloudShape> cloud { make_shared<CirclePointCloudShape>(id, isFilled, radius, density) };
        cloud->generateShape();

        model.addCloud(cloud);
        mainWindow.pclEditorView.addCloud(cloud->getId(), cloud->getShape());
    } catch (const std::exception& e) {
        mainWindow.showErrorMessageBox(e.what());
    }
}

void Controller::generateCube(const string & id, bool isFilled, float width, float height, float length, float density) {
    try {
        shared_ptr<CuboidPointCloudShape> cloud { make_shared<CuboidPointCloudShape>(id, isFilled, width, height, length, density) };
        cloud->generateShape();

        model.addCloud(cloud);
        mainWindow.pclEditorView.addCloud(cloud->getId(), cloud->getShape());
    } catch (const std::exception& e) {
        mainWindow.showErrorMessageBox(e.what());
    }
}

void Controller::generateSphere(const string& id, bool isFilled, float radius, float density) {
    try {
        shared_ptr<SpherePointCloudShape> cloud  { make_shared<SpherePointCloudShape>(id, isFilled, radius, density) };
        cloud->generateShape();

        model.addCloud(cloud);
        mainWindow.pclEditorView.addCloud(cloud->getId(), cloud->getShape());
    } catch (const std::exception& e) {
        mainWindow.showErrorMessageBox(e.what());
    }
}

void Controller::generateCylinder(const string& id, bool isFilled, float radius, float height, float density) {
    try {
        shared_ptr<CylinderPointCloudShape> cloud { make_shared<CylinderPointCloudShape>(id, isFilled, radius, height, density) };
        cloud->generateShape();

        model.addCloud(cloud);
        mainWindow.pclEditorView.addCloud(cloud->getId(), cloud->getShape());
    } catch (const std::exception& e) {
        mainWindow.showErrorMessageBox(e.what());
    }
}

void Controller::generateCone(const string& id, bool isFilled, float radius, float height, float density) {
    try {
        shared_ptr<ConePointCloudShape> cloud { make_shared<ConePointCloudShape>(id, isFilled, radius, height, density) };
        cloud->generateShape();

        model.addCloud(cloud);
        mainWindow.pclEditorView.addCloud(cloud->getId(), cloud->getShape());
    } catch (const std::exception& e) {
        mainWindow.showErrorMessageBox(e.what());
    }
}

/*************************************************EDITING**************************************************************/
void Controller::changeSelectedCloudColor(uint8_t r, uint8_t g, uint8_t b) {
    if (!model.isCloudSelected()) return;

    model.colorSelectedCloud({r,g,b});
    mainWindow.pclEditorView.updateCloud(model.getSelectedCloudName(), model.getSelectedCloudAreNormalsPresent(), model.getSelectedCloudShape());
    mainWindow.refreshView();
}

void Controller::updateSelectedCloudDimensions(float x, float y, float z) {
    if (!model.isCloudSelected()) return;

    model.updateSelectedCloudDimensions(x, y, z);
    mainWindow.pclEditorView.updateCloud(model.getSelectedCloudName(), model.getSelectedCloudAreNormalsPresent(), model.getSelectedCloudShape());

    mainWindow.pclEditorView.removeBoundingBoxCube();
    mainWindow.pclEditorView.addBoundingBoxCube(model.getBoundingBoxDataAroundSelectedCloud());

    mainWindow.refreshView();
}

void Controller::updateSelectedCloudDensity(int density) {
    if (!model.isCloudSelected()) return;

    model.updateSelectedCloudDensity(density);
    mainWindow.pclEditorView.updateCloud(model.getSelectedCloudName(), model.getSelectedCloudAreNormalsPresent(), model.getSelectedCloudShape());
    mainWindow.refreshView();
}

void Controller::updateSelectedCloudIsFilled(bool isFilled)
{
    if (!model.isCloudSelected()) return;

    model.updateSelectedCloudIsFilled(isFilled);
    mainWindow.pclEditorView.updateCloud(model.getSelectedCloudName(), model.getSelectedCloudAreNormalsPresent(), model.getSelectedCloudShape());
    mainWindow.refreshView();
}

void Controller::updateSelectedCloudNormals(bool areNormalsPresent)
{
    if (!model.isCloudSelected()) return;

    if (areNormalsPresent) {
        model.generateNormalsForSelectedCloud();
        mainWindow.pclEditorView.addNormals(model.getSelectedCloudNormalsName(), model.getSelectedCloudShape());
    } else {
        mainWindow.pclEditorView.removeNormals(model.getSelectedCloudNormalsName());
    }
}

void Controller::removeSelectedCloud() {
    if (!model.isCloudSelected()) return;
    mainWindow.pclEditorView.removeCloud(model.getSelectedCloudName(), model.getSelectedCloudAreNormalsPresent());
    mainWindow.pclEditorView.removeBoundingBoxCube();
    mainWindow.changeToAddShapeWidget();
    model.removeSelectedCloud();

    mainWindow.refreshView();
}
/*********************************************TRANSFORMATION***********************************************************/
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



