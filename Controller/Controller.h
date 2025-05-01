#ifndef CONTROLLER_H
#define CONTROLLER_H

#include "Model.h"
#include "mainwindow.h"

class Controller {
    Model model;
    MainWindow mainWindow;
public:
    Controller();
    void start();
    void selectCloud(const string&);

    void importProject(const string&);
    void importCloud(const string&, const string&);
    void exportProject(const string&);
    void exportCloud(const string&);
    void generateRectangle(const string&, bool, float, float, float);
    void generateCircle(const string&, bool, float, float);
    void generateCube(const string&, bool, float, float, float, float);
    void generateSphere(const string&, bool, float, float);
    void generateCylinder(const string&, bool, float, float, float);
    void generateCone(const string&, bool, float, float, float);

    void changeSelectedCloudColor(uint8_t, uint8_t, uint8_t);
    void updateSelectedCloudDimensions(float,float,float);
    void updateSelectedCloudDensity(int);
    void updateSelectedCloudIsFilled(bool);
    void updateSelectedCloudNormals(bool);
    void removeSelectedCloud();

    void translate(float,float,float);
    void rotate(int,char);
};

#endif //CONTROLLER_H
