#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QFileDialog>
#include <QMessageBox>
#include <QInputDialog>
#include <../Forms/InputDialogBox.h>
#include "../DataStructures/EditCloudData.h"
#include "Viewer.h"

QT_BEGIN_NAMESPACE
namespace Ui {
class MainWindow;
}
QT_END_NAMESPACE

using EventImportListener = std::function<void(const std::string&,const std::string&)>;
using EventOneInputListener = std::function<void(const std::string&,bool,float)>;
using EventTwoInputListener = std::function<void(const std::string&,bool,float,float)>;
using EventThreeInputListener = std::function<void(const std::string&,bool,float,float,float)>;
using EventDensitySliderListener = std::function<void(int)>;
using EventColorSliderChangedListener = std::function<void(int,int,int)>;
using EventRotationSliderChangedListener = std::function<void(int,char)>;
using EventChangeShapeDimensions = std::function<void(float,float,float)>;
using EventTickBoxChanged = std::function<void(bool)>;
using EventRemoveCloud = std::function<void()>;

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    Viewer pclEditorView;

    static EventImportListener importEventListener;
    static EventOneInputListener addSquareEventListener;
    static EventOneInputListener addCubeEventListener;
    static EventOneInputListener addCircleEventListener;
    static EventOneInputListener addSphereEventListener;
    static EventTwoInputListener addRectangleEventListener;
    static EventTwoInputListener addCylinderEventListener;
    static EventTwoInputListener addConeEventListener;
    static EventThreeInputListener addCuboidEventListener;
    static EventDensitySliderListener densityChangedEventListener;
    static EventColorSliderChangedListener colorChangedEventListener;
    static EventRotationSliderChangedListener rotationChangedEventListener;
    static EventChangeShapeDimensions shapeChangedEventListener;
    static EventTickBoxChanged isFilledChangedEventListener;
    static EventTickBoxChanged areNormalsPresentChangedEventListener;
    static EventRemoveCloud removeCloudEventListener;

    MainWindow(QWidget *parent = nullptr);
    void refreshView();
    void showErrorMessageBox(const std::string&);
    void changeToEditShapeWidget(EditCloudData);
    void changeToAddShapeWidget();
    ~MainWindow();
private:
    Ui::MainWindow *ui;
private slots:
    void onImportCloudButtonClicked();
    void onAddSquareButtonClicked();
    void onRectangleButtonClicked();
    void onAddCircleButtonClicked();
    void onAddCubeButtonClicked();
    void onAddCuboidButtonClicked();
    void onAddSphereButtonClicked();
    void onAddCylinderButtonClicked();
    void onAddConeButtonClicked();
    void onDensitySliderChanged();
    void onColorSliderChanged();
    void onRotationSliderChanged();
    void onDimensionChanged();

};
#endif // MAINWINDOW_H
