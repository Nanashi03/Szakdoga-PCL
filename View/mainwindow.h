#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QFileDialog>
#include <QMessageBox>
#include <QInputDialog>
#include <../Forms/InputDialogBox.h>
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

    MainWindow(QWidget *parent = nullptr);
    void refreshView();
    void showErrorMessageBox(const std::string&);
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
};
#endif // MAINWINDOW_H
