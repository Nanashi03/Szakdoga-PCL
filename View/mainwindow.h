#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <vtkGenericOpenGLRenderWindow.h>
#include "Viewer.h"

QT_BEGIN_NAMESPACE
namespace Ui {
class MainWindow;
}
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

    void refreshView();
    Viewer pclEditorView;
private:
    Ui::MainWindow *ui;
    QSizePolicy pclEditorSizePolicy;

};
#endif // MAINWINDOW_H
