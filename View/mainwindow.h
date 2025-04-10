#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QFileDialog>
#include <QMessageBox>
#include <QInputDialog>
#include "Viewer.h"

QT_BEGIN_NAMESPACE
namespace Ui {
class MainWindow;
}
QT_END_NAMESPACE

using EventImportListener = std::function<void(const std::string&,const std::string&)>;

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

    static EventImportListener eventImportListener;

    void refreshView();
    Viewer pclEditorView;
    void showErrorMessageBox(const std::string&);
private:
    Ui::MainWindow *ui;
private slots:
    void onImportCloudButtonClicked();
};
#endif // MAINWINDOW_H
