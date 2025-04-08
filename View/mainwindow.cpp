#include "mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent) : QMainWindow(parent),
    ui { new Ui::MainWindow },
    pclEditorSizePolicy { QSizePolicy::Preferred, QSizePolicy::Preferred }
{
    ui->setupUi(this);

    auto renderer = vtkSmartPointer<vtkRenderer>::New();
    auto renderWindow = vtkSmartPointer<vtkGenericOpenGLRenderWindow>::New();
    renderWindow->AddRenderer(renderer);
    pclEditorView.init(renderer, renderWindow);
    ui->PCLEditorWidget->setRenderWindow(pclEditorView.getRender());
    pclEditorView.setupInteractor(ui->PCLEditorWidget->interactor(), ui->PCLEditorWidget->renderWindow());

    pclEditorSizePolicy.setHorizontalStretch(2);
    ui->PCLEditorWidget->setSizePolicy(pclEditorSizePolicy);

    refreshView();
}

void MainWindow::refreshView()
{
    ui->PCLEditorWidget->renderWindow()->Render();
}

MainWindow::~MainWindow()
{
    delete ui;
}
