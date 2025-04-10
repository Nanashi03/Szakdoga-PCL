#include "mainwindow.h"
#include "ui_mainwindow.h"

EventImportListener MainWindow::eventImportListener = nullptr;

MainWindow::MainWindow(QWidget *parent) : QMainWindow(parent),
    ui { new Ui::MainWindow }
{
    ui->setupUi(this);

    auto renderer = vtkSmartPointer<vtkRenderer>::New();
    auto renderWindow = vtkSmartPointer<vtkGenericOpenGLRenderWindow>::New();
    renderWindow->AddRenderer(renderer);
    pclEditorView.init(renderer, renderWindow);
    ui->PCLEditorWidget->setRenderWindow(pclEditorView.getRender());
    pclEditorView.setupInteractor(ui->PCLEditorWidget->interactor(), ui->PCLEditorWidget->renderWindow());

    connect(ui->ImportCloudButton, &QPushButton::clicked, this, &MainWindow::onImportCloudButtonClicked);


    refreshView();
}

void MainWindow::refreshView()
{
    ui->PCLEditorWidget->renderWindow()->Render();
}

void MainWindow::showErrorMessageBox(const std::string& message)
{
    QMessageBox msgBox;
    msgBox.critical(0,"Error",message.data());
    msgBox.setFixedSize(500,200);
}

void MainWindow::onImportCloudButtonClicked()
{
    QString fileName = QFileDialog::getOpenFileName(this, "Open a poinc cloud", QDir::homePath(), "Point CLoud (*.pcd)");
    if (fileName.isNull()) return;

    QString id = QInputDialog::getText(this,"Cloud ID","Imported point cloud ID:");
    if (id.isNull()) return;

    eventImportListener(id.toStdString(), fileName.toStdString());
}

MainWindow::~MainWindow()
{
    delete ui;
}
