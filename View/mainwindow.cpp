#include "mainwindow.h"
#include "ui_mainwindow.h"

EventImportListener MainWindow::importEventListener = nullptr;
EventOneInputListener MainWindow::addSquareEventListener = nullptr;
EventOneInputListener MainWindow::addCubeEventListener = nullptr;
EventOneInputListener MainWindow::addCircleEventListener = nullptr;
EventOneInputListener MainWindow::addSphereEventListener = nullptr;
EventTwoInputListener MainWindow::addRectangleEventListener = nullptr;
EventTwoInputListener MainWindow::addCylinderEventListener = nullptr;
EventTwoInputListener MainWindow::addConeEventListener = nullptr;
EventThreeInputListener MainWindow::addCuboidEventListener = nullptr;

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
    connect(ui->AddSquareButton, &QPushButton::clicked, this, &MainWindow::onAddSquareButtonClicked);
    connect(ui->AddRectangleButton, &QPushButton::clicked, this, &MainWindow::onRectangleButtonClicked);
    connect(ui->AddCircleButton, &QPushButton::clicked, this, &MainWindow::onAddCircleButtonClicked);
    connect(ui->AddCubeButton, &QPushButton::clicked, this, &MainWindow::onAddCubeButtonClicked);
    connect(ui->AddCuboidButton, &QPushButton::clicked, this, &MainWindow::onAddCuboidButtonClicked);
    connect(ui->AddSphereButton, &QPushButton::clicked, this, &MainWindow::onAddSphereButtonClicked);
    connect(ui->AddCylinderButton, &QPushButton::clicked, this, &MainWindow::onAddCylinderButtonClicked);
    connect(ui->AddConeButton, &QPushButton::clicked, this, &MainWindow::onAddConeButtonClicked);
 
    refreshView();
}

void MainWindow::refreshView()
{
    ui->PCLEditorWidget->renderWindow()->Render();
}

void MainWindow::showErrorMessageBox(const std::string& message)
{
    QMessageBox::critical(this, "Error", message.data());
}

void MainWindow::onImportCloudButtonClicked()
{
    QString fileName = QFileDialog::getOpenFileName(this, "Open a poinc cloud", QDir::homePath(), "Point CLoud (*.pcd)");
    if (fileName.isNull()) return;

    QString id = QInputDialog::getText(this,"Cloud ID","Imported point cloud ID:");
    if (id.isNull()) return;

    importEventListener(id.toStdString(), fileName.toStdString());
}

void MainWindow::onAddSquareButtonClicked()
{
    InputFormData data;
    InputDialogBox dBox {this, {"Sides"}, {true, false, false}, data};
    dBox.setModal(true);
    if (dBox.exec() == QDialog::Accepted) {
        addSquareEventListener(data.id, data.isFilled, data.x);
    }
}

void MainWindow::onRectangleButtonClicked()
{
    InputFormData data;
    InputDialogBox dBox {this, {"Width", "Height"}, {true, true, false}, data};
    dBox.setModal(true);
    if (dBox.exec() == QDialog::Accepted) {
        addRectangleEventListener(data.id, data.isFilled, data.x, data.y);
    }
}

void MainWindow::onAddCircleButtonClicked()
{
    InputFormData data;
    InputDialogBox dBox {this, {"Radius"}, {true, false, false}, data};
    dBox.setModal(true);
    if (dBox.exec() == QDialog::Accepted) {
        addCircleEventListener(data.id, data.isFilled, data.x);
    }
}

void MainWindow::onAddCubeButtonClicked()
{
    InputFormData data;
    InputDialogBox dBox {this, {"Sides"}, {true, false, false}, data};
    dBox.setModal(true);
    if (dBox.exec() == QDialog::Accepted) {
        addCubeEventListener(data.id, data.isFilled, data.x);
    }
}

void MainWindow::onAddCuboidButtonClicked()
{
    InputFormData data;
    InputDialogBox dBox {this, {"Width", "Height", "Depth"}, {true, true, true}, data};
    dBox.setModal(true);
    if (dBox.exec() == QDialog::Accepted) {
        addCuboidEventListener(data.id, data.isFilled, data.x, data.y, data.z);
    }
}

void MainWindow::onAddSphereButtonClicked()
{
    InputFormData data;
    InputDialogBox dBox {this, {"Radius"}, {true, false, false}, data};
    dBox.setModal(true);
    if (dBox.exec() == QDialog::Accepted) {
        addSphereEventListener(data.id, data.isFilled, data.x);
    }
}

void MainWindow::onAddCylinderButtonClicked()
{
    InputFormData data;
    InputDialogBox dBox {this, {"Radius", "Height"}, {true, true, false}, data};
    dBox.setModal(true);
    if (dBox.exec() == QDialog::Accepted) {
        addCylinderEventListener(data.id, data.isFilled, data.x, data.y);
    }
}

void MainWindow::onAddConeButtonClicked()
{
    InputFormData data;
    InputDialogBox dBox {this, {"Radius", "Height"}, {true, true, false}, data};
    dBox.setModal(true);
    if (dBox.exec() == QDialog::Accepted) {
        addConeEventListener(data.id, data.isFilled, data.x, data.y);
    }
}

MainWindow::~MainWindow()
{
    delete ui;
}
