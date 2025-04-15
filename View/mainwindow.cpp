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
EventDensitySliderListener MainWindow::densityChangedEventListener = nullptr;
EventSelectedCloudSliderListener MainWindow::colorChangedEventListener = nullptr;
EventChangeShapeDimensions MainWindow::shapeChangedEventListener = nullptr;
EventTickBoxChanged MainWindow::isFilledChangedEventListener = nullptr;
EventTickBoxChanged MainWindow::areNormalsPresentChangedEventListener = nullptr;
EventRemoveCloud MainWindow::removeCloudEventListener = nullptr;

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

    ui->EditCloudsWidget->hide();

    connect(ui->ImportCloudButton, &QPushButton::clicked, this, &MainWindow::onImportCloudButtonClicked);
    connect(ui->AddSquareButton, &QPushButton::clicked, this, &MainWindow::onAddSquareButtonClicked);
    connect(ui->AddRectangleButton, &QPushButton::clicked, this, &MainWindow::onRectangleButtonClicked);
    connect(ui->AddCircleButton, &QPushButton::clicked, this, &MainWindow::onAddCircleButtonClicked);
    connect(ui->AddCubeButton, &QPushButton::clicked, this, &MainWindow::onAddCubeButtonClicked);
    connect(ui->AddCuboidButton, &QPushButton::clicked, this, &MainWindow::onAddCuboidButtonClicked);
    connect(ui->AddSphereButton, &QPushButton::clicked, this, &MainWindow::onAddSphereButtonClicked);
    connect(ui->AddCylinderButton, &QPushButton::clicked, this, &MainWindow::onAddCylinderButtonClicked);
    connect(ui->AddConeButton, &QPushButton::clicked, this, &MainWindow::onAddConeButtonClicked);

    connect(ui->DensitySlider, &QSlider::valueChanged, this, &MainWindow::onDensitySliderChanged);
    connect(ui->RedSlider, &QSlider::valueChanged, this, &MainWindow::onColorSliderChanged);
    connect(ui->GreenSlider, &QSlider::valueChanged, this, &MainWindow::onColorSliderChanged);
    connect(ui->BlueSlider, &QSlider::valueChanged, this, &MainWindow::onColorSliderChanged);
    connect(ui->xInput, &QLineEdit::editingFinished, this, &MainWindow::onDimensionChanged);
    connect(ui->yInput, &QLineEdit::editingFinished, this, &MainWindow::onDimensionChanged);
    connect(ui->zInput, &QLineEdit::editingFinished, this, &MainWindow::onDimensionChanged);
    connect(ui->IsFilledCheckBox, &QCheckBox::toggled, this, [](bool v) { isFilledChangedEventListener(v); });
    connect(ui->ShowNormals, &QCheckBox::toggled, this, [](bool v) { areNormalsPresentChangedEventListener(v); });
    connect(ui->RemoveCloud, &QPushButton::clicked, this, []() { removeCloudEventListener(); });

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
        cout << data.x << endl;
        cout << data.y << endl;
        cout << data.z << endl;
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

void MainWindow::onDensitySliderChanged()
{
    ui->DensityNumber->display(ui->DensitySlider->value());
    densityChangedEventListener(ui->DensitySlider->value());
}

void MainWindow::onColorSliderChanged()
{
    std::string sliderName = sender()->objectName().toStdString();
    if (sliderName == "RedSlider") ui->RedNumber->display(ui->RedSlider->value());
    else if (sliderName == "GreenSlider") ui->GreenNumber->display(ui->GreenSlider->value());
    else if (sliderName == "BlueSlider") ui->BlueNumber->display(ui->BlueSlider->value());

    colorChangedEventListener(ui->RedSlider->value(), ui->GreenSlider->value(), ui->BlueSlider->value());
}

void MainWindow::onDimensionChanged()
{
    if (!ui->xInput->hasAcceptableInput() || !ui->yInput->hasAcceptableInput() || !ui->zInput->hasAcceptableInput())
    {
        showErrorMessageBox(R"(For correct decimal input use "." instead of ","!)");
        return;
    }

    shapeChangedEventListener(ui->xInput->text().toFloat(), ui->yInput->text().toFloat(), ui->zInput->text().toFloat());
}

void MainWindow::changeToEditShapeWidget(EditCloudData editData)
{
    ui->AddCloudsWidget->hide();

    ui->xWidget->hide();
    ui->yWidget->hide();
    ui->zWidget->hide();
    ui->IsFilledCheckBox->hide();
    ui->DensityEdit->hide();
    ui->DensitySlider->hide();
    ui->ColorEdit1->hide();
    ui->RedSlider->hide();
    ui->ColorEdit2->hide();
    ui->GreenSlider->hide();
    ui->ColorEdit3->hide();
    ui->BlueSlider->hide();

    ui->EditedShapeName->setTitle(editData.name.data());
    ui->ShowNormals->setChecked(editData.areNormalsPresent);
    if (editData.showLabels.at(0))
    {
        ui->xWidget->show();
        ui->xLabel->setText(editData.labels.at(0).data());
        ui->xInput->setValidator(new QDoubleValidator(2.0, 999999.99, 6));
        ui->xInput->setText(std::to_string(editData.dim.at(0)).data());
    }
    if (editData.showLabels.at(1))
    {
        ui->yWidget->show();
        ui->yLabel->setText(editData.labels.at(1).data());
        ui->yInput->setValidator(new QDoubleValidator(2.0, 999999.99, 6));
        ui->yInput->setText(std::to_string(editData.dim.at(1)).data());
    }
    if (editData.showLabels.at(2))
    {
        ui->zWidget->show();
        ui->zLabel->setText(editData.labels.at(2).data());
        ui->zInput->setValidator(new QDoubleValidator(2.0, 999999.99, 6));
        ui->zInput->setText(std::to_string(editData.dim.at(2)).data());
    }
    if (editData.showFilledEdit)
    {
        ui->IsFilledCheckBox->setChecked(editData.isFilled);
        ui->IsFilledCheckBox->show();
    }
    if (editData.showDensityEdit)
    {
        ui->DensityNumber->display(editData.density);
        ui->DensitySlider->setValue(editData.density);

        ui->DensityEdit->show();
        ui->DensitySlider->show();
    }
    if (editData.showColorEdit)
    {
        ui->RedNumber->display(editData.rgb.at(0));
        ui->RedSlider->setValue(editData.rgb.at(0));
        ui->ColorEdit1->show();
        ui->RedSlider->show();

        ui->GreenNumber->display(editData.rgb.at(1));
        ui->GreenSlider->setValue(editData.rgb.at(1));
        ui->ColorEdit2->show();
        ui->GreenSlider->show();

        ui->BlueNumber->display(editData.rgb.at(2));
        ui->BlueSlider->setValue(editData.rgb.at(2));
        ui->ColorEdit3->show();
        ui->BlueSlider->show();
    }

    ui->EditCloudsWidget->show();
}

void MainWindow::changeToAddShapeWidget()
{
    ui->EditCloudsWidget->hide();
    ui->AddCloudsWidget->show();
}

MainWindow::~MainWindow()
{
    delete ui;
}
