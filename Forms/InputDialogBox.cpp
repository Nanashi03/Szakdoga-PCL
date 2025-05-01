#include "InputDialogBox.h"
#include "ui_InputDialogBox.h"

InputDialogBox::InputDialogBox(QWidget *parent, const std::vector<std::string>& labels, const std::vector<bool>& showData, InputFormData& data) : QDialog(parent),
    ui {new Ui::InputDialogBox},
    inputFormData {data}
{
    ui->setupUi(this);
    auto numberValidator { std::make_shared<QRegExp>("(([1-9][0-9]{1,2})|([2-9]))|((([1-9][0-9]{1,2})|([2-9]))\\.[0-9]{1,6})") };

    if (showData[0]) {
        ui->InputLabel1->setText((labels.at(0)+":").data());
        ui->InputBox1->setValidator(new QRegExpValidator(*numberValidator));
        ui->InputBox1->setText("2.0");
        ui->InputBox1->setPlaceholderText("2.0");

        ui->InputLabel1->show();
        ui->InputBox1->show();
    }
    if (showData[1]) {
        ui->InputLabel2->setText((labels.at(1)+":").data());
        ui->InputBox2->setValidator(new QRegExpValidator(*numberValidator));
        ui->InputBox2->setText("2.0");
        ui->InputBox2->setPlaceholderText("2.0");

        ui->InputLabel2->show();
        ui->InputBox2->show();
    }
    if (showData[2]) {
        ui->InputLabel3->setText((labels.at(2)+":").data());
        ui->InputBox3->setValidator(new QRegExpValidator(*numberValidator));
        ui->InputBox3->setText("2.0");
        ui->InputBox3->setPlaceholderText("2.0");

        ui->InputLabel3->show();
        ui->InputBox3->show();
    }
}

void InputDialogBox::accept() {
    if (ui->IdInputBox->text().isEmpty())
    {
        QMessageBox::critical(this, "Error", "Shape name must be filled!");
        return;
    }
    if (!ui->InputBox1->hasAcceptableInput() || !ui->InputBox2->hasAcceptableInput() || !ui->InputBox3->hasAcceptableInput())
    {
        QMessageBox::critical(this, "Error", R"(For correct decimal input use "." instead of ","!)");
        return;
    }

    inputFormData.id = ui->IdInputBox->text().toStdString();
    inputFormData.x = ui->InputBox1->text().toDouble();
    inputFormData.y = ui->InputBox2->text().toDouble();
    inputFormData.z = ui->InputBox3->text().toDouble();
    inputFormData.isFilled = ui->isFilledCheckBox->checkState();

    QDialog::accept();
}

InputDialogBox::~InputDialogBox()
{
    delete ui;
}

