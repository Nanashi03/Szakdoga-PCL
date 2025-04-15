#ifndef INPUTDIALOGBOX_H
#define INPUTDIALOGBOX_H

#include <QDialog>
#include <QValidator>
#include <QMessageBox>
#include "../DataStructures/InputFormData.h"

namespace Ui {
class InputDialogBox;
}

class InputDialogBox : public QDialog
{
    Q_OBJECT
public:
    explicit InputDialogBox(QWidget*, const std::vector<std::string>&, const std::vector<bool>&, InputFormData&);
    ~InputDialogBox();
protected:
    void accept() override;
private:
    Ui::InputDialogBox *ui;
    InputFormData& inputFormData;
};

#endif // INPUTDIALOGBOX_H
