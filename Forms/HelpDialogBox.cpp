#include "HelpDialogBox.h"
#include "ui_HelpDialogBox.h"

HelpDialogBox::HelpDialogBox(QWidget *parent)
    : QDialog(parent)
    , ui(new Ui::HelpDialogBox)
{
    ui->setupUi(this);

    ui->rotCamDrag->setIcon(QPixmap("../Assets/left_click_drag.png"));
    ui->moveCamShift->setIcon(QPixmap("../Assets/shift.png"));
    ui->moveCamDrag->setIcon(QPixmap("../Assets/left_click_drag.png"));
    ui->displayGrid->setIcon(QPixmap("../Assets/letter_g.png"));
    ui->displayLookUp->setIcon(QPixmap("../Assets/letter_u.png"));
    ui->incPointSize->setIcon(QPixmap("../Assets/letter_plus.png"));
    ui->decPointSize->setIcon(QPixmap("../Assets/letter_minus.png"));
    ui->movUX->setIcon(QPixmap("../Assets/arrow_left.png"));
    ui->movDX->setIcon(QPixmap("../Assets/arrow_right.png"));
    ui->movUY->setIcon(QPixmap("../Assets/letter_a.png"));
    ui->movDY->setIcon(QPixmap("../Assets/letter_d.png"));
    ui->movUZ->setIcon(QPixmap("../Assets/arrow_up.png"));
    ui->movDZ->setIcon(QPixmap("../Assets/arrow_down.png"));
    ui->zoomScroll->setIcon(QPixmap("../Assets/scroll.png"));
    ui->selShift->setIcon(QPixmap("../Assets/shift.png"));
    ui->selLeftClick->setIcon(QPixmap("../Assets/left_click.png"));
    ui->flyP->setIcon(QPixmap("../Assets/letter_f.png"));
    ui->camReset->setIcon(QPixmap("../Assets/letter_r.png"));
}

HelpDialogBox::~HelpDialogBox()
{
    delete ui;
}
