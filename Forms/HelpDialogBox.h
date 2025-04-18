#ifndef HELPDIALOGBOX_H
#define HELPDIALOGBOX_H

#include <QDialog>
#include <QLabel>
#include <QPixmap>

class AutoScalingLabel : public QLabel {
    Q_OBJECT

public:
    explicit AutoScalingLabel(QWidget* parent = nullptr) : QLabel(parent)
    {
        setAlignment(Qt::AlignCenter);
        setScaledContents(false);
    }

    void setIcon(const QPixmap& pixmap) {
        originalPixmap = pixmap;
        updatePixmap();
    }

protected:
    void resizeEvent(QResizeEvent* event) override {
        QLabel::resizeEvent(event);
        updatePixmap();
    }

private:
    QPixmap originalPixmap;
    void updatePixmap() {
        if (!originalPixmap.isNull()) {
            QPixmap scaled = originalPixmap.scaled(size(), Qt::KeepAspectRatio, Qt::SmoothTransformation);
            QLabel::setPixmap(scaled);
        }
    }
};

namespace Ui {
class HelpDialogBox;
}

class HelpDialogBox : public QDialog
{
    Q_OBJECT

public:
    explicit HelpDialogBox(QWidget *parent = nullptr);
    ~HelpDialogBox();

private:
    Ui::HelpDialogBox *ui;
};

#endif // HELPDIALOGBOX_H
