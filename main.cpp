#include "Controller/Controller.h"
#include "View/mainwindow.h"

#include <QApplication>

int main(int argc, char *argv[])
{
  QApplication a(argc, argv);

  Controller controller;
  controller.tmp();
  controller.start();

  return a.exec();
}