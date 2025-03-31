#include "Controller/Controller.h"

int main (int argc, char** argv) {
  Controller controller;

  //controller.generateCylinder("im", true, 5,5,1);
  //controller.generateCone("ima", true, 5,10,1);
  //controller.generateCircle("cc", true, 5, 1.7);
  //controller.generateRectangle("ccc", false, 5, 7, 1.0);
  //controller.tmp();
  controller.importCloud("id", "col.pcd");
  controller.start();

  return 0;
}