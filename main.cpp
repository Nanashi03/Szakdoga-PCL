#include "Controller/Controller.h"

int main (int argc, char** argv) {
  Controller controller;

  controller.generateCylinder("im", true, 5,5,1);
  //controller.generateCone("ima", true, 5,10,1);
  controller.tmp();
  controller.start();

  return 0;
}