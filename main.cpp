#include "Controller/Controller.h"

int main (int argc, char** argv) {
  Controller controller;

  controller.generateCylinder("im", true, 5,5,1);
  controller.start();

  return 0;
}