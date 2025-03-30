#include "Controller/Controller.h"

int main (int argc, char** argv) {
  Controller controller;

  controller.generateCircle("id", true, 5, 0.7);
  controller.start();

  return 0;
}