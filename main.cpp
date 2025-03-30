#include "Controller/Controller.h"

int main (int argc, char** argv) {
  Controller controller;

  controller.generateCone("id1", true, 5, 10, 1.0f);
  controller.start();

  return 0;
}