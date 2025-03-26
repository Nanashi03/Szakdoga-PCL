#include "Controller/Controller.h"

int main (int argc, char** argv) {
  Controller controller;

  controller.generateSphere("id1", 5, 1.0f);
  controller.start();

  return 0;
}