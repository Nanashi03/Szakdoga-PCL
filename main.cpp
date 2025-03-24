#include <iostream>
#include <thread>
#include "Controller/Controller.h"

int main (int argc, char** argv) {
  Controller controller;

  controller.generateSphere("rectangle1", 5, 1);
  controller.start();

  return 0;
}