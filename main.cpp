#include <iostream>
#include <thread>
#include "Controller/Controller.h"

int main (int argc, char** argv) {
  Controller controller;

  controller.importCloud("bunny.pcd");
  controller.start();

  return 0;
}