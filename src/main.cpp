#include "include/time.hpp"
#include <iostream>

int main(int argc, char** argv) {
  utils::time start = utils::Time();
  utils::time end = utils::Time();
  double duration = utils::Duration(start, end);
  std::cout << "duration = " << duration << std::endl;
}