#include "timer/timer.h"
#include "filter/digital_filter.h"
#include "filter/first_oeder_lowpass_filter.h"
#include "filter/second_order_lowpass_filter.h"
#include "filter/mean_filter.h"
#include "filter/kalman_filter.h"

#include <iostream>

int main(int argc, char** argv) {
  utils::time start = utils::Time();
  utils::time end = utils::Time();
  double duration = utils::Duration(start, end);
  std::cout << "duration = " << duration << std::endl;
}