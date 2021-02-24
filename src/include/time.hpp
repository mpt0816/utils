#ifndef UTILS_TIME_H_
#define UTILS_TIME_H_

#include <chrono>

namespace utils {

// 毫秒 1 s = 1000 ms
typedef std::chrono::time_point<std::chrono::steady_clock,
                                std::chrono::milliseconds> time;
static inline time Time() {
  return std::chrono::time_point_cast<std::chrono::milliseconds>(
      std::chrono::steady_clock::now());
}

static inline double Duration(const time& start, const time& end) {
  return std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
}

} // namespace utils

#endif // UTILS_TIME_H_