#ifndef UTILS_FILTER_MEAN_FILTER_H_
#define UTILS_FILTER_MEAN_FILTER_H_

#include <deque>
#include <limits>

namespace utils {

class MeanFilter {
 public:
  explicit MeanFilter(const size_t window_size);

  MeanFilter() = default;
  ~MeanFilter() = default;

  double Filter(const double x_insert);

 private:
  void RemoveEarliest();
  void Insert(const double value);
  double GetMin() const;
  double GetMax() const;
  bool ShouldPopOldestCandidate(const size_t old_time) const;
  size_t window_size_ = 0;
  double sum_ = 0.0;
  size_t time_ = 0;
  // front = earliest
  std::deque<double> values_;
  // front = min
  std::deque<std::pair<size_t, double>> min_candidates_;
  // front = max
  std::deque<std::pair<size_t, double>> max_candidates_;
  bool initialized_ = false;
};

MeanFilter::MeanFilter(const size_t window_size) 
    : window_size_(window_size) {
  initialized_ = true;
}

double MeanFilter::GetMin() const {
  if (min_candidates_.empty()) {
    return std::numeric_limits<double>::infinity();
  } else {
    return min_candidates_.front().second;
  }
}

double MeanFilter::GetMax() const {
  if (max_candidates_.empty()) {
    return -std::numeric_limits<double>::infinity();
  } else {
    return max_candidates_.front().second;
  }
}

double MeanFilter::Filter(const double x_insert) {
  ++time_;
  time_ %= static_cast<size_t>(2 * window_size_);
  if (values_.size() == window_size_) {
    RemoveEarliest();
  }
  Insert(x_insert);
  if (values_.size() > 2) {
    return (sum_ - GetMin() - GetMax()) /
           static_cast<double>(values_.size() - 2);
  } else {
    return sum_ / static_cast<double>(values_.size());
  }
}

bool MeanFilter::ShouldPopOldestCandidate(const size_t old_time) const {
  if (old_time < window_size_) {
    return old_time + window_size_ == time_;
  } else if (time_ < window_size_) {
    return old_time == time_ + window_size_;
  } else {
    return false;
  }
}

void MeanFilter::RemoveEarliest() {
  double removed = values_.front();
  values_.pop_front();
  sum_ -= removed;
  if (ShouldPopOldestCandidate(min_candidates_.front().first)) {
    min_candidates_.pop_front();
  }
  if (ShouldPopOldestCandidate(max_candidates_.front().first)) {
    max_candidates_.pop_front();
  }
}

void MeanFilter::Insert(const double value) {
  values_.push_back(value);
  sum_ += value;
  while (min_candidates_.size() > 0 && min_candidates_.back().second > value) {
    min_candidates_.pop_back();
  }
  min_candidates_.push_back(std::make_pair(time_, value));
  while (max_candidates_.size() > 0 && max_candidates_.back().second < value) {
    max_candidates_.pop_back();
  }
  max_candidates_.push_back(std::make_pair(time_, value));
}

} // namespace utils

#endif // UTILS_FILTER_MEAN_FILTER_H_