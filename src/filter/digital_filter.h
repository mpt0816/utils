#ifndef UTILS_FILTER_DIGITAL_FILTER_H_
#define UTILS_FILTER_DIGITAL_FILTER_H_

#include <deque>
#include <vector>
#include <cmath>

/**
 * @file
 * @brief Defines Universal DigitalFilter class
 *                a_n * z^n + a_n-1 * z^n-1 + ... + a_0
 *        G(z) = ---------------------------------------            
 *                b_n * z^n + b_n-1 * z^n-1 + ... + b_0
 */

namespace {
  const double kDoubleEpsilon = 1.0e-6;
} // namespace

namespace utils {
class DigitalFilter {
 public:
  DigitalFilter() = default;
  DigitalFilter(const std::vector<double>& numerators,
                const std::vector<double>& denominators,
                const double dead_zone = 0.0) {
    Init(numerators, denominators, dead_zone);
  }

  ~DigitalFilter() = default;
  void Init(const std::vector<double>& numerators,
            const std::vector<double>& denominators,
            const double dead_zone = 0.0);
  
  double Filter(const double x_insert);

  void Reset();

  void SetNumerators(const std::vector<double>& numerators) {
    numerators_ = numerators;
    x_values_.resize(numerators_.size(), 0.0);
  }

  void SetDenominators(const std::vector<double>& denominators) {
    denominators_ = denominators;
    y_values_.resize(denominators_.size(), 0.0);
  }

  void SetDeadZone(const double dead_zone) { dead_zone_ = std::fabs(dead_zone); }

  std::vector<double> GetNumerators() const { return numerators_; }

  std::vector<double> GetDenominators() const { return denominators_; }

  double GetDeadZone() const { return dead_zone_; }

 private:
  double Compute(const std::deque<double>& values,
                 const std::vector<double>& coefficients,
                 const std::size_t coeff_start,
                 const std::size_t coeff_end);
  
  double UpdateLast(const double input);

 private:
  // Front is latest, back is oldest.
  std::deque<double> x_values_;
  // Front is latest, back is oldest.
  std::deque<double> y_values_;
  // Coefficients with x values
  std::vector<double> numerators_;
  // Coefficients with y values
  std::vector<double> denominators_;
  // threshold of updating last-filtered value
  double dead_zone_ = 0.0;
  // last-filtered value
  double last_ = 0.0;
};

void DigitalFilter::Init(const std::vector<double>& numerators,
                         const std::vector<double>& denominators,
                         const double dead_zone) {
  SetDeadZone(dead_zone);
  SetNumerators(numerators);
  SetDenominators(denominators);
}

double DigitalFilter::Filter(const double x_insert) {
  if (numerators_.empty() || denominators_.empty()) {
    return 0.0;
  }

  x_values_.pop_back();
  x_values_.push_front(x_insert);
  const double xside = 
      Compute(x_values_, numerators_, 0, numerators_.size() - 1);
  
  y_values_.pop_back();
  const double yside = 
      Compute(y_values_, denominators_, 1, denominators_.size() - 1);
  
  double y_insert = 0.0;
  if (std::fabs(denominators_.front()) > kDoubleEpsilon) {
    y_insert = (xside - yside) / denominators_.front();
  }
  y_values_.push_front(y_insert);
  
  return UpdateLast(y_insert);
}

void DigitalFilter::Reset() {
  std::fill(x_values_.begin(), x_values_.end(), 0.0);
  std::fill(y_values_.begin(), y_values_.end(), 0.0);
}

inline double DigitalFilter::Compute(const std::deque<double>& values,
                                     const std::vector<double>& coefficients,
                                     const std::size_t coeff_start,
                                     const std::size_t coeff_end) {
  if (coeff_end >= coefficients.size() ||
      coeff_start > coeff_end ||
      coeff_end - coeff_start + 1 == coefficients.size()) {
    return 0.0;
  }
  double sum = 0.0;
  auto i = coeff_start;
  for (const double& value : values) {
    sum += value * coefficients.at(i);
    ++i;
  }
  return sum;
}

inline double DigitalFilter::UpdateLast(const double input) {
  const double diff = std::fabs(input - last_);
  if (diff < dead_zone_) {
    return last_;
  }
    last_ = input;
  return input;
}

} // namespace utils

#endif // UTILS_FILTER_DIGITAL_FILTER_H_