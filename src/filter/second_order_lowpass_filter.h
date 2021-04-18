#ifndef UTILS_FILTER_SECOND_ORDER_LOWPASS_FILTER_H_
#define UTILS_FILTER_SECOND_ORDER_LOWPASS_FILTER_H_

#include "filter/digital_filter.h"

#include <vector>
/**
 * @file
 * @brief Defines Second Order DigitalFilter(ButterWorth Filter) class
 *                            wn^2
 *        G(s) = ---------------------------------            
 *                s^2 + 2 * 0.707 * wn * s + wn^2
 *          wn = 2 * M_PI * fn 
 */

namespace utils {
class SecondOrderLowPassFilter{
 public:
  SecondOrderLowPassFilter() = default;
  ~SecondOrderLowPassFilter() = default;
  SecondOrderLowPassFilter(const double sample_time,
                          const double cutoff_freq,
                          const double dead_zone = 0.0) {
    Init(sample_time, cutoff_freq, dead_zone);
  }
  void Init(const double sample_time,
            const double cutoff_freq,
            const double dead_zone = 0.0);

  double Filter(const double x_insert) { return digital_filter_.Filter(x_insert); }

  std::vector<double> GetNumerators() const { return numerators_; }

  std::vector<double> GetDenominators() const { return denominators_; }
  
  double GetSampleTime() const { return sample_time_; }

  double GetCutoffFreq() const { return cutoff_freq_; }

  double GetDeadZone() const { return dead_zone_; }
 private:
  // Coefficients with x values
  std::vector<double> numerators_;
  // Coefficients with y values
  std::vector<double> denominators_;

  double sample_time_ = 0.0;
  double cutoff_freq_ = 0.0;
  double dead_zone_ = 0.0;

  DigitalFilter digital_filter_;
};

void SecondOrderLowPassFilter::Init(const double sample_time,
                                   const double cutoff_freq,
                                   const double dead_zone) {
  if (sample_time <= 0.0 || cutoff_freq <= 0.0) {
    return;
  }
  
  sample_time_ = sample_time;
  cutoff_freq_ = cutoff_freq;
  dead_zone_ = dead_zone;

  denominators_.clear();
  numerators_.clear();
  denominators_.reserve(3);
  numerators_.reserve(3);

  double wa = 2.0 * M_PI * cutoff_freq;  // Analog frequency in rad/s
  double alpha = wa * sample_time / 2.0;          // tan(Wd/2), Wd is discrete frequency
  double alpha_sqr = alpha * alpha;
  double tmp_term = std::sqrt(2.0) * alpha + alpha_sqr;
  double gain = alpha_sqr / (1.0 + tmp_term);

  denominators_.push_back(1.0);
  denominators_.push_back(2.0 * (alpha_sqr - 1.0) / (1.0 + tmp_term));
  denominators_.push_back((1.0 - std::sqrt(2.0) * alpha + alpha_sqr) /
                          (1.0 + tmp_term));

  numerators_.push_back(gain);
  numerators_.push_back(2.0 * gain);
  numerators_.push_back(gain);
  
  digital_filter_.Init(numerators_, denominators_, dead_zone);
}

} // namespace utils

#endif // UTILS_FILTER_SECOND_ORDER_LOWPASS_FILTER_H_