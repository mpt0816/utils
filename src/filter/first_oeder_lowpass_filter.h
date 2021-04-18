#ifndef UTILS_FILTER_FIRST_ORDER_LOWPASS_FILTER_H_
#define UTILS_FILTER_FIRST_ORDER_LOWPASS_FILTER_H_

#include "filter/digital_filter.h"

#include <vector>

namespace utils {

class FirstOrderLowPassFilter{
 public:
  FirstOrderLowPassFilter() = default;
  ~FirstOrderLowPassFilter() = default;
  FirstOrderLowPassFilter(const double sample_time,
                          const double cutoff_freq,
                          const double dead_zone = 0.0) {
    Init(sample_time, cutoff_freq, dead_zone);
  }
  void Init(const double sample_time,
            const double cutoff_freq,
            const double dead_zone = 0.0);

  void InitTimeDomain(const double sample_time,
                      const double settling_time,
                      const double dead_zone = 0.0,
                      const double dead_time = 0.0);

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

void FirstOrderLowPassFilter::Init(const double sample_time,
                                   const double cutoff_freq,
                                   const double dead_zone) {
  if (sample_time <= 0.0 || cutoff_freq <= 0.0) {
    return;
  }
    
  sample_time_ = sample_time;
  cutoff_freq_ = cutoff_freq;
  dead_zone_ = dead_zone;

  numerators_.clear();
  denominators_.clear();
  numerators_.reserve(2);
  denominators_.reserve(2);
   
  double wn = 2 * M_PI * cutoff_freq;

  numerators_.insert(numerators_.end(), 2, wn * sample_time);
  denominators_.push_back(wn * sample_time + 2);
  denominators_.push_back(wn * sample_time - 2);
  
  digital_filter_.Init(numerators_, denominators_, dead_zone);
}

void FirstOrderLowPassFilter::InitTimeDomain(const double sample_time,
                      const double settling_time,
                      const double dead_zone,
                      const double dead_time) {
    if (sample_time <= 0.0 || settling_time < 0.0 || dead_time < 0.0) {
    return;
  } 
  
  const size_t k_d = static_cast<size_t>(dead_time / sample_time);
  double a_term = 0.0;

  numerators_.clear();
  denominators_.clear();
  numerators_.reserve(k_d + 1);
  denominators_.reserve(2);

  if (settling_time == 0.0) {
    a_term = 0.0;
  } else {
    a_term = exp(-1 * sample_time / settling_time);
  }
  
  numerators_.insert(numerators_.end(), k_d, 0.0);
  numerators_.push_back(1 - a_term);
  denominators_.push_back(1.0);
  denominators_.push_back(-a_term);

  digital_filter_.Init(numerators_, denominators_, dead_zone);
}

} // namespace utils

#endif // UTILS_FILTER_FIRST_ORDER_LOWPASS_FILTER_H_