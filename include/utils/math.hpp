#pragma once
#include <cassert>
#include <cmath>
#include <numeric>

namespace Math {

constexpr double kdeg2rad = M_PI / 180.0;

template <typename DataType, typename StaticsType, typename GetterType>
void MeanVariance(const DataType &data, StaticsType &mean,
                  StaticsType &variance, GetterType getter) {
  mean = std::accumulate(data.begin(), data.end(), StaticsType::Zero().eval(),
                         [&getter](const StaticsType &sum, const auto &data)
                             -> StaticsType { return sum + getter(data); }) /
         data.size();

  variance =
      std::accumulate(data.begin(), data.end(), StaticsType::Zero().eval(),
                      [&mean, &getter](const StaticsType &sum,
                                       const auto &data) -> StaticsType {
                        return sum + (getter(data) - mean).cwiseAbs2().eval();
                      }) /
      (data.size() - 1);
}

} // namespace Math