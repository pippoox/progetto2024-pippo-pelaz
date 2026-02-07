#ifndef B_STATS_HPP
#define B_STATS_HPP

#include <vector>

#include "boids.hpp"

namespace b {

// Stats struct: holds parameters representing the mean and standard deviation.
struct Stats {
  double mean;
  double stdDev;
};

Stats computeMeanDistance(const std::vector<b::Boid>& boids);
Stats computeMeanSpeed(const std::vector<b::Boid>& boids);

}  // namespace b

#endif