
#include "stats.hpp"

#include <cmath>
#include <vector>

#include "boids.hpp"

namespace b {

static Stats finalizeStats(double sum, double sumSq, size_t count) {
  if (count == 0) return {0.0, 0.0};

  const double mean = sum / static_cast<double>(count);
  double var = (sumSq / static_cast<double>(count)) - mean * mean;
  if (var < 0.0) var = 0.0;

  return {mean, std::sqrt(var)};
}

// Calculates the average distance and its standard deviation between all unique
// pairs of boids in the population.
Stats computeMeanDistance(const std::vector<b::Boid>& boids) {
  size_t n = boids.size();
  if (n < 2) return {0.0, 0.0};

  double sum = 0.0;
  double sumSq = 0.0;
  size_t pairs = 0;

  for (size_t i = 0; i < n; ++i) {
    for (size_t j = i + 1; j < n; ++j) {
      const double d = (boids[i].position - boids[j].position).length();
      sum += d;
      sumSq += d * d;
      ++pairs;
    }
  }

  return finalizeStats(sum, sumSq, pairs);
}

// Computes the average speed and velocity dispersion across the entire boid
// population.
Stats computeMeanSpeed(const std::vector<b::Boid>& boids) {
  double sum = 0.0;
  double sumSq = 0.0;

  for (const auto& b : boids) {
    const double d = b.speed.length();
    sum += d;
    sumSq += d * d;
  }
  return finalizeStats(sum, sumSq, boids.size());
}

}  // namespace b