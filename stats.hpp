#ifndef B_STATS_HPP
#define B_STATS_HPP

#include <vector>
#include "boids.hpp"

namespace b {

// Struct stats: contiene parametri che rappresentano media e deviazione standard
struct Stats {
  double mean;
  double stdDev;
};

static Stats finalizeStats(double sum, double sumSq, size_t count); // Funzione ausiliaria per calcolare
// media e deviazione standard (formula matematica)
Stats computeMeanDistance(const std::vector<b::Boid>& boids); 
Stats computeMeanSpeed(const std::vector<b::Boid>& boids);

}  // namespace b

#endif