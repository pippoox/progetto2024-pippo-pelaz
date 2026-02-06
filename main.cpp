#include <algorithm>
#include <iostream>
#include <random>

#include "boids.hpp"
#include "simulation_setup.hpp"

// Funzione per calcolare la distanza mean e deviazione standard
void computeMeanDistance(const std::vector<b::Boid>& boids, double& mean,
                          double& devStd) {
  size_t n = boids.size();
  std::vector<double> distances;
  // Calcolo della distanza tra tutte le coppie di boid
  for (size_t i = 0; i < n; ++i) {
    for (size_t j = i + 1; j < n; ++j) {
      double d = (boids[i].position - boids[j].position).length();
      distances.push_back(d);
    }
  }
  // Media delle distances
  double sum = 0.0;
  for (double d : distances) sum += d;
  mean = sum / static_cast<double>(distances.size());

  // Deviazione standard delle distances
  double sumOfSquares = 0.0;
  for (double d : distances) sumOfSquares += (d - mean) * (d - mean);
  devStd = std::sqrt(sumOfSquares / static_cast<double>(distances.size()));
}

// Funzione per calcolare la velocity mean e deviazione standard
void computeMeanVelocity(const std::vector<b::Boid>& boids, double& mean,
                          double& devStd) {
  size_t n = boids.size();
  std::vector<double> magnitudes;
  // Calcola length (intensità) della velocity per ogni boid
  for (const auto& b : boids) {
    magnitudes.push_back(b.speed.length());
  }
  // Media delle velocity
  double sum = 0.0;
  for (double v : magnitudes) sum += v;
  mean = sum / static_cast<double>(n);

  // Deviazione standard delle velocity
  double sumOfSquares = 0.0;
  for (double v : magnitudes) sumOfSquares += (v - mean) * (v - mean);
  devStd = std::sqrt(sumOfSquares / static_cast<double>(n));
}

int main() {
  const double width = 800;
  const double height = 600;
  const double dt = 1.;

  auto cfg = b::readConfig();
  auto sim = b::buildSimulation(cfg, width, height);
  
  const int updates = 100;
  for (int step = 0; step < updates; ++step) {
    sim.updateBoids(dt);

    double meanDistance;
    double distanceDevStd;
    computeMeanDistance(sim.getBoids(), meanDistance, distanceDevStd);

    double meanVelocity; 
    double velocityDevStd;
    computeMeanVelocity(sim.getBoids(), meanVelocity, velocityDevStd);

    // Output: tempo, distanza mean e dev std, velocity mean e dev std
    std::cout << step * dt << " " << meanDistance << " " << distanceDevStd << " "
              << meanVelocity << " " << velocityDevStd << "\n";
  }

  return 0;
}