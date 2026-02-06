#include <algorithm>
#include <iostream>
#include <random>

#include "boids.hpp"
#include "simulation_setup.hpp"
#include "stats.hpp"

int main() {
  const double width = 800;
  const double height = 600;
  const double dt = 0.5;

  auto cfg = b::readConfig();
  auto sim = b::buildSimulation(cfg, width, height);

  const int updates = 100;
  for (int step = 0; step < updates; ++step) {
    sim.updateBoids(dt);

    auto meanDistance = b::computeMeanDistance(sim.getBoids());
    auto meanVelocity = b::computeMeanVelocity(sim.getBoids());
    // Output: tempo, distanza mean e dev std, velocity mean e dev std
    std::cout << step * dt << " " << meanDistance.mean << " "
              << meanDistance.stdDev << " " << meanVelocity.mean << " "
              << meanVelocity.stdDev << "\n";
  }

  return 0;
}