#include <algorithm>
#include <iostream>
#include <random>

#include "boids.hpp"
#include "simulation_setup.hpp"
#include "stats.hpp"

int main() {

  auto cfg = b::readConfig();
  auto sim = b::buildSimulation(cfg);

  const int updates = 100;
  for (int step = 0; step < updates; ++step) {
    sim.updateBoids(cfg.dt);

    auto meanDistance = b::computeMeanDistance(sim.getBoids());
    auto meanSpeed = b::computeMeanSpeed(sim.getBoids());
    // Output: time, distance mean and dev std, velocity mean and dev std
    std::cout << step * cfg.dt << " " << meanDistance.mean << " "
              << meanDistance.stdDev << " " << meanSpeed.mean << " "
              << meanSpeed.stdDev << "\n";
  }

  return 0;
}