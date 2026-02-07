#ifndef B_SIMULATION_SETUP_HPP
#define B_SIMULATION_SETUP_HPP

#include <vector>

#include "boids.hpp"

namespace b {

// SimConfig struct: holds the parameters required to initialize the simulation
struct simConfig {
  std::size_t N = 0;               // Total number of boids
  std::size_t NF = 0;              // Total number of flocks
  std::vector<Flock> flocks;       // Collection of flock parameters
  std::vector<std::size_t> count;  // Number of boids assigned to each flock

  double width;   // Width of the simulation area
  double height;  // Height of the simulation area
  double dt;      // Time step for physics updates (Delta Time)
};

// Handles user input or file parsing to construct the simulation configuration
simConfig readConfig();

// Builds the simulation by processing config and generating random boid states
BoidSimulation buildSimulation(const simConfig& cfg);

}  // namespace b

#endif