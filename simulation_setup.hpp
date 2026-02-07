#ifndef B_SIMULATION_SETUP_HPP
#define B_SIMULATION_SETUP_HPP

#include <vector>
#include "boids.hpp"

namespace b {

// Struct SimConfig: contiene gli oggetti necessari per creare la simualzione.
struct simConfig {
  std::size_t N = 0;   // Numero di boid
  std::size_t NF = 0;  // Numero di stormi
  std::vector<Flock> flocks;
  std::vector<std::size_t> count;  // Vettore che contiene numero di boid per ogni flock

  double width;
  double height;
  double dt;
};

simConfig readConfig();  // Gestisce input utente per costruire simulazione
BoidSimulation buildSimulation(const simConfig& cfg);
// Costruisce la simulazione ricevendo input utente e generando posizioni e
// velocità casuali per ogni boid

}  // namespace b

#endif