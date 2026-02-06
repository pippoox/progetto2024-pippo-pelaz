
#include "boids.hpp"

namespace b {

// Struct SimConfig: contiene gli oggetti necessari per creare la simualzione.
struct simConfig {
  int input;
  std::size_t N = 0;   // Numero di boid
  std::size_t NF = 0;  // Numero di stormi
  std::vector<Flock> flocks;
  std::vector<std::size_t>
      count;  // Vettore che contiene numero di boid per ogni flock
};

simConfig readConfig();  // Gestisce input utente per costruire simulazione
BoidSimulation buildSimulation(const simConfig& cfg, const double& width,
                               const double& height);
// Costruisce la simulazione ricevendo input utente e generando posizioni e
// velocità casuali per ogni boid

}  // namespace b