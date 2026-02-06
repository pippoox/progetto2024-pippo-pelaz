
#include "boids.hpp"

namespace b {

// Struct SimConfig: contiene gli oggetti necessari per creare la simualzione.
struct simConfig {
 int N = 0; // Numero di boid 
 int NF = 0; // Numero di stormi
 std::vector<Flock> flocks; 
 std::vector<int> count; // Vettore che contiene numero di boid per ogni flock
};

simConfig readConfig(); // Gestisce input utente per costruire simulazione
BoidSimulation buildSimulation(const simConfig& cfg, const double& width, const double& height); 
// Costruisce la simulazione ricevendo input utente e generando posizioni e velocità 
// casuali per ogni boid

}