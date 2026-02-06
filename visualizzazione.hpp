#ifndef VISUALIZZAZIONE_HPP
#define VISUALIZZAZIONE_HPP

#include <SFML/Graphics.hpp>

#include "boids.hpp"
namespace b {
// Classe SimulationRenderer: gestisce il rendering dei boid in una finestra SFML
// - windowWidth e windowHeight: dimensioni della finestra
// - drawBoids: disegna i boid dati in input (posizioni)
class SimulationRenderer {
 public:
  SimulationRenderer();

  
  void drawBoids(const std::vector<Boid>& boids ,
                    sf::RenderWindow& window); // Disegna i boid usando le loro posizioni

 private:
  sf::ConvexShape boidShape;
};
}  // namespace b

#endif