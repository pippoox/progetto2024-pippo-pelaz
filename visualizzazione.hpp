#ifndef VISUALIZZAZIONE_HPP
#define VISUALIZZAZIONE_HPP

#include <SFML/Graphics.hpp>

#include "boids.hpp"
namespace b {
// Classe Visualizzazione
// Scopo: gestisce il rendering dei boid in una finestra SFML
// - windowWidth e windowHeight: dimensioni della finestra
// - disegnaBoids: disegna i boid dati in input (posizioni)
class Visualizzazione {
 public:
  Visualizzazione(unsigned int width, unsigned int height);

  // Disegna i boid usando le loro posizioni
  void disegnaBoids(const std::vector<std::vector<Boid>>& flocks,
                    sf::RenderWindow& window);

 private:
  unsigned int windowWidth;
  unsigned int windowHeight;
  sf::ConvexShape boidShape;
};
}  // namespace b

#endif