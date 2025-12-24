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
<<<<<<< HEAD
  void disegnaBoids(const std::vector<sf::Vector2f>& posizioni,
=======
  void disegnaBoids(const std::vector<Boid>& boidData,
>>>>>>> master
                    sf::RenderWindow& window);

 private:
  unsigned int windowWidth;
  unsigned int windowHeight;
<<<<<<< HEAD
=======
  sf::ConvexShape boidShape;
>>>>>>> master
};
}  // namespace b

#endif