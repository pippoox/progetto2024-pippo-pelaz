#ifndef VISUALIZZAZIONE_HPP
#define VISUALIZZAZIONE_HPP

#include <SFML/Graphics.hpp>

#include "boids.hpp"
namespace sf {
class Visualizzazione {
 public:
  Visualizzazione(unsigned int width, unsigned int height);
  void disegnaBoids(const std::vector<sf::Vector2f>& posizioni,
                    sf::RenderWindow& window);

 private:
  unsigned int windowWidth;
  unsigned int windowHeight;
};
}  // namespace b

#endif