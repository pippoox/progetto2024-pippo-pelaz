#include "visualizzazione.hpp"
#include "boids.hpp"
namespace sf {
Visualizzazione::Visualizzazione(unsigned int width, unsigned int height)
    : windowWidth(width), windowHeight(height) {}

void Visualizzazione::disegnaBoids(const std::vector<sf::Vector2f>& posizioni,
                                   sf::RenderWindow& window) {
  for (const auto& pos : posizioni) {
    sf::CircleShape boidShape(5.0);
    boidShape.setFillColor(sf::Color::Green);
    boidShape.setPosition(pos);
    window.draw(boidShape);
  }
}
}  // namespace b