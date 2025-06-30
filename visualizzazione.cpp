#include "visualizzazione.hpp"

#include "boids.hpp"
namespace b {
// Costruttore della classe Visualizzazione
// Imposta le dimensioni della finestra
Visualizzazione::Visualizzazione(unsigned int width, unsigned int height)
    : windowWidth(width), windowHeight(height) {}

// Metodo per disegnare i boid
// - Prende un vettore di posizioni (sf::Vector2f)
// - Disegna un cerchio verde per ogni boid
// - Li disegna sulla finestra passata come riferimento
void Visualizzazione::disegnaBoids(const std::vector<sf::Vector2f>& posizioni,
                                   sf::RenderWindow& window) {
  for (const auto& pos : posizioni) {
    sf::CircleShape boidShape(5.0); //Disegna un cerchio con raggio 5
    boidShape.setFillColor(sf::Color::Green);
    boidShape.setPosition(pos);
    window.draw(boidShape);
  }
}
}  // namespace b