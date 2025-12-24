#include "visualizzazione.hpp"
<<<<<<< HEAD

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
=======
#include "boids.hpp"

#include <cmath>

namespace b {
// Costruttore della classe Visualizzazione
// Imposta le dimensioni della finestra
const double pi = 3.14159265359;

Visualizzazione::Visualizzazione(unsigned int width, unsigned int height)
    : windowWidth(width), windowHeight(height) {
  boidShape.setPointCount(3);
  const double size = 10.;
  const double width_boid = 5.;

  boidShape.setPoint(0, sf::Vector2f(0., -size));
  boidShape.setPoint(1, sf::Vector2f(width_boid / 2., size / 2.));
  boidShape.setPoint(2, sf::Vector2f(-width_boid / 2., size / 2.));

  boidShape.setOrigin(0., 0.);
  boidShape.setFillColor(sf::Color::Green);
}

// Metodo per disegnare i boid

void Visualizzazione::disegnaBoids(const std::vector<Boid>& boidData,
                                   sf::RenderWindow& window) {
  for (const auto& boid : boidData) {
    double vx = static_cast<double>(boid.velocità.x);
    double vy = static_cast<double>(boid.velocità.y);
    double px = static_cast<double>(boid.posizione.x);
    double py = static_cast<double>(boid.posizione.y);

    // Calcola l'angolo in radianti (atan2(y, x))
    double angleRad = std::atan2(vy, vx);

    // Converte i radianti in gradi
    double angleDeg = angleRad * (180. / pi);
    
    // Applica la correzione di +90 gradi. 
    // CiÃ² compensa il fatto che atan2 misura da +X (destra)
    // mentre il nostro triangolo a 0 gradi punta verso -Y (alto).
    double rotation = angleDeg + 90.; 

    // 1. Applica la posizione
    boidShape.setPosition(px, py);
    // 2. Applica la rotazione
    boidShape.setRotation(rotation);
   
>>>>>>> master
    window.draw(boidShape);
  }
}
}  // namespace b