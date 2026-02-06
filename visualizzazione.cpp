#include "visualizzazione.hpp"

#include <cmath>

#include "boids.hpp"

namespace b {
// Costruttore della classe SimulationRenderer
// Imposta le dimensioni della finestra
const double pi = 3.14159265359;

SimulationRenderer::SimulationRenderer() {
  boidShape.setPointCount(3);
  const double size = 10.;
  const double width_boid = 5.;

  boidShape.setPoint(0, sf::Vector2f(0., -size));
  boidShape.setPoint(1, sf::Vector2f(width_boid / 2., size / 2.));
  boidShape.setPoint(2, sf::Vector2f(-width_boid / 2., size / 2.));

  boidShape.setOrigin(0., 0.);
}

static const std::vector<sf::Color> flockColors = {
    sf::Color::Green,  sf::Color::Red,     sf::Color::Blue,
    sf::Color::Yellow, sf::Color::Magenta, sf::Color::Cyan};

// Metodo per disegnare i boid

void SimulationRenderer::drawBoids(const std::vector<Boid>& boids,
                                   sf::RenderWindow& window) {
  for (const auto& boid : boids) {
    int fid = boid.flockid;
    sf::Color colore =
        flockColors[static_cast<size_t>(fid) % flockColors.size()];
    boidShape.setFillColor(colore);

    const auto& pos = boid.position;
    const auto& vel = boid.speed;

    // Calcola l'angolo in radianti (atan2(y, x))
    double angleRad = std::atan2(vel.y, vel.x);

    // Converte i radianti in gradi
    double angleDeg = angleRad * (180. / pi);

    // Applica la correzione di +90 gradi.
    // CiÃ² compensa il fatto che atan2 misura da +X (destra)
    // mentre il nostro triangolo a 0 gradi punta verso -Y (alto).
    double rotation = angleDeg + 90.;

    // 1. Applica la position
    boidShape.setPosition(static_cast<float>(pos.x),
                          static_cast<float>(pos.y));
    // 2. Applica la rotazione
    boidShape.setRotation(static_cast<float>(rotation));

    window.draw(boidShape);
  }
};
}  // namespace b
// namespace b