
#include "renderer.hpp"

#include <cmath>

#include "boids.hpp"

namespace b {

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

// Renders each boid as a geometric shape (a triangle).
// The orientation of the shape is updated to match the boid's speed vector.
void SimulationRenderer::drawBoids(const std::vector<Boid>& boids,
                                   sf::RenderWindow& window) {
  for (const auto& boid : boids) {
    int fid = boid.flockid;
    sf::Color colore =
        flockColors[static_cast<size_t>(fid) % flockColors.size()];
    boidShape.setFillColor(colore);

    const auto& pos = boid.position;
    const auto& spd = boid.speed;

    // Calculates the rotation angle based on the velocity vector to point
    // the boid in its direction of travel.
    double angleRad = std::atan2(spd.y, spd.x);

    double angleDeg = angleRad * (180. / pi);

    double rotation = angleDeg + 90.;

    boidShape.setPosition(static_cast<float>(pos.x), static_cast<float>(pos.y));

    boidShape.setRotation(static_cast<float>(rotation));

    window.draw(boidShape);
  }
};
}  // namespace b