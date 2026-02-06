#include <SFML/Graphics.hpp>
#include <iostream>
#include <random>

#include "boids.hpp"
#include "visualizzazione.hpp"
#include "simulation_setup.hpp"

using namespace sf;
using namespace b;

int main() {  
  const double width = 800;
  const double height = 600; 
  const double dt = 0.5;

  auto cfg = b::readConfig();
  auto sim = b::buildSimulation(cfg, width, height);

  sf::RenderWindow window(sf::VideoMode(static_cast<unsigned int>(width),
                                        static_cast<unsigned int>(height)),
                          "Boids Simulation");

    b::SimulationRenderer renderer;

    while (window.isOpen()) {
      sf::Event event;
      while (window.pollEvent(event)) {
        if (event.type == sf::Event::Closed) window.close();
      }

      sim.updateBoids(dt);

      window.clear(sf::Color::Black);
      renderer.drawBoids(sim.getBoids(), window); // <-- cambia firma
      window.display();
    }

    return 0;
}