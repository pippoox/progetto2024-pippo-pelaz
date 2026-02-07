#include <SFML/Graphics.hpp>
#include <iostream>
#include <random>

#include "boids.hpp"
#include "renderer.hpp"
#include "simulation_setup.hpp"

using namespace sf;
using namespace b;

int main() {   

  auto cfg = b::readConfig();
  auto sim = b::buildSimulation(cfg);

  sf::RenderWindow window(sf::VideoMode(static_cast<unsigned int>(sim.getWidth()),
                                        static_cast<unsigned int>(sim.getHeight())),
                          "Boids Simulation");

    b::SimulationRenderer renderer;

    while (window.isOpen()) {
      sf::Event event;
      while (window.pollEvent(event)) {
        if (event.type == sf::Event::Closed) window.close();
      }

      sim.updateBoids(cfg.dt);

      window.clear(sf::Color::Black);
      renderer.drawBoids(sim.getBoids(), window); 
      window.display();
    }

    return 0;
}