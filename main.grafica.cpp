#include <SFML/Graphics.hpp>
#include "boids.hpp"        
#include "visualizzazione.hpp"  

using namespace sf;
using namespace b; 


void inizializzaBoids(std::vector<Boid>& boids, int N, double width,double height) {
    for (int i = 0; i < N; ++i) {
        double x = static_cast<double>(rand() % static_cast<int>(width));
        double y = static_cast<double>(rand() % static_cast<int>(height));
        vettore pos(x, y);
        vettore vel((rand() % 200 - 100) / 100.f, (rand() % 200 - 100) / 100.f); // random tra -1 e 1
        boids.emplace_back(pos, vel);  
    }
}

int main() {
    std::srand(std::time(nullptr));

    const int N = 100;
    const double width = 800;
    const double height = 600;

    const double d = 100.0;
    const double ds = 50.0;
    const double s = 10.0;
    const double a = 0.01;
    const double c = 0.01;
    const double dt = 1.0;

    sf::RenderWindow window(sf::VideoMode((unsigned int)width, (unsigned int)height), "Boids Simulation");

    std::vector<b::Boid> boids;
    inizializzaBoids(boids, N, width, height);

    while (window.isOpen()) {
        sf::Event event;
        while (window.pollEvent(event)) {
            if (event.type == sf::Event::Closed)
                window.close();
        }

        
        for (auto& boid : boids) {
            boid.aggiorna(boids, width, height, d, ds, s, a, c, dt);
        }

        
        window.clear(sf::Color::Black);

        for (const auto& boid : boids) {
            sf::CircleShape shape(4);
            shape.setFillColor(sf::Color::White);
            shape.setPosition(boid.posizione.x, boid.posizione.y);
            window.draw(shape);
        }

        window.display();
    }

    return 0;
}