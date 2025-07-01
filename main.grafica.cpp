#include <SFML/Graphics.hpp>
#include <random>
#include <iostream>

#include "boids.hpp"
#include "visualizzazione.hpp"

using namespace sf;
using namespace b;

// Funzione per inizializzare N boid con posizioni casuali e velocità casuali
void inizializzaBoids(std::vector<Boid>& boids, int N, double width,
                      double height) {
  std::random_device rd;   // genera un numero casuale imprevedibile dal sistema
  std::mt19937 gen(rd());  // motore di generazione (Mersenne Twister)
  std::uniform_real_distribution<double> posDistX(0.0, width);
  std::uniform_real_distribution<double> posDistY(0.0, height);
  std::uniform_real_distribution<double> velDist(-1.0, 1.0);

  for (int i = 0; i < N; ++i) {
    vettore pos(posDistX(gen),
                posDistY(gen));  // posizione casuale nell'area della finestra
    vettore vel(velDist(gen), velDist(gen));  // velocità casuale tra -1 e 1
    boids.emplace_back(pos, vel);             // aggiunta al vettore
  }
}

int main() {
  
  const double width = 800;
  const double height = 600;

  int N;
  double d, ds, s, a, c, dt;

  // Input utente
  std::cout << "Inserisci il numero di boid (N): ";
  std::cin >> N;

  std::cout << "Inserisci il raggio di visione (d): ";
  std::cin >> d;

  std::cout << "Inserisci il raggio di separazione (ds): ";
  std::cin >> ds;

  std::cout << "Inserisci il fattore di separazione (s): ";
  std::cin >> s;

  std::cout << "Inserisci il fattore di allineamento (a): ";
  std::cin >> a;

  std::cout << "Inserisci il fattore di coesione (c): ";
  std::cin >> c;

  std::cout << "Inserisci il passo temporale (dt): ";
  std::cin >> dt;

  sf::RenderWindow window(sf::VideoMode(static_cast<unsigned int>(width),
                                        static_cast<unsigned int>(height)),
                          "Boids Simulation");

  std::vector<b::Boid> boids;
  inizializzaBoids(boids, N, width, height);

  b::Visualizzazione visualizzazione(width, height);

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

    // Costruisci le posizioni da passare alla visualizzazione
    std::vector<sf::Vector2f> posizioni;
    for (const auto& boid : boids) {
      posizioni.emplace_back(static_cast<float>(boid.posizione.x),
                             static_cast<float>(boid.posizione.y));
    }

    visualizzazione.disegnaBoids(posizioni, window);
    window.display();
  }

  return 0;
}