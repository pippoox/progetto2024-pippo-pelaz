#include <SFML/Graphics.hpp>
#include <random>
#include <iostream>

#include "boids.hpp"
#include "visualizzazione.hpp"

using namespace sf;
using namespace b;

// Funzione per inizializzare N boid con posizioni casuali e velocità casuali
void inizializzaBoids(Flock& flock, int N, double width,
                      double height) {
  std::random_device rd;   // genera un numero casuale imprevedibile dal sistema
  std::mt19937 gen(rd());  // motore di generazione (Mersenne Twister)
  std::uniform_real_distribution<double> posX(0.0, width);
  std::uniform_real_distribution<double> posY(0.0, height);
  std::uniform_real_distribution<double> velX(-1.0, 1.0);
  std::uniform_real_distribution<double> velY(-1.0, 1.0);

  for (int i = 0; i < N; ++i) {
    vettore pos(posX(gen),
                posY(gen));  // posizione casuale nell'area della finestra
    vettore vel(velX(gen), velY(gen));  // velocità casuale tra -1 e 1
    flock.aggiungiBoid(Boid(pos, vel));             // aggiunta al vettore
  }
}

int main() {
  

  int N;
  double d, ds, s, a, c, dt;
  const double width = 800;
  const double height = 600;

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

  if (std::cin.fail()) {
    std::cout << "Errore: non hai inserito un numero!\n";
  }

  sf::RenderWindow window(sf::VideoMode(static_cast<unsigned int>(width),
                                        static_cast<unsigned int>(height)),
                          "Boids Simulation");

  Flock flock(d, dt, 2.0);
  inizializzaBoids(flock, N, width, height);

  b::Visualizzazione visualizzazione(width, height);

  while (window.isOpen()) {
    sf::Event event;
    while (window.pollEvent(event)) {
      if (event.type == sf::Event::Closed)
        window.close();
    }

    flock.aggiornaBoids(d, ds, s, a, c, width, height);

    window.clear(sf::Color::Black);

    // Costruisci le posizioni da passare alla visualizzazione
    visualizzazione.disegnaBoids(flock.getBoids(), window);
    window.display();
  }

  return 0;
}