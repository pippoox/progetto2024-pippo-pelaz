#include <iostream>
#include <random>

#include "boids.hpp"

// Funzione per calcolare la distanza media e deviazione standard
void distanzaMediaeDevStd(const std::vector<b::Boid>& boids, double& media,
                          double& devStd) {
  size_t n = boids.size();
  std::vector<double> distanze;
  // Calcolo della distanza tra tutte le coppie di boid
  for (size_t i = 0; i < n; ++i) {
    for (size_t j = i + 1; j < n; ++j) {
      double d = (boids[i].posizione - boids[j].posizione).modulo();
      distanze.push_back(d);
    }
  }
  // Media delle distanze
  double somma = 0.0;
  for (double d : distanze) somma += d;
  media = somma / static_cast<double>(distanze.size());

  // Deviazione standard delle distanze
  double sommaQuad = 0.0;
  for (double d : distanze) sommaQuad += (d - media) * (d - media);
  devStd = std::sqrt(sommaQuad / static_cast<double>(distanze.size()));
}

// Funzione per calcolare la velocità media e deviazione standard
void velocitaMediaeDevStd(const std::vector<b::Boid>& boids, double& media,
                          double& devStd) {
  size_t n = boids.size();
  std::vector<double> moduli;
  // Calcola modulo (intensità) della velocità per ogni boid
  for (const auto& b : boids) {
    moduli.push_back(b.velocità.modulo());
  }
  // Media delle velocità
  double somma = 0.0;
  for (double v : moduli) somma += v;
  media = somma / static_cast<double>(n);

  // Deviazione standard delle velocità
  double sommaQuad = 0.0;
  for (double v : moduli) sommaQuad += (v - media) * (v - media);
  devStd = std::sqrt(sommaQuad / static_cast<double>(n));
}

int main() {
  size_t N;   // Numero di boid
  double d;   // Raggio visivo per allineamento e coesione
  double ds;  // Raggio visivo per separazione
  double s;   // Peso della separazione
  double a;   // Peso dell'allineamento
  double c;   // Peso della coesione
  double dt;  // Delta tempo
<<<<<<< HEAD

  std::cout << "Inserisci N, d, ds, s, a, c, dt:\n";
  std::cin >> N >> d >> ds >> s >> a >> c >> dt;

  // Crea stormo (insieme di boid)
  b::Allboids stormo(d, dt, 2.0);
=======
  const double width = 800;
  const double height = 600;

  std::cout << "Inserisci N, d, ds, s, a, c, dt:\n";

  std::cin >> N >> d >> ds >> s >> a >> c >> dt;
  if (std::cin.fail()) {
    /*std::cout << "Errore: non hai inserito un numero!\n";
    return 1;*/
    throw std::runtime_error("Errore: non hai inserito un numero!");
  }
  // Crea stormo (insieme di boid)
  b::Flock stormo(d, dt, 2.0);
>>>>>>> master

  // Generatori casuali per posizione e velocità iniziali
  std::mt19937 gen(std::random_device{}());
  std::uniform_real_distribution<double> posDist(-100, 100);
  std::uniform_real_distribution<double> velDist(-1, 1);

  // Inizializza boid con posizioni e velocità casuali
  for (size_t i = 0; i < N; ++i) {
    b::vettore pos{posDist(gen), posDist(gen)};
    b::vettore vel{velDist(gen), velDist(gen)};
    stormo.aggiungiBoid(b::Boid(pos, vel));
  }
  const int aggiornamenti = 100;
  for (int step = 0; step < aggiornamenti; ++step) {
<<<<<<< HEAD
    stormo.aggiornaBoids(d, ds, s, a, c);
=======
    stormo.aggiornaBoids(d, ds, s, a, c,width, height );
>>>>>>> master

    double distMedia, distDevStd;
    distanzaMediaeDevStd(stormo.getBoids(), distMedia, distDevStd);

    double velMedia, velDevStd;
    velocitaMediaeDevStd(stormo.getBoids(), velMedia, velDevStd);

    // Output: tempo, distanza media e dev std, velocità media e dev std
    std::cout << step * dt << " " << distMedia << " " << distDevStd << " "
              << velMedia << " " << velDevStd << "\n";
  }

  return 0;
}