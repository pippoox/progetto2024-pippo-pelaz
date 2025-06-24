#include <iostream>
#include <random>

#include "boids.hpp"
#include "visualizzazione.hpp"

void distanzaMediaeDevStd(const std::vector<b::Boid>& boids, double& media,
                          double& devStd) {
  size_t n = boids.size();
  std::vector<double> distanze;
  for (size_t i = 0; i < n; ++i) {
    for (size_t j = i + 1; j < n; ++j) {
      double d = (boids[i].posizione - boids[j].posizione).modulo();
      distanze.push_back(d);
    }
  }
  double somma = 0.0;
  for (double d : distanze) somma += d;
  media = somma / static_cast<double>(distanze.size());

  double sommaQuad = 0.0;
  for (double d : distanze) sommaQuad += (d - media) * (d - media);
  devStd = std::sqrt(sommaQuad / static_cast<double>(distanze.size()));
}

void velocitaMediaeDevStd(const std::vector<b::Boid>& boids, double& media,
                          double& devStd) {
  size_t n = boids.size();
  std::vector<double> moduli;
  for (const auto& b : boids) {
    moduli.push_back(b.velocità.modulo());
  }
  double somma = 0.0;
  for (double v : moduli) somma += v;
  media = somma / static_cast<double>(n);

  double sommaQuad = 0.0;
  for (double v : moduli) sommaQuad += (v - media) * (v - media);
  devStd = std::sqrt(sommaQuad / static_cast<double>(n));
}

int main() {
  size_t N;
  double d;
  double ds;
  double s;
  double a;
  double c;
  double dt;

  std::cout << "Inserisci N, d, ds, s, a, c, dt:\n";
  std::cin >> N >> d >> ds >> s >> a >> c >> dt;

  b::Allboids stormo(d, dt, 2.0);

  std::mt19937 gen(std::random_device{}());
  std::uniform_real_distribution<double> posDist(-100, 100);
  std::uniform_real_distribution<double> velDist(-1, 1);

  for (size_t i = 0; i < N; ++i) {
    b::vettore pos{posDist(gen), posDist(gen)};
    b::vettore vel{velDist(gen), velDist(gen)};
    stormo.aggiungiBoid(b::Boid(pos, vel));
  }
  const int aggiornamenti = 100;
  for (int step = 0; step < aggiornamenti; ++step) {
    stormo.aggiornaBoids(d, ds, s, a, c);

    double distMedia, distDevStd;
    distanzaMediaeDevStd(stormo.getBoids(), distMedia, distDevStd);

    double velMedia, velDevStd;
    velocitaMediaeDevStd(stormo.getBoids(), velMedia, velDevStd);

    std::cout << step * dt << " " << distMedia << " " << distDevStd << " "
              << velMedia << " " << velDevStd << "\n";
  }

  return 0;
}