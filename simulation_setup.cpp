#include "simulation_setup.hpp"

#include <algorithm>
#include <iostream>
#include <random>

#include "boids.hpp"

namespace b {

simConfig readConfig() {
  simConfig cfg;
  int input;

  std::cout << "Inserisci N(int):\n";
  std::cin >> input;
  if (std::cin.fail()) {
    throw std::runtime_error("Errore: non hai inserito il tipo corretto!");
  }
  if (input < 0) {
    throw std::runtime_error("Errore: inserisci un numero positivo");
  }
  cfg.N = static_cast<size_t>(input);

  std::cout << "Inserisci NF(int):\n";
  std::cin >> input;
  if (std::cin.fail()) {
    throw std::runtime_error("Errore: non hai inserito il tipo corretto!");
  }
  if (input < 0) {
    throw std::runtime_error("Errore: inserisci un numero positivo");
  }
  cfg.NF = static_cast<size_t>(input);
  if (cfg.N < cfg.NF) {
    throw std::runtime_error(
        "Errore: il numero di flock deve essere minore di quello dei boid "
        "(almeno 1 boid per flock)");
  }

  cfg.flocks.reserve(cfg.NF);
<<<<<<< HEAD
  for (int i = 0; i < cfg.NF; ++i) {
    const double maxSpeed = 6.0;
    const double minSpeed = 1.2;

=======
  for (size_t i = 0; i < static_cast<size_t>(cfg.NF); ++i) {
>>>>>>> 96b4b6d35e21c07ff05948a3f7f8f745228d7706
    double d;
    double ds;
    double s;
    double a;
    double c;
    std::cout << "Inserisci parametri di questo stormo d(double), ds(double), "
                 "s(double), a(double), c(double):\n";
    std::cin >> d >> ds >> s >> a >> c;
    if (std::cin.fail()) {
      throw std::runtime_error("Errore: non hai inserito il tipo corretto!");
    }
    cfg.flocks.emplace_back(i, d, ds, s, a, c, maxSpeed, minSpeed);
  }

  cfg.count.assign(cfg.NF, 0);
  size_t sum = 0;
  if (cfg.NF > 1) {
<<<<<<< HEAD
    for (int i = 0; i < cfg.NF - 1; ++i) {
      int remaining = cfg.NF - i - 1;
      int max = cfg.N - sum -
                remaining;  // Numero di boid massimo affinchè i flock rimanenti
=======
    for (size_t i = 0; i < static_cast<size_t>(cfg.NF) - 1; ++i) {
      size_t remaining = cfg.NF - i - 1; 
      size_t max = cfg.N - sum - remaining; // Numero di boid massimo affinchè i flock rimanenti
>>>>>>> 96b4b6d35e21c07ff05948a3f7f8f745228d7706
      // contengano almeno un boid.
      std::cout << "Inserisci boid nel flock" << i << "(min 1, max " << max
                << "):\n";
      std::cin >> cfg.count[i];
      if (std::cin.fail()) {
        throw std::runtime_error("Errore: non hai inserito il tipo corretto!");
      };
      if (cfg.count[i] < 1 || cfg.count[i] > max) {
        throw std::runtime_error(
            "Errore non hai inserito numero di boid compatibile");
      }
      sum += cfg.count[i];
    }
    cfg.count[cfg.NF - 1] =
        cfg.N - sum;  // L'ultimo flock è riempito automaticamente con
    // i boid non ancora inseriti.
  } else {
    cfg.count[0] = cfg.N;
  }
  return cfg;
}

BoidSimulation buildSimulation(const simConfig& cfg, const double& width,
                               const double& height) {
  BoidSimulation sim(cfg.flocks, width, height);

  std::mt19937 gen(std::random_device{}());
  std::uniform_real_distribution<double> posxDist(0.0, width);
  std::uniform_real_distribution<double> posyDist(0.0, height);
  std::uniform_real_distribution<double> angleDist(0.0, 2 * M_PI);

  std::vector<int>
      labels;  // Vettore che contiene gli int che identificano a quale flock
  // appartiene un boid
  labels.reserve(cfg.N);
  for (size_t fid = 0; fid < cfg.NF; ++fid) {
    for (size_t k = 0; k < cfg.count[fid]; ++k) {
      labels.push_back(static_cast<int>(fid));
    };
  }

  std::shuffle(labels.begin(), labels.end(),
               gen);  // Rimescola in maniera casuale i flockid
  // dentro labels

  for (size_t i = 0; i < cfg.N; ++i) {
    b::Vector pos{posxDist(gen), posyDist(gen)};

    int fid = labels[i];
    const auto& f = cfg.flocks[fid];
    std::uniform_real_distribution<double> speedDist(f.minSpeed, f.maxSpeed);

    double ang = angleDist(gen);
    double spd = speedDist(gen);
    Vector speed{std::cos(ang) * spd, std::sin(ang) * spd};

    sim.addBoids(b::Boid(pos, speed, fid));
  }
  return sim;
}

}  // namespace b