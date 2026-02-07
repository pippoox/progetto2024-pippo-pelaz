
#include <algorithm>
#include <iostream>
#include <random>

#include "boids.hpp"
#include "simulation_setup.hpp"

namespace b {

// Parses simulation parameters from a configuration file or standard input.
// Includes physics constants like delta time (dt), interaction radii, 
// and steering behavior weights (s, a, c).
simConfig readConfig() {
  simConfig cfg;
  cfg.width = 800;
  cfg.height = 600;
  cfg.dt = 0.5;
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
  for (size_t i = 0; i < static_cast<size_t>(cfg.NF); ++i) {
    const double minSpeed = 1.2;
    const double maxSpeed = 6.0;

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
    cfg.flocks.emplace_back(i, d, ds, s, a, c, minSpeed, maxSpeed);
  }

  cfg.count.assign(cfg.NF, 0);
  size_t sum = 0;
  if (cfg.NF > 1) {
    for (size_t i = 0; i < static_cast<size_t>(cfg.NF) - 1; ++i) {
      size_t remaining = cfg.NF - i - 1;
      size_t max =
          cfg.N - sum -
          remaining;  // Maximum number of boids to ensure that each remaining flock contains at least one boid.
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
        cfg.N - sum;  // The last flock is automatically populated with the remaining boids.
  } else {
    cfg.count[0] = cfg.N;
  }
  return cfg;
}

// Factory function to create a BoidSimulation object.
// Sets up the initial state by distributing boids across the coordinate system
// with randomized positions and speeds.
BoidSimulation buildSimulation(const simConfig& cfg) {
  BoidSimulation sim(cfg.flocks, cfg.width, cfg.height);

  std::mt19937 gen(std::random_device{}());
  std::uniform_real_distribution<double> posxDist(0.0, cfg.width);
  std::uniform_real_distribution<double> posyDist(0.0, cfg.height);
  std::uniform_real_distribution<double> angleDist(0.0, 2 * M_PI);

  std::vector<int>
      labels; // Vector containing the IDs that identify which flock each boid belongs to. 
  labels.reserve(cfg.N);
  for (size_t fid = 0; fid < cfg.NF; ++fid) {
    for (size_t k = 0; k < cfg.count[fid]; ++k) {
      labels.push_back(static_cast<int>(fid));
    };
  }

  std::shuffle(labels.begin(), labels.end(),
               gen); // Randomly shuffles the flock IDs within labels.

  for (size_t i = 0; i < cfg.N; ++i) {
    b::Vector pos{posxDist(gen), posyDist(gen)};

    const auto fid = static_cast<std::size_t>(labels[i]);
    const auto& f = cfg.flocks[fid];
    std::uniform_real_distribution<double> speedDist(f.minSpeed, f.maxSpeed);

    double ang = angleDist(gen);
    double spd = speedDist(gen);
    Vector speed{std::cos(ang) * spd, std::sin(ang) * spd};

    sim.addBoids(b::Boid(pos, speed, static_cast<int>(fid)));
  }
  return sim;
}

}  // namespace b