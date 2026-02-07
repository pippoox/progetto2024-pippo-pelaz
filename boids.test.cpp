#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "boids.hpp"

#include "doctest.h"
#include "simulation_setup.hpp"
#include "stats.hpp"
#include "visualizzazione.hpp"

TEST_CASE("Testing b::Vector implementation") {
  SUBCASE("Testing operator+") {
    b::Vector v1 = {1.0, 2.0};
    b::Vector v2 = {2.0, 3.0};
    b::Vector result = v1 + v2;
    CHECK(result.x == doctest::Approx(3.0));
    CHECK(result.y == doctest::Approx(5.0));
  }

  SUBCASE("Testing operator-") {
    b::Vector v1 = {5.0, 6.0};
    b::Vector v2 = {1.0, 2.0};
    b::Vector result = v1 - v2;
    CHECK(result.x == doctest::Approx(4.0));
    CHECK(result.y == doctest::Approx(4.0));
  }

  SUBCASE("Testing operator*") {
    b::Vector v = {2.0, 3.0};
    double scalar = 2.0;
    b::Vector result = v * scalar;
    CHECK(result.x == doctest::Approx(4.0));
    CHECK(result.y == doctest::Approx(6.0));
  }

  SUBCASE("Testing length") {
    b::Vector v = {3.0, 4.0};
    CHECK(v.length() == doctest::Approx(5.0));
  }

  SUBCASE("Testing length v={0.0,0.0}") {
    b::Vector v = {0.0, 0.0};
    CHECK(v.length() == doctest::Approx(0.0));
  }

  SUBCASE("Testing norm") {
    b::Vector v = {3.0, 4.0};
    b::Vector normalized_v = v.norm();
    CHECK(normalized_v.length() == doctest::Approx(1.0));
    CHECK(normalized_v.x == doctest::Approx(0.6));
    CHECK(normalized_v.y == doctest::Approx(0.8));
  }

  SUBCASE("Testing norm v={0.0,0.0}") {
    b::Vector v = {0.0, 0.0};
    b::Vector normalized_v = v.norm();
    CHECK(normalized_v.x == doctest::Approx(0.0));
    CHECK(normalized_v.y == doctest::Approx(0.0));
  }
}

TEST_CASE("Testing BoidSimulation implementation") {
  std::vector<b::Flock> flocks = {{0, 10.0, 2.0, 1.0, 1.0, 1.0, 1.2, 6.0}};
  b::BoidSimulation sim(flocks, 100.0, 100.0);

  SUBCASE("Testing getNeighbors same flock") {
    sim.addBoids(b::Boid({0.0, 0.0}, {0.0, 0.0}, 0));
    sim.addBoids(b::Boid({5.0, 0.0}, {0.0, 0.0}, 0));
    sim.addBoids(b::Boid({15.0, 0.0}, {0.0, 0.0}, 0));

    std::vector<size_t> neighbors = sim.getNeighbors(0, 10.0);

    CHECK(neighbors.size() == 1);
    CHECK(neighbors[0] == 1);
  }

  SUBCASE("Testing getNeighborsDS") {
    sim.addBoids(b::Boid({0.0, 0.0}, {0.0, 0.0}, 0));
    sim.addBoids(b::Boid({1.0, 0.0}, {0.0, 0.0}, 1));

    std::vector<size_t> neighborsDS = sim.getNeighborsDS(0, 2.0);
    CHECK(neighborsDS.size() == 1);
    CHECK(neighborsDS[0] == 1);
  }

  SUBCASE("Testing separation") {
    sim.addBoids(b::Boid({0.0, 0.0}, {0.0, 0.0}, 0));
    sim.addBoids(b::Boid({1.0, 0.0}, {0.0, 0.0}, 0));

    std::vector<size_t> neighborsDS = {1};

    b::Vector sep = sim.separation(0, neighborsDS, 1.0);

    CHECK(sep.x < 0);
    CHECK(sep.y == doctest::Approx(0.0));
  }

  SUBCASE("Testing alignment") {
    sim.addBoids(b::Boid({0.0, 0.0}, {1.0, 0.0}, 0));
    sim.addBoids(b::Boid({1.0, 0.0}, {3.0, 0.0}, 0));

    std::vector<size_t> neighbors = {1};

    b::Vector align = sim.alignment(0, neighbors, 0.5);

    CHECK(align.x == doctest::Approx(1.0));
  }

  SUBCASE("Testing cohesion") {
    sim.addBoids(b::Boid({0.0, 0.0}, {0.0, 0.0}, 0));
    sim.addBoids(b::Boid({10.0, 10.0}, {0.0, 0.0}, 0));

    std::vector<size_t> neighbors = {1};

    b::Vector coh = sim.cohesion(0, neighbors, 0.1);

    CHECK(coh.x == doctest::Approx(1.0));
    CHECK(coh.y == doctest::Approx(1.0));
  }
}
TEST_CASE("Testing BoidSimulation Integration (updateBoids)") {
  SUBCASE("Simple linear movement without neighbors") {
    b::Flock f(0, 10.0, 2.0, 0.0, 0.0, 0.0, 0.0, 100.0);
    b::BoidSimulation sim({f}, 100.0, 100.0);

    sim.addBoids(b::Boid({50.0, 50.0}, {2.0, 0.0}, 0));
    double dt = 1.0;
    sim.updateBoids(dt);

    const std::vector<b::Boid>& boids = sim.getBoids();

    CHECK(boids[0].position.x == doctest::Approx(52.0));
    CHECK(boids[0].position.y == doctest::Approx(50.0));
  }

  SUBCASE("Maximum speed clamping logic") {
    b::Flock f(0, 10.0, 2.0, 0.0, 0.0, 0.0, 0.0, 6.0);
    b::BoidSimulation sim({f}, 100.0, 100.0);

    b::Boid b1({50.0, 50.0}, {5.9, 0.0}, 0);
    b::Boid b2({51.0, 50.0}, {6.0, 0.0}, 0);

    sim.addBoids(b1);
    sim.addBoids(b2);

    double dt = 1.0;
    sim.updateBoids(dt);

    const std::vector<b::Boid>& boids = sim.getBoids();

    CHECK(boids[0].speed.length() <= doctest::Approx(f.maxSpeed));
  }

  SUBCASE("Minimum speed clamping logic (per-flock minSpeed)") {
    b::Flock f(0, 10.0, 2.0, 0.0, 0.0, 0.0, 1.2, 6.0);
    b::BoidSimulation sim({f}, 100.0, 100.0);

    b::Boid b1({50.0, 50.0}, {5.9, 0.0}, 0);

    sim.addBoids(b1);

    double dt = 1.0;
    sim.updateBoids(1.0);

    const auto& boids = sim.getBoids();
    CHECK(boids[0].speed.length() == doctest::Approx(f.minSpeed));
  }

  SUBCASE("Toroidal screen wrapping (Edge handling)") {
    b::Flock f(0, 10.0, 2.0, 0.0, 0.0, 0.0, 0.0, 100.0);
    b::BoidSimulation sim({f}, 100.0, 100.0);

    b::Boid b1({99.5, 50.0}, {1.0, 0.0}, 0);

    sim.addBoids(b1);

    double dt = 1.0;
    sim.updateBoids(dt);

    const std::vector<b::Boid>& boids = sim.getBoids();

    CHECK(boids[0].position.x < 1.0);
    CHECK(boids[0].position.x >= 0.0);
  }
}
TEST_CASE("Testing Simulation Setup (buildSimulation)") {
  b::simConfig cfg;
  cfg.N = 10;
  cfg.NF = 2;
  cfg.count = {7, 3};
  cfg.width = 100.0;
  cfg.height = 100.0;
  cfg.dt = 0.5;

  cfg.flocks = {{0, 10.0, 2.0, 1.0, 1.0, 1.0, 1.2, 6.0},
                {1, 10.0, 2.0, 1.0, 1.0, 1.0, 1.2, 6.0}};

  b::BoidSimulation sim = b::buildSimulation(cfg);
  const std::vector<b::Boid>& boids = sim.getBoids();

  SUBCASE("Correct total number of boids") { CHECK(boids.size() == 10); }

  SUBCASE("Correct flock distribution") {
    int count0 = 0;
    int count1 = 0;
    for (const auto& boid : boids) {
      if (boid.flockid == 0) count0++;
      if (boid.flockid == 1) count1++;
    }
    CHECK(count0 == 7);
    CHECK(count1 == 3);
  }

  SUBCASE("Boids are within screen boundaries") {
    for (const auto& boid : boids) {
      CHECK(boid.position.x >= 0.0);
      CHECK(boid.position.x <= cfg.width);
      CHECK(boid.position.y >= 0.0);
      CHECK(boid.position.y <= cfg.height);
    }
  }
}

TEST_CASE("Stats::computeMeanSpeed"){
    SUBCASE("Empty vector -> mean=0, stdDev=0"){std::vector<b::Boid> boids;
auto s = b::computeMeanSpeed(boids);
CHECK(s.mean == doctest::Approx(0.0));
CHECK(s.stdDev == doctest::Approx(0.0));
}

SUBCASE("Single boid -> stdDev=0") {
  std::vector<b::Boid> boids = {b::Boid({0.0, 0.0}, {3.0, 4.0}, 0)};
  auto s = b::computeMeanSpeed(boids);
  CHECK(s.mean == doctest::Approx(5.0));
  CHECK(s.stdDev == doctest::Approx(0.0));
}

SUBCASE("Two boids with speeds 0 and 2 -> mean=1, stdDev=1") {
  std::vector<b::Boid> boids = {b::Boid({0.0, 0.0}, {0.0, 0.0}, 0),
                                b::Boid({0.0, 0.0}, {2.0, 0.0}, 0)};
  auto s = b::computeMeanSpeed(boids);
  CHECK(s.mean == doctest::Approx(1.0));
  CHECK(s.stdDev == doctest::Approx(1.0));
}

SUBCASE("Three boids speeds 1,2,3 -> mean=2, stdDev=sqrt(2/3)") {
  std::vector<b::Boid> boids = {
      b::Boid({0.0, 0.0}, {1.0, 0.0}, 0),
      b::Boid({0.0, 0.0}, {2.0, 0.0}, 0),
      b::Boid({0.0, 0.0}, {3.0, 0.0}, 0),
  };
  auto s = b::computeMeanSpeed(boids);
  CHECK(s.mean == doctest::Approx(2.0));
  CHECK(s.stdDev == doctest::Approx(std::sqrt(2.0 / 3.0)));
}
}
;

TEST_CASE("Stats::computeMeanDistance") {
  SUBCASE("Empty vector -> mean=0, stdDev=0") {
    std::vector<b::Boid> boids;
    auto s = b::computeMeanDistance(boids);
    CHECK(s.mean == doctest::Approx(0.0));
    CHECK(s.stdDev == doctest::Approx(0.0));
  }

  SUBCASE("Single boid -> mean=0, stdDev=0 (no pairs)") {
    std::vector<b::Boid> boids = {b::Boid({0.0, 0.0}, {0.0, 0.0}, 0)};
    auto s = b::computeMeanDistance(boids);
    CHECK(s.mean == doctest::Approx(0.0));
    CHECK(s.stdDev == doctest::Approx(0.0));
  }

  SUBCASE("Two boids at distance 5 -> mean=5, stdDev=0") {
    std::vector<b::Boid> boids = {b::Boid({0.0, 0.0}, {0.0, 0.0}, 0),
                                  b::Boid({3.0, 4.0}, {0.0, 0.0}, 0)};
    auto s = b::computeMeanDistance(boids);
    CHECK(s.mean == doctest::Approx(5.0));
    CHECK(s.stdDev == doctest::Approx(0.0));
  }

  SUBCASE(
      "Three boids on a line: distances 1,2,1 -> mean=4/3, stdDev=sqrt(2/9)") {
    std::vector<b::Boid> boids = {
        b::Boid({0.0, 0.0}, {0.0, 0.0}, 0),
        b::Boid({1.0, 0.0}, {0.0, 0.0}, 0),
        b::Boid({2.0, 0.0}, {0.0, 0.0}, 0),
    };
    auto s = b::computeMeanDistance(boids);
    CHECK(s.mean == doctest::Approx(4.0 / 3.0));
    CHECK(s.stdDev == doctest::Approx(std::sqrt(2.0 / 9.0)));
  }

  SUBCASE("Square points (0,0),(1,0),(0,1),(1,1)") {
    std::vector<b::Boid> boids = {
        b::Boid({0.0, 0.0}, {0.0, 0.0}, 0),
        b::Boid({1.0, 0.0}, {0.0, 0.0}, 0),
        b::Boid({0.0, 1.0}, {0.0, 0.0}, 0),
        b::Boid({1.0, 1.0}, {0.0, 0.0}, 0),
    };
    auto s = b::computeMeanDistance(boids);

    const double mean = (4.0 + 2.0 * std::sqrt(2.0)) / 6.0;
    const double e2 = 4.0 / 3.0; 
    const double var = e2 - mean * mean;
    const double stddev = std::sqrt(var);

    CHECK(s.mean == doctest::Approx(mean));
    CHECK(s.stdDev == doctest::Approx(stddev));
  }
}

