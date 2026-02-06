#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "boids.hpp"
#include "simulation_setup.hpp"
#include "visualizzazione.hpp"

#include "doctest.h"

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
    // Uso length() con la 'h' finale come nel tuo boids.cpp
    CHECK(v.length() == doctest::Approx(5.0));
  }

  SUBCASE("Testing length v={0.0,0.0}") {
    b::Vector v = {0.0, 0.0};
    CHECK(v.length() == doctest::Approx(0.0));
  }

  SUBCASE("Testing norm") {
    b::Vector v = {3.0, 4.0};
    b::Vector normalized_v = v.norm();
    // Verifichiamo che il modulo del vettore normalizzato sia 1
    CHECK(normalized_v.length() == doctest::Approx(1.0));
    CHECK(normalized_v.x == doctest::Approx(0.6));
    CHECK(normalized_v.y == doctest::Approx(0.8));
  }

  SUBCASE("Testing norm v={0.0,0.0}") {
    b::Vector v = {0.0, 0.0};
    b::Vector normalized_v = v.norm();
    // Il tuo codice in boids.cpp restituisce {0.0, 0.0} in questo caso
    CHECK(normalized_v.x == doctest::Approx(0.0));
    CHECK(normalized_v.y == doctest::Approx(0.0));
  }
}

TEST_CASE("Testing BoidSimulation implementation") {
  std::vector<b::Flock> flocks = {{0, 10.0, 2.0, 1.0, 1.0, 1.0}};
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
    sim.addBoids(b::Boid({1.0, 0.0}, {0.0, 0.0},
                         1));  

    
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
    sim.addBoids(
        b::Boid({1.0, 0.0}, {3.0, 0.0}, 0));  

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
  // Common setup: 100.0 x 100.0 area, single flock definition
  // Parameters: id=0, visual_range=10.0, separation_range=2.0, weights(s=1, a=1, c=1)
  std::vector<b::Flock> flocks = {{0, 10.0, 2.0, 1.0, 1.0, 1.0}};
  b::BoidSimulation sim(flocks, 100.0, 100.0);

  SUBCASE("Simple linear movement without neighbors") {
    // Adding a single isolated boid at the center
    b::Boid b1({50.0, 50.0}, {2.0, 0.0}, 0);
    sim.addBoids(b1);

    double dt = 1.0;
    sim.updateBoids(dt);

    const std::vector<b::Boid>& boids = sim.getBoids();
    
    // With no neighbors, speed should remain constant.
    // New Position = Old Position + (Velocity * dt) -> 50.0 + (2.0 * 1.0) = 52.0
    CHECK(boids[0].position.x == doctest::Approx(52.0));
    CHECK(boids[0].position.y == doctest::Approx(50.0));
  }

  SUBCASE("Maximum speed clamping logic") {
    // Initialize a boid near the global speed limit (b::maxVel = 6.0)
    b::Boid b1({50.0, 50.0}, {5.9, 0.0}, 0);
    
    // Add a neighbor that forces it to accelerate (e.g., via alignment)
    b::Boid b2({51.0, 50.0}, {6.0, 0.0}, 0);
    
    sim.addBoids(b1);
    sim.addBoids(b2);

    double dt = 1.0;
    sim.updateBoids(dt);

    const std::vector<b::Boid>& boids = sim.getBoids();
    
    // Ensure the resulting speed does not exceed the defined maxVel
    CHECK(boids[0].speed.length() <= doctest::Approx(b::maxVel));
  }

  SUBCASE("Toroidal screen wrapping (Edge handling)") {
    // Place a boid at the far right edge (width = 100.0) moving right
    b::Boid b1({99.5, 50.0}, {1.0, 0.0}, 0);
    sim.addBoids(b1);

    double dt = 1.0;
    sim.updateBoids(dt);

    const std::vector<b::Boid>& boids = sim.getBoids();
    
    // The boid should wrap around and appear on the left side (near 0.5)
    CHECK(boids[0].position.x < 1.0);
    CHECK(boids[0].position.x >= 0.0);
  }
}
TEST_CASE("Testing Simulation Setup (buildSimulation)") {
  // Create a dummy config
  b::simConfig cfg;
  cfg.N = 10;           // Total boids
  cfg.NF = 2;          // Two flocks
  cfg.count = {7, 3};  // 7 boids in flock 0, 3 in flock 1
  
  // Define flock parameters
  cfg.flocks = {
      {0, 10.0, 2.0, 1.0, 1.0, 1.0},
      {1, 10.0, 2.0, 1.0, 1.0, 1.0}
  };

  double width = 800.0;
  double height = 600.0;

  // Run the builder
  b::BoidSimulation sim = b::buildSimulation(cfg, width, height);
  const std::vector<b::Boid>& boids = sim.getBoids();

  SUBCASE("Correct total number of boids") {
    CHECK(boids.size() == 10);
  }

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
      CHECK(boid.position.x <= width);
      CHECK(boid.position.y >= 0.0);
      CHECK(boid.position.y <= height);
    }
  }
}
