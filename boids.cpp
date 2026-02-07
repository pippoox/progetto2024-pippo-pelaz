#include <cmath>
#include <vector>

#include "boids.hpp"

namespace b {

Vector Vector::operator+(Vector const& v) const { return {x + v.x, y + v.y}; }
Vector Vector::operator-(Vector const& v) const { return {x - v.x, y - v.y}; }
Vector Vector::operator*(double a) const { return {x * a, y * a}; }
double Vector::length() const { return std::sqrt(x * x + y * y); }
Vector Vector::norm() const {
  double l = length();
  if (l > 0)
    return {x / l, y / l};
  else
    return {0.0, 0.0};
}

// Metodi della classe BoidSimulation:
void BoidSimulation::addBoids(Boid const& boid) { boids.push_back(boid); };

const std::vector<Boid>& BoidSimulation::getBoids() const { return boids; }

const double& BoidSimulation::getWidth() const { return width; }

const double& BoidSimulation::getHeight() const { return height; }

std::vector<size_t> BoidSimulation::getNeighbors(size_t i, double d) const {
  std::vector<size_t> neighbors;
  const Boid& boid = boids[i];
  for (size_t j = 0; j < boids.size(); ++j) {
    if (j != i) {
      if (boids[j].flockid == boid.flockid) {
        if ((boids[j].position - boid.position).length() < d) {
          neighbors.push_back(j);
        }
      };
    };
  };
  return neighbors;
}

std::vector<size_t> BoidSimulation::getNeighborsDS(size_t i,
                                                   double ds) const {
  std::vector<size_t> neighborsDS;
  const Boid& boid = boids[i];
  for (size_t j = 0; j < boids.size(); ++j) {
    if (j != i) {
      double distance = (boids[j].position - boid.position).length();
      if (distance < ds) {
        neighborsDS.push_back(j);
      }
    }
  }
  return neighborsDS;
}

// Separation: avoids crowding and collisions with nearby neighbors.
// Steers away from neighbors that are within the 'ds' (separation distance) radius.
Vector BoidSimulation::separation(size_t i,
                                  const std::vector<size_t>& neighborsDS,
                                  double s) {
  if (neighborsDS.empty()) return {0.0, 0.0};
  Vector sum{0.0, 0.0};
  for (size_t j : neighborsDS) {
    Vector diff = boids[i].position - boids[j].position;
    double dist = diff.length();
    if (dist > 0) {
      sum = sum + (diff.norm() * (1.0 / dist));
    }
  }
  return sum * s;
}

// Alignment: steers boids towards the average heading of local neighbors.
// Calculates the average speed of neighbors and adjusts the boid's 
// speed to match the group's direction and speed.
Vector BoidSimulation::alignment(size_t i,
                                 const std::vector<size_t>& getNeighbors,
                                 double a) {
  if (getNeighbors.empty()) return {0.0, 0.0};
  Vector speedSum{0.0, 0.0};
  for (size_t j : getNeighbors) {
    speedSum = speedSum + boids[j].speed;
  }
  Vector meanSpeed =
      speedSum * (1.0 / static_cast<double>(getNeighbors.size()));
  Vector diff = meanSpeed - boids[i].speed;
  return diff * a;
}

// Cohesion: steers boids toward the average position (center of mass) of local neighbors.
// This keeps the flock together by preventing individual boids from drifting away.
Vector BoidSimulation::cohesion(size_t i,
                                const std::vector<size_t>& getNeighbors,
                                double c) {
  if (getNeighbors.empty()) {
    return {0.0, 0.0};
  }
  Vector positionSum{0.0, 0.0};
  for (size_t j : getNeighbors) {
    positionSum = positionSum + boids[j].position;
  }
  Vector centerOfMass =
      positionSum * (1.0 / static_cast<double>(getNeighbors.size()));
  Vector centerDirection = centerOfMass - boids[i].position;
  return centerDirection * c;
}

// Main update loop for all boids in the simulation.
// Computes all three steering forces, applies velocity limits (clamping),
// updates positions, and handles screen-wrap boundaries.
void BoidSimulation::updateBoids(double dt) {
  std::vector<Vector> newSpeed(boids.size());
  std::vector<Vector> newPosition(boids.size());

  for (size_t i = 0; i < boids.size(); ++i) {
    const Boid& boid = boids[i];
    const Flock& f = flocks[static_cast<std::size_t>(boid.flockid)];
    std::vector<size_t> neighbors = getNeighbors(i, f.d);
    std::vector<size_t> neighborsDS = getNeighborsDS(i, f.ds);

    Vector v1 = separation(i, neighborsDS, f.s);
    Vector v2 = alignment(i, neighbors, f.a);
    Vector v3 = cohesion(i, neighbors, f.c);

    Vector vel = boid.speed + v1 + v2 + v3;
    double len = vel.length();
    if (len > 0.0) {
      if (len > f.maxSpeed)
        vel = vel * (f.maxSpeed / len);
      else if (len < f.minSpeed)
        vel = vel * (f.minSpeed / len);
    } else {
      vel = {f.minSpeed, 0.0};
    }

    Vector pos = boid.position + vel * dt;

    // Toroidal world logic: boids that exit one side of the screen reappear on the opposite side.
    if (pos.x < 0) pos.x = width;
    if (pos.x > width) pos.x = 0;
    if (pos.y < 0) pos.y = height;
    if (pos.y > height) pos.y = 0;

    newSpeed[i] = vel;
    newPosition[i] = pos;
  }
  for (size_t i = 0; i < boids.size(); ++i) {
    boids[i].speed = newSpeed[i];
    boids[i].position = newPosition[i];
  }
}
}

;  // namespace b
