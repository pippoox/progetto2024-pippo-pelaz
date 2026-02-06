#include "boids.hpp"

namespace b {

Vector Vector::operator+(Vector const& v) const { return {x + v.x, y + v.y}; }
Vector Vector::operator-(Vector const& v) const { return {x - v.x, y - v.y}; }
Vector Vector::operator*(double a) const { return {x * a, y * a}; }
double Vector::lenght() const { return std::sqrt(x * x + y * y); }
Vector Vector::norm() const {
  double l = lenght();
  if (l > 0)
    return {x / l, y / l};
  else
    return {0.0, 0.0};
}

// Metodi della classe BoidSistem:
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
        if ((boids[j].position - boid.position).lenght() < d) {
          neighbors.push_back(j);
        }
      };
    };
  };
  return neighbors;
}

std::vector<size_t> BoidSimulation::getNeighborsDS(size_t indice,
                                                   double ds) const {
  std::vector<size_t> neighborsDS;
  const Boid& boid = boids[indice];
  for (size_t i = 0; i < boids.size(); ++i) {
    if (i != indice) {
      double distance = (boids[i].position - boid.position).lenght();
      if (distance < ds && distance > 0) {
        neighborsDS.push_back(i);
      }
    }
  }
  return neighborsDS;
}

Vector BoidSimulation::separation(size_t i,
                                  const std::vector<size_t>& neighborsDS,
                                  double s) {
  if (neighborsDS.empty()) return {0.0, 0.0};
  Vector sum{0.0, 0.0};
  for (size_t j : neighborsDS) {
    Vector diff = boids[i].position - boids[j].position;
    double dist = diff.lenght();
    if (dist > 0) {
      sum = sum + (diff.norm() * (1.0 / dist));
    }
  }
  return sum * s;
}

Vector BoidSimulation::alignment(size_t i,
                                 const std::vector<size_t>& getNeighbors,
                                 double a) {
  if (getNeighbors.empty()) return {0.0, 0.0};
  Vector velocitySum{0.0, 0.0};
  for (size_t j : getNeighbors) {
    velocitySum = velocitySum + boids[j].speed;
  }
  Vector mediaVel =
      velocitySum * (1.0 / static_cast<double>(getNeighbors.size()));
  Vector diff = mediaVel - boids[i].speed;
  return diff * a;
}

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

void BoidSimulation::updateBoids(double dt) {
  std::vector<Vector> newVelocity(boids.size());
  std::vector<Vector> newPosition(boids.size());

  for (size_t i = 0; i < boids.size(); ++i) {
    const Boid& boid = boids[i];
    const Flock& f = flocks[boid.flockid];
    std::vector<size_t> neighbors = getNeighbors(i, f.d);
    std::vector<size_t> neighborsDS = getNeighborsDS(i, f.ds);

    Vector v1 = separation(i, neighborsDS, f.s);
    Vector v2 = alignment(i, neighbors, f.a);
    Vector v3 = cohesion(i, neighbors, f.c);

    Vector vel = boid.speed + (v1 + v2 + v3) * dt;
    double len = vel.lenght();
    if (len > 0.0) {
      if (len > f.maxSpeed)
        vel = vel * (f.maxSpeed / len);
      else if (len < f.minSpeed)
        vel = vel * (f.minSpeed / len);
    } else {
      vel = {f.minSpeed, 0.0};
    }

    Vector pos = boid.position + vel * dt;

    // Gestione ai bordi dello schermo:
    if (pos.x < 0) pos.x = width;
    if (pos.x > width) pos.x = 0;
    if (pos.y < 0) pos.y = height;
    if (pos.y > height) pos.y = 0;

    newVelocity[i] = vel;
    newPosition[i] = pos;
  }
  for (size_t i = 0; i < boids.size(); ++i) {
    boids[i].speed = newVelocity[i];
    boids[i].position = newPosition[i];
  }
}
}

;  // namespace b
