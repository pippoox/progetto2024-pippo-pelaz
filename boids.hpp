#ifndef B_BOIDS_HPP
#define B_BOIDS_HPP

#include <cmath>
#include <iostream>
#include <vector>

namespace b {
constexpr double maxVel = 6.0;

// Struct Vector: rappresenta un vettore 2D con operazioni di base: somma, differenza,
// prodotto per scalare, modulo e normalizzazione.
struct Vector {
  double x;
  double y;
  Vector(double x_ = 0, double y_ = 0) : x(x_), y(y_) {}
  Vector operator+(Vector const& v) const;
  Vector operator-(Vector const& v) const;
  Vector operator*(double a) const;
  double length() const;
  Vector norm() const;
}; 

// Classe Boid: rappresenta un singolo boid con position, speed e numero che
// identifica lo stormo a cui appartiene. Include metodi per aggiornare posizioni e velocità.
class Boid {
 public:
  Vector position;
  Vector speed;
  int flockid;
  Boid(Vector pos, Vector vel, int id)
      : position(pos), speed(vel), flockid(id) {}
};

// Struct Flock: contiene i parametri caratteristici per l'applicazione delle
// regole di volo di uno stormo.
struct Flock {
  int id;     // Numero che identifica il flock
  double d;   // Raggio visivo per alignment e cohesion
  double ds;  // Raggio visivo per separation
  double s;   // Peso della separation
  double a;   // Peso dell'alignment
  double c;   // Peso della cohesion

  Flock(const int& id_, const double& d_, const double& ds_, const double& s_,
        const double& a_, const double& c_)
      : id{id_}, d{d_}, ds{ds_}, s{s_}, a{a_}, c{c_} {}
};

// Classe BoidSimulation: rappresenta un insieme di boid che possono appartenere a stormi differenti. 
// Contiene anche i valori di width e height entro cui si muovono i boid. Include metodi per costruire
// simulazione.
class BoidSimulation {
 private:
  std::vector<Boid> boids;
  std::vector<Flock> flocks;

  const double width;
  const double height; 
 public:
  BoidSimulation(const std::vector<Flock>& flocks_, const double& width_, const double& height_)
      :flocks(flocks_), width(width_), height(height_) {}
  const std::vector<Boid>& getBoids() const;
  const double& getWidth() const;
  const double& getHeight() const;
  void addBoids(Boid const& boid);
  std::vector<size_t> getNeighbors(size_t indice, double d) const; // Restituisce un vettore di i entro distanza d 
  // da un boid i-esimo
  std::vector<size_t> getNeighborsDS(size_t indice, double ds) const; // Restituisce un vettore di int, che corrispondono
  // alle posizioni dei boid nel vettore boids entro distanza d da un boid i-esimo
  Vector separation(size_t i, const std::vector<size_t>& neighborsDS, double s); // Restituisce un vettore di int, 
  // che corrispondon alle posizioni dei boid nel vettore boids entro distanza ds da un boid i-esimo
  Vector alignment(size_t i, const std::vector<size_t>& getNeighbors, 
                       double a); // Calcola vettore allineamento. Si applica solo su boid dello stesso flock
  Vector cohesion(size_t i, const std::vector<size_t>& getNeighbors, double c); // Calcola vettore coesione.
  // Si applica solo su boid dello stesso flock
  void updateBoids(double dt); // Aggiorna posizione e velocità dei boid attraverso i vettori separazione, 
  // allineamento e coesione
};

}  // namespace b

#endif