#ifndef B_BOIDS_HPP
#define B_BOIDS_HPP

#include <vector>
#include <cstddef>

namespace b {

// Struct Vector: rappresenta un vettore 2D con operazioni di base: somma,
// differenza, prodotto per scalare, modulo e normalizzazione.
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

// Struct Boid: rappresenta un singolo boid con position, speed e numero che
// identifica lo stormo a cui appartiene.
struct Boid {
  Vector position;
  Vector speed;
  int flockid;
  Boid(Vector pos, Vector vel, int id)
      : position(pos), speed(vel), flockid(id) {}
};

// Struct Flock: contiene i parametri caratteristici per l'applicazione delle
// regole di volo di uno stormo e un int che identifica lo stormo.
struct Flock {
  int id;     // Numero che identifica il flock
  double d;   // Raggio visivo per alignment e cohesion
  double ds;  // Raggio visivo per separation
  double s;   // Peso della separation
  double a;   // Peso dell'alignment
  double c;   // Peso della cohesion
  double minSpeed;
  double maxSpeed;

  Flock(int id_, double d_, double ds_, double s_, double a_, double c_,
        double minS_, double maxS_)
      : id{id_},
        d{d_},
        ds{ds_},
        s{s_},
        a{a_},
        c{c_},
        minSpeed{minS_},
        maxSpeed{maxS_} {}
};

// Classe BoidSimulation: rappresenta un insieme di boid che possono appartenere
// a stormi differenti. Contiene anche i valori di width e height che
// definiscono le dimensioni della finestra grafica. Include metodi per
// costruire simulazione.
class BoidSimulation {
 private:
  std::vector<Boid> boids;
  std::vector<Flock> flocks;

  const double width;
  const double height;

 public:
  BoidSimulation(const std::vector<Flock>& flocks_, const double& width_,
                 const double& height_)
      : flocks(flocks_), width(width_), height(height_) {}
  const std::vector<Boid>& getBoids() const;
  const double& getWidth() const;
  const double& getHeight() const;
  void addBoids(Boid const& boid);
  std::vector<size_t> getNeighbors(size_t indice, double d)
      const;  // Restituisce un vettore di int (posizione boid dentro vettore
              // boids) entro distanza d da un boid i-esimo
  std::vector<size_t> getNeighborsDS(size_t indice, double ds)
      const;  // Restituisce un vettore di int(posizione boid dentro vettore
              // boids)entro distanza ds da un boid i-esimo
  Vector separation(size_t i, const std::vector<size_t>& neighborsDS,
                    double s);  // Calcola correzione velocità data dalla regola
                                // di separazione sui boid del vettore viciniDS.
                                // Si applica su tutti i boid
  Vector alignment(size_t i, const std::vector<size_t>& getNeighbors,
                   double a);  // Calcola correzione velocità data dalla regola
                               // di allineamento sui boid del vettore vicini.
                               // Si applica solo su boid dello stesso flock.
  Vector cohesion(size_t i, const std::vector<size_t>& getNeighbors,
                  double c);  // Calcola correzione velocità data dalla regola
                              // di coesione sui boid del vettore vicini.
                              // Si applica solo su boid dello stesso flock.
  void updateBoids(
      double dt);  // Aggiorna posizione e velocità dei boid
                   // attraverso le correzioni di velocità date da separazione,
                   // allineamento e coesione
};

}  // namespace b

#endif