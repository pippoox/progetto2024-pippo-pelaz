#ifndef B_BOIDS_HPP
#define B_BOIDS_HPP

#include <cmath>
#include <vector>

namespace b {
struct vettore {
  double x;
  double y;
  vettore(double x_ = 0, double y_ = 0) : x(x_), y(y_) {}
  vettore operator+(vettore const& v) const;
  vettore operator-(vettore const& v) const;
  vettore operator*(double a) const;
  double modulo() const;
  vettore normalizzato() const;
};

class Boid {
 public:
  vettore posizione;
  vettore velocità;
  Boid(vettore pos, vettore vel) : posizione(pos), velocità(vel) {}
  void variazionePos(double dt);
  void variazioneVel(vettore const& v1, vettore const& v2, vettore const& v3,
                     double maxVelocità);
  void aggiorna(std::vector<Boid>& boids, double width, double height,
                      double d, double ds, double s, double a, double c,
                      double dt);
};

class Allboids {
 private:
  std::vector<Boid> boids;
  double rvisuale;
  double deltaTempo;
  double maxVel;

 public:
  Allboids(double d, double dt, double mv)
      : rvisuale(d), deltaTempo(dt), maxVel(mv) {}
  const std::vector<Boid>& getBoids() const;
  void aggiungiBoid(Boid const& boid);
  std::vector<Boid> boidsVicini(size_t indice, double d) const;
  std::vector<Boid> viciniDS(size_t indice, double ds) const;
  vettore separazione(Boid const& boid, const std::vector<Boid>& vicinids,
                      double s);
  vettore allineamento(Boid const& boid, const std::vector<Boid>& boidsVicini,
                       double a);

  vettore coesione(Boid const& boid, const std::vector<Boid>& boidsVicini,
                   double c);
  void aggiornaBoids(double d, double ds, double s, double a, double c);
  double getdeltaTempo() const { return deltaTempo; }
};
}  // namespace b

#endif