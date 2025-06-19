#include "boids.hpp"

namespace b {

vettore vettore::operator+(vettore const& v) const {
  return {x + v.x, y + v.y};
}
vettore vettore::operator-(vettore const& v) const {
  return {x - v.x, y - v.y};
}
vettore vettore::operator*(double a) const { return {x * a, y * a}; }
double vettore::modulo() const { return std::sqrt(x * x + y * y); }
vettore vettore::normalizzato() const {
  double l = modulo();
  if (l > 0)
    return {x / l, y / l};
  else
    return {0.0, 0.0};
}

void Boid::variazionePos(double dt) { posizione = posizione + velocità * dt; };
void Boid::variazioneVel(vettore const& v1, vettore const& v2,
                         vettore const& v3, double maxVelocità) {
  velocità = velocità + v1 + v2 + v3;
  if (velocità.modulo() > maxVelocità) {
    velocità = velocità.normalizzato() * maxVelocità;
  }
}
void Boid::aggiorna(std::vector<Boid>& boids, double width, double height,
  double d, double ds, double s, double a, double c, double dt) {
vettore somma_allineamento(0.f, 0.f);
vettore somma_coesione(0.f, 0.f);
vettore separazione(0.f, 0.f);
int count = 0;

for (const auto& altro : boids) {
if (&altro == this) continue;

double distanza = (altro.posizione - posizione).modulo();

if (distanza < d) {

somma_allineamento = somma_allineamento + altro.velocità;


somma_coesione = somma_coesione +altro.posizione;

count++;


if (distanza < ds && distanza > 0) {
separazione = separazione - (altro.posizione - posizione) * (1.0/ (distanza * distanza));
}
}
}

vettore allineamento(0.f, 0.f);
vettore coesione(0.f, 0.f);

if (count > 0) {

allineamento = (somma_allineamento * (1.0 / static_cast<double>(count))) - velocità;


vettore centro = somma_coesione * (1.0 / static_cast<double>(count));
coesione = centro - posizione;
}


velocità = velocità + separazione * s  + allineamento * a + coesione * c;


double vmax = 2.0f;
if (velocità.modulo() > vmax) {
velocità = velocità.normalizzato() * vmax;
}


posizione = posizione + velocità * dt;


if (posizione.x < 0) posizione.x = width;
if (posizione.x > width) posizione.x = 0;
if (posizione.y < 0) posizione.y = height;
if (posizione.y > height) posizione.y = 0;
}


void Allboids::aggiungiBoid(Boid const& boid) { boids.push_back(boid); };
const std::vector<Boid>& Allboids::getBoids() const { return boids; }
std::vector<Boid> Allboids::boidsVicini(size_t indice, double d) const {
  std::vector<Boid> vicini;
  const Boid& boid = boids[indice];
  for (size_t i = 0; i < boids.size(); ++i) {
    if (i != indice) {
      if ((boids[i].posizione - boid.posizione).modulo() < rvisuale) {
        vicini.push_back(boids[i]);
      };
    };
  };
  return vicini;
};
std::vector<Boid> Allboids::viciniDS(size_t indice, double ds) const {
  std::vector<Boid> vicinids;
  const Boid& boid = boids[indice];
  for (size_t i = 0; i < boids.size(); ++i) {
    if (i != indice) {
      double distanza = (boids[i].posizione - boid.posizione).modulo();
      if (distanza < ds && distanza > 0) {
        vicinids.push_back(boids[i]);
      }
    }
  }
  return vicinids;
}
vettore Allboids::separazione(Boid const& boid,
                              const std::vector<Boid>& vicinids, double s) {
  vettore somma{0.0, 0.0};
  for (const auto& b : vicinids) {
    vettore diff = boid.posizione - b.posizione;
    double dist = diff.modulo();
    if (dist > 0) {
      somma = somma + (diff.normalizzato() * (1.0 / dist));
    }
  }
  return somma * s;
}

vettore Allboids::allineamento(Boid const& boid,
                               const std::vector<Boid>& boidsVicini, double a) {
  if (boidsVicini.empty()) return {0.0, 0.0};
  vettore sommaVel{0.0, 0.0};
  for (auto& b : boidsVicini) {
    sommaVel = sommaVel + b.velocità;
  }
  vettore mediaVel = sommaVel * (1.0 / boidsVicini.size());
  vettore diff = mediaVel - boid.velocità;
  return diff * a;
}
vettore Allboids::coesione(Boid const& boid,
                           const std::vector<Boid>& boidsVicini, double c) {
  if (boidsVicini.empty()) {
    return {0.0, 0.0};
  }
  vettore sommaPos{0.0, 0.0};
  for (const auto& b : boidsVicini) {
    sommaPos = sommaPos + b.posizione;
  }
  vettore centrodimassa = sommaPos * (1.0 / boidsVicini.size());
  vettore direzionecentro = centrodimassa - boid.posizione;
  return direzionecentro * c;
}
void Allboids::aggiornaBoids(double d, double ds, double s, double a,
                             double c) {
  std::vector<vettore> nuoveVelocità(boids.size());
  const double maxVel = 2.0;
  for (size_t i = 0; i < boids.size(); ++i) {
    const Boid& boid = boids[i];
    std::vector<Boid> vicini = boidsVicini(i, d);
    std::vector<Boid> vicinids = viciniDS(i, ds);

    vettore v1 = separazione(boid, vicinids, s);
    vettore v2 = allineamento(boid, vicini, a);
    vettore v3 = coesione(boid, vicini, c);

    vettore nuovaVel = boid.velocità + v1 + v2 + v3;
    double modulo = nuovaVel.modulo();
    if (modulo > maxVel) {
      nuovaVel = nuovaVel * (maxVel / modulo);
    }
    nuoveVelocità[i] = nuovaVel;
  }
  for (size_t i = 0; i < boids.size(); ++i) {
    boids[i].velocità = nuoveVelocità[i];
    boids[i].posizione = boids[i].posizione + boids[i].velocità * deltaTempo;
  }
}
};  // namespace b
