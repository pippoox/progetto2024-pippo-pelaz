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

// Metodi della classe Boid:

// Aggiorna la posizione in base alla velocità e al tempo
void Boid::variazionePos(double dt) { posizione = posizione + velocità * dt; };

// Aggiorna la velocità in base ai tre contributi: separazione, allineamento,
// coesione; controlla anche che la velocità non superi il massimo
void Boid::variazioneVel(vettore const& v1, vettore const& v2,
                         vettore const& v3, double maxVelocità) {
  velocità = velocità + v1 + v2 + v3;
  if (velocità.modulo() > maxVelocità) {
    velocità = velocità.normalizzato() * maxVelocità;
  }
}

// Metodi della classe Flock:
void Flock::aggiungiBoid(Boid const& boid) {
  boids.push_back(boid);
};  // Aggiunge un nuovo boid al sistema
const std::vector<Boid>& Flock::getBoids() const {
  return boids;
}  // Restituisce tutti i boid

// Restituisce tutti i boid vicini entro distanza "d" dal boid con indice dato
std::vector<Boid> Flock::boidsVicini(size_t indice, double d) const {
  std::vector<Boid> vicini;
  const Boid& boid = boids[indice];
  for (size_t i = 0; i < boids.size(); ++i) {
    if (i != indice) {
      if ((boids[i].posizione - boid.posizione).modulo() < d) {
        vicini.push_back(boids[i]);
      };
    };
  };
  return vicini;
};

// Restituisce i boid troppo vicini (entro distanza ds)
std::vector<Boid> Flock::viciniDS(size_t indice, double ds) const {
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

// Calcolo del vettore separazione (evita collisioni)
vettore Flock::separazione(Boid const& boid, const std::vector<Boid>& vicinids,
                           double s) {
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

// Calcolo del vettore allineamento (uniformità direzione)
vettore Flock::allineamento(Boid const& boid,
                            const std::vector<Boid>& boidsVicini, double a) {
  if (boidsVicini.empty()) return {0.0, 0.0};
  vettore sommaVel{0.0, 0.0};
  for (auto& b : boidsVicini) {
    sommaVel = sommaVel + b.velocità;
  }
  vettore mediaVel = sommaVel * (1.0 / static_cast<double>(boidsVicini.size()));
  vettore diff = mediaVel - boid.velocità;
  return diff * a;
}

// Calcolo del vettore coesione (muoversi verso il centro dei vicini)
vettore Flock::coesione(Boid const& boid, const std::vector<Boid>& boidsVicini,
                        double c) {
  if (boidsVicini.empty()) {
    return {0.0, 0.0};
  }
  vettore sommaPos{0.0, 0.0};
  for (const auto& b : boidsVicini) {
    sommaPos = sommaPos + b.posizione;
  }
  vettore centrodimassa =
      sommaPos * (1.0 / static_cast<double>(boidsVicini.size()));
  vettore direzionecentro = centrodimassa - boid.posizione;
  return direzionecentro * c;
}

// Aggiorna velocità e posizione di tutti i boid nel sistema
void Flock::aggiornaBoids(double d, double ds, double s, double a, double c,
                          double width, double height) {
  maxVel = 2.0;

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
    boids[i].velocità = nuovaVel;
    boids[i].posizione = boids[i].posizione + boids[i].velocità * dt;

    // Gestione ai bordi dello schermo:
    if (boids[i].posizione.x < 0) boids[i].posizione.x = width;
    if (boids[i].posizione.x > width) boids[i].posizione.x = 0;
    if (boids[i].posizione.y < 0) boids[i].posizione.y = height;
    if (boids[i].posizione.y > height) boids[i].posizione.y = 0;
  }
}
};  // namespace b
