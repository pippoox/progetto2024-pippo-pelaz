#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "boids.hpp"

#include "doctest.h"

TEST_CASE("Testing vettore implementation") {
  SUBCASE("Testing operator+") {
    b::vettore v1 = {1.0, 2.0};
    b::vettore v2 = {2.0, 3.0};
    b::vettore result = v1 + v2;
    CHECK(result.x == doctest::Approx(3.0));
    CHECK(result.y == doctest::Approx(5.0));
  }
  SUBCASE("Testing operator-") {
    b::vettore v1 = {5.0, 6.0};
    b::vettore v2 = {1.0, 2.0};
    b::vettore result = v1 - v2;
    CHECK(result.x == doctest::Approx(4.0));
    CHECK(result.y == doctest::Approx(4.0));
  }
  SUBCASE("Testing operator*") {
    b::vettore v = {2.0, 3.0};
    double scalar = 2;
    b::vettore result = v * scalar;
    CHECK(result.x == doctest::Approx(4.0));
    CHECK(result.y == doctest::Approx(6.0));
  }
  SUBCASE("Testing modulo") {
    b::vettore v = {3.0, 4.0};
    CHECK(v.modulo() == doctest::Approx(5.0));
  }
  SUBCASE("Testing modulo con v nullo") {
    b::vettore v = {0.0, 0.0};
    CHECK(v.modulo() == doctest::Approx(0.0));
  }
  SUBCASE("Testing normalizzazione") {
    b::vettore v = {3.0, 4.0};
    b::vettore normalized_v = v.normalizzato();
    CHECK(normalized_v.modulo() == doctest::Approx(1.0));
    CHECK(normalized_v.x == doctest::Approx(0.6));
    CHECK(normalized_v.y == doctest::Approx(0.8));
  }
  SUBCASE("Testing normalizzazione con v nullo") {
    b::vettore v = {0.0, 0.0};
    b::vettore normalized_v = v.normalizzato();
    CHECK(normalized_v.x == doctest::Approx(0.0));
    CHECK(normalized_v.y == doctest::Approx(0.0));
  }
}
TEST_CASE("Testing Boid implementation") {
  SUBCASE("Testing variazionePos") {
    b::vettore pos = {0.0, 0.0};
    b::vettore vel = {1.0, 1.0};
    b::Boid b(pos, vel);
    double dt = 2.0;
    b.variazionePos(dt);
    CHECK(b.posizione.x == doctest::Approx(2.0));
    CHECK(b.posizione.y == doctest::Approx(2.0));
  }
  SUBCASE("Testing variazioneVel con v>vmax") {
    b::vettore pos = {0.0, 0.0};
    b::vettore vel = {1.0, 0.0};
    b::Boid b(pos, vel);
    b::vettore v1 = {0.5, 0.0};
    b::vettore v2 = {0.3, 0.0};
    b::vettore v3 = {0.1, 0.0};
    double maxVelocità = 1.0;
    b.variazioneVel(v1, v2, v3, maxVelocità);
    CHECK(b.velocità.x == doctest::Approx(1.0));
    CHECK(b.velocità.y == doctest::Approx(0.0));
    CHECK(b.velocità.modulo() == doctest::Approx(maxVelocità));
  }
  SUBCASE("Testing variazioneVel con v<vmax") {
    b::vettore pos = {0.0, 0.0};
    b::vettore vel = {0.1, 0.0};
    b::Boid b(pos, vel);
    b::vettore v1 = {0.1, 0.0};
    b::vettore v2 = {0.1, 0.0};
    b::vettore v3 = {0.1, 0.0};
    double maxVelocità = 2.0;
    b.variazioneVel(v1, v2, v3, maxVelocità);
    CHECK(b.velocità.x == doctest::Approx(0.4));
    CHECK(b.velocità.y == doctest::Approx(0.0));
  }
}
TEST_CASE("Testing Allboids implementation") {
  SUBCASE("Testing boidsVicini") {
    b::Allboids gruppo(10.0, 0.1, 2.0);
    b::Boid boid1({0.0, 0.0}, {0.0, 0.0});
    b::Boid boid2({5.0, 0.0}, {0.0, 0.0});
    b::Boid boid3({15.0, 0.0}, {0.0, 0.0});
    gruppo.aggiungiBoid(boid1);
    gruppo.aggiungiBoid(boid2);
    gruppo.aggiungiBoid(boid3);
    const b::Boid& boidriferimento = gruppo.getBoids()[0];
    size_t indice = 0;

    std::vector<b::Boid> vicini = gruppo.boidsVicini(indice, 10.0);
    CHECK(vicini.size() == 1);
    CHECK(vicini[0].posizione.x == doctest::Approx(5.0));
  }
  SUBCASE("Testing boidsVicini vuoto") {
    b::Allboids gruppo(10.0, 0.1, 2.0);
    b::Boid boid1({0.0, 0.0}, {0.0, 0.0});
    gruppo.aggiungiBoid(boid1);

    const b::Boid& boidriferimento = gruppo.getBoids()[0];
    size_t indice =0;
    std::vector<b::Boid> vicini = gruppo.boidsVicini(indice, 10.0);
    CHECK(vicini.empty());
  }
  SUBCASE("Testing viciniDS") {
    b::Allboids gruppo(10.0, 0.1, 2.0);
    b::Boid boid1({0.0, 0.0}, {0.0, 0.0});
    b::Boid boid2({1.0, 0.0}, {0.0, 0.0});
    b::Boid boid3({3.0, 0.0}, {0.0, 0.0});
    gruppo.aggiungiBoid(boid1);
    gruppo.aggiungiBoid(boid2);
    gruppo.aggiungiBoid(boid3);
    size_t indice = 0;

    std::vector<b::Boid> vicinids = gruppo.viciniDS(indice, 2.0);
    CHECK(vicinids.size() == 1);
    CHECK(vicinids[0].posizione.x == doctest::Approx(1.0));
  }
  SUBCASE("Testing vettore di separazione") {
    b::Allboids gruppo(10.0, 0.1, 2.0);
    b::Boid boid1({0.0, 0.0}, {0.0, 0.0});
    b::Boid boid2({1.0, 0.0}, {0.0, 0.0});
    b::Boid boid3({-1.0, 0.0}, {0.0, 0.0});

    std::vector<b::Boid> vicinids = {boid2, boid3};
    double s = 1.0;
    b::vettore separazione = gruppo.separazione(boid1, vicinids, s);
    CHECK(separazione.x == doctest::Approx(0.0));
    CHECK(separazione.y == doctest::Approx(0.0));
  }
  SUBCASE("Testing separazione con un solo vicino") {
    b::Allboids gruppo(10.0, 0.1, 2.0);
    b::Boid boid1({0.0, 0.0}, {0.0, 0.0});
    b::Boid boid2({1.0, 0.0}, {0.0, 0.0});

    std::vector<b::Boid> vicinids = {boid2};
    double s = 1.0;
    b::vettore separazione = gruppo.separazione(boid1, vicinids, s);
    CHECK(separazione.x == doctest::Approx(-1.0));
    CHECK(separazione.y == doctest::Approx(0.0));
  }
  SUBCASE("Testing vettore allineamento") {
    b::Allboids gruppo(10.0, 0.1, 2.0);
    b::Boid boid1({0.0, 0.0}, {1.0, 0.0});
    b::Boid boid2({1.0, 0.0}, {2.0, 0.0});
    b::Boid boid3({2.0, 0.0}, {3.0, 0.0});

    std::vector<b::Boid> boidsVicini = {boid2, boid3};
    double a = 0.5;
    b::vettore allineamento = gruppo.allineamento(boid1, boidsVicini, a);

    CHECK(allineamento.x == doctest::Approx(0.75));
    CHECK(allineamento.y == doctest::Approx(0.0));
  }
  SUBCASE("Testing allineamento con nessun vicino") {
    b::Allboids gruppo(10.0, 0.1, 2.0);
    b::Boid boid1({0.0, 0.0}, {1.0, 0.0});
    std::vector<b::Boid> boidsVicini = {};
    double a = 0.5;
    b::vettore allineamento = gruppo.allineamento(boid1, boidsVicini, a);
    CHECK(allineamento.x == doctest::Approx(0.0));
    CHECK(allineamento.y == doctest::Approx(0.0));
  }
  SUBCASE("Testing vettore coesione") {
    b::Allboids gruppo(10.0, 0.1, 2.0);
    b::Boid boid1({0.0, 0.0}, {0.0, 0.0});
    b::Boid boid2({1.0, 1.0}, {0.0, 0.0});
    b::Boid boid3({-1.0, 1.0}, {0.0, 0.0});

    std::vector<b::Boid> boidsVicini = {boid2, boid3};
    double c = 0.5;
    b::vettore coesione = gruppo.coesione(boid1, boidsVicini, c);

    CHECK(coesione.x == doctest::Approx(0.0));
    CHECK(coesione.y == doctest::Approx(0.5));
  }
  SUBCASE("Testing coesione con nessun vicino") {
    b::Allboids gruppo(10.0, 0.1, 2.0);
    b::Boid boid1({0.0, 0.0}, {0.0, 0.0});
    std::vector<b::Boid> boidsVicini = {};
    double c = 0.5;
    b::vettore coesione = gruppo.coesione(boid1, boidsVicini, c);
    CHECK(coesione.x == doctest::Approx(0.0));
    CHECK(coesione.y == doctest::Approx(0.0));
  }
  SUBCASE("Testing aggiornaBoids con nessun vicino") {
    b::Allboids gruppo(10.0, 0.1, 2.0);

    b::vettore initial_pos = {0.0, 0.0};
    b::vettore initial_vel = {1.0, 0.0};
    gruppo.aggiungiBoid(b::Boid(initial_pos, initial_vel));

    gruppo.aggiornaBoids(10.0, 1.0, 0.5, 0.5, 0.5);

    const std::vector<b::Boid>& updated_boids = gruppo.getBoids();

    CHECK(updated_boids.size() == 1);

    CHECK(updated_boids[0].posizione.x == doctest::Approx(0.1));
    CHECK(updated_boids[0].posizione.y == doctest::Approx(0.0));

    CHECK(updated_boids[0].velocità.x == doctest::Approx(1.0));
    CHECK(updated_boids[0].velocità.y == doctest::Approx(0.0));
  }
  SUBCASE("Testing aggiornaboids con separazione, no a,c") {
    b::Allboids gruppo(10.0, 0.1, 2.0);

    b::vettore posA = {0.0, 0.0};
    b::vettore velA = {0.0, 0.0};
    gruppo.aggiungiBoid(b::Boid(posA, velA));

    b::vettore posB = {0.1, 0.0};
    b::vettore velB = {0.0, 0.0};
    gruppo.aggiungiBoid(b::Boid(posB, velB));

    double raggioVisuale = 10.0;
    double raggioSeparazione = 0.5;
    double fattoreSeparazione = 1.0;
    double fattoreAllineamento = 0.0;
    double fattoreCoesione = 0.0;

    gruppo.aggiornaBoids(raggioVisuale, raggioSeparazione, fattoreSeparazione, fattoreAllineamento,
                         fattoreCoesione);

    const std::vector<b::Boid>& updated_boids = gruppo.getBoids();

    CHECK(updated_boids.size() == 2);

    double nuovaDistanza =
        (updated_boids[0].posizione - updated_boids[1].posizione).modulo();
    CHECK(nuovaDistanza > doctest::Approx(0.1));

    CHECK(updated_boids[0].velocità.x < doctest::Approx(0.0));
    CHECK(updated_boids[1].velocità.x > doctest::Approx(0.0));

    CHECK(updated_boids[0].velocità.x == doctest::Approx(-updated_boids[1].velocità.x));

    CHECK(updated_boids[0].posizione.x < doctest::Approx(posA.x));
    CHECK(updated_boids[1].posizione.x > doctest::Approx(posB.x));
  }
  SUBCASE("Testing aggiornaboids con allineamento, no s,c") {
    b::Allboids gruppo(10.0, 0.1, 2.0);

    b::vettore posA = {0.0, 0.0};
    b::vettore velA = {1.0, 1.0};
    gruppo.aggiungiBoid(b::Boid(posA, velA));

    b::vettore posB = {1.0, 0.0};
    b::vettore velB = {-1.0, -1.0};
    gruppo.aggiungiBoid(b::Boid(posB, velB));
    double raggioVisuale = 10.0;

    double raggioSeparazioneDS = 0.1;

    double fattoreSeparazione = 0.0;

    double fattoreAllineamento = 1.0;

    double fattoreCoesione = 0.0;

    gruppo.aggiornaBoids(raggioVisuale, raggioSeparazioneDS, fattoreSeparazione,
                        fattoreAllineamento, fattoreCoesione);

    const std::vector<b::Boid>& updated_boids = gruppo.getBoids();

    CHECK(updated_boids.size() == 2);

    CHECK(updated_boids[0].velocità.x == doctest::Approx(-1.0));
    CHECK(updated_boids[0].velocità.y == doctest::Approx(-1.0));

    CHECK(updated_boids[1].velocità.x == doctest::Approx(1.0));
    CHECK(updated_boids[1].velocità.y == doctest::Approx(1.0));

    CHECK(updated_boids[0].posizione.x ==
          doctest::Approx(posA.x +
                          updated_boids[0].velocità.x * gruppo.getdeltaTempo()));
    CHECK(updated_boids[0].posizione.y ==
          doctest::Approx(posA.y +
                          updated_boids[0].velocità.y * gruppo.getdeltaTempo()));

    CHECK(updated_boids[1].posizione.x ==
          doctest::Approx(posB.x +
                          updated_boids[1].velocità.x * gruppo.getdeltaTempo()));
    CHECK(updated_boids[1].posizione.y ==
          doctest::Approx(posB.y +
                          updated_boids[1].velocità.y * gruppo.getdeltaTempo()));
  }
  SUBCASE("Testing aggiornaboids con coesione, no s,a") {
    b::Allboids gruppo(10.0, 0.1, 2.0);

    b::vettore posA = {-2.0, 0.0};
    b::vettore velA = {0.0, 0.0};
    gruppo.aggiungiBoid(b::Boid(posA, velA));

    b::vettore posB = {2.0, 0.0};
    b::vettore velB = {0.0, 0.0};
    gruppo.aggiungiBoid(b::Boid(posB, velB));

    double raggioVisuale = 10.0;
    double raggioSeparazioneDS = 0.1;
    double fattoreSeparazione = 0.0;
    double fattoreAllineamento = 0.0;
    double fattoreCoesione = 1.0;

    gruppo.aggiornaBoids(raggioVisuale, raggioSeparazioneDS, fattoreSeparazione,
                         fattoreAllineamento, fattoreCoesione);

    const std::vector<b::Boid>& updated_boids = gruppo.getBoids();

    CHECK(updated_boids.size() == 2);

    double nuovaDistanza =
        (updated_boids[0].posizione - updated_boids[1].posizione).modulo();
    CHECK(nuovaDistanza < doctest::Approx(4.0));

    CHECK(updated_boids[0].velocità.x == doctest::Approx(2.0)); 
    CHECK(updated_boids[1].velocità.x == doctest::Approx(-2.0)); 
    CHECK(updated_boids[0].velocità.y == doctest::Approx(0.0));
    CHECK(updated_boids[1].velocità.y == doctest::Approx(0.0));


    CHECK(updated_boids[0].posizione.x > doctest::Approx(posA.x));
    CHECK(updated_boids[1].posizione.x < doctest::Approx(posB.x));
  }
}