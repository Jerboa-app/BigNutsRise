#ifndef PARTICLE_SYSTEM_H
#define PARTICLE_SYSTEM_H

const uint64_t NULL_INDEX = uint64_t(-1);

#include <vector>
#include <time.h>
#include <math.h>
#include <random>
#include <iostream>
#include <thread>

std::default_random_engine generator;
std::uniform_real_distribution<double> U(0.0,1.0);
std::normal_distribution<double> normal(0.0,1.0);

class ParticleSystem{

  friend class ParticleSystemRenderer;
  friend class Trajectory;

public:

  ParticleSystem(
    uint64_t N,
    double dt = 1.0/300.0,
    double density = 0.25,
    double Lx = 0.5, double Ly = 1.0,
    uint64_t seed = clock()
  )
  : nParticles(N), radius(std::sqrt(density/(N*M_PI))),speed(0),drag(0),
    rotationalDrag(.01),mass(1.0), momentOfInertia(0.01),
    rotationalDiffusion(0.01),dt(dt),collisionTime(10*dt),
    shakerPeriod(1.0),shakerAmplitude(radius),shakerTime(0.0),
    Lx(Lx), Ly(Ly)
  {

    floatState = new float [N*4];

    generator.seed(seed);
    Nc = std::ceil(1.0/(4.0*radius));
    deltax = Lx / Nc;
    deltay = Ly / Nc;

    shakerDisplacement = shakerAmplitude/2.0;
    massRatio = 1.0;
    radiusRatio = 2.0;

    for (int c = 0; c < Nc*Nc; c++){
      cells.push_back(NULL_INDEX);
    }

    for (int i = 0; i < N; i++){
      list.push_back(NULL_INDEX);
    }

    for (int i = 0; i < N; i++){
      double x = U(generator)*(Lx-2*radius)+radius;
      double y = U(generator)*(Ly-2*radius)+radius;
      double theta = U(generator)*2.0*3.14;

      double r = i%2 == 0 ? radius : radius / radiusRatio;
      double m = i%2 == 0 ? mass : mass / massRatio;

      addParticle(x,y,theta,r,m);
      uint64_t c = hash(i);
      if (cells[c] == NULL_INDEX){
        cells[c] = i;
      }
      else{
        insert(cells[c],uint64_t(i));
      }
    }

    setCoeffientOfRestitution(0.95);
  }

  void applyForce(double fx, double fy);

  void step();

  void addParticle();

  void removeParticle(){removeParticle(size()-1);}

  uint64_t size(){return uint64_t(std::floor(state.size() / 3));}

  // for judging separation, 1 means perfect separation, 0 means random
  //   particle placement, works 'better' the more particles there are
  double orderParameter();

  // parameter setters

  void randomise(double propBig);

  void setTimeStep(double dt){ if(this->dt!=dt) {newTimeStepStates(this->dt,dt);} this->dt = dt; dtdt = dt*dt; }

  void setShakerPeriod(double p){
    if (p != shakerPeriod) {shakerTime = shakerTime*p/shakerPeriod;}
    shakerPeriod = p;
  }

  void setMassRatio(double mr){
    if (massRatio != mr){massRatio = mr; randomise(0.5);}
  }

  void setRadiusRatio(double rr){
    if (radiusRatio != rr){radiusRatio = rr; randomise(0.5);}
  }

  void changeRatio();

  void setShakerAmplitude(double a){
    if (shakerAmplitude != a*radius){
      shakerAmplitude = a*radius; shakerDisplacement = shakerAmplitude;
    }
  }

  // depends on collision time (which should be >> timestep, 10x seems to work)
  //  and the masses of two colliding particles.
  void setCoeffientOfRestitution(double c);

  // helper for setting restitution
  double reducedMass(float m1, float m2){
      return 1.0 / (1.0/m1 + 1.0/m2);
  }

  // helper for setting restitution
  double damping(float m1, float m2){
      double meff = reducedMass(m1,m2);
      return 2*meff*(-std::log(coefficientOfRestitution)/collisionTime);
  }

  // helper for setting restitution
  double restoration(float m1, float m2){
      double meff = reducedMass(m1,m2);
      return meff/(collisionTime*collisionTime)*(std::log(coefficientOfRestitution)+M_PI*M_PI);
  }

  // parameter getters

  double getshakerPeriod(){return shakerPeriod;}
  double getShakerAmplitude(){return shakerAmplitude/radius;}

  // scenarios

  void oneBigOnBottom();
  void one();

  ~ParticleSystem(){
    free(floatState);
  }

private:

  std::vector<double> state;
  std::vector<double> lastState;
  std::vector<double> noise;

  std::vector<double> parameters;

  std::vector<double> forces;
  std::vector<double> velocities;

  std::vector<uint64_t> cells;
  std::vector<uint64_t> list;

  double Lx;
  double Ly;

  uint64_t Nc;
  double deltax;
  double deltay;

  uint64_t nParticles;

  double coefficientOfRestitution;
  double collisionTime;

  double massRatio;
  double radiusRatio;

  double dampingSS; // small-small
  double dampingBB; // big-big
  double dampingSB;

  double restorationSS;
  double restorationBB;
  double restorationSB;

  double alpha;
  double beta;

  double rotationalDiffusion;
  double speed;
  double radius;
  double drag;
  double rotationalDrag;
  double mass;
  double momentOfInertia;
  double dt;
  double dtdt;

  double shakerDisplacement;
  double shakerPeriod;
  double shakerAmplitude;
  double shakerTime;

  float * floatState;

  void addParticle(double x, double y, double theta, double r, double m);
  void removeParticle(uint64_t i);

  // Cell Linked List Collisions detection
  void resetLists();
  void insert(uint64_t next, uint64_t particle);
  void populateLists();
  void handleCollision(uint64_t i, uint64_t j);
  void cellCollisions(
    uint64_t a1,
    uint64_t b1,
    uint64_t a2,
    uint64_t b2
  );

  uint64_t hash(float x, float y){
    return uint64_t(floor(x/deltax))*Nc + uint64_t(floor(y/deltay));
  }

  uint64_t hash(uint64_t particle){
    uint64_t h = uint64_t(floor(state[particle*3]/deltax))*Nc + uint64_t(floor(state[particle*3+1]/deltay));
    // a particle outside the box would normally crash the program
    //  (segfault), if this happens there is no logic to get it back in currently
    //  these cases happen due to numerical instability, so a smaller time step
    //  will help! - pretty rare with defaults
    if (h < 0 || h > Nc*Nc){return 0;}
    return h;
  }

  // integrator requires current and last positions
  //  changing the timestep can introduce instability if
  //  not accounted for
  void newTimeStepStates(double oldDt, double newDt);
};

#endif
