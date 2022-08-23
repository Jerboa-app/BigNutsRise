#ifndef PARTICLE_SYSTEM_H
#define PARTICLE_SYSTEM_H

const uint64_t NULL_INDEX = uint64_t(-1);
const int ARPERIOD = 60;
const float attractionStrength = 0.01;
const float repellingStrength = 0.02;

#include <vector>
#include <time.h>
#include <math.h>
#include <random>
#include <iostream>

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>

#include <shaders.h>
#include <glUtils.h>

std::default_random_engine generator;
std::uniform_real_distribution<double> U(0.0,1.0);
std::normal_distribution<double> normal(0.0,1.0);

class ParticleSystem{

  friend class ParticleSystemRenderer;

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
    alpha(0.0),beta(1.0),shakerPeriod(1.0),shakerAmplitude(radius),shakerTime(0.0),
    Lx(Lx), Ly(Ly)
  {

    floatState = new float [N*4];

    generator.seed(seed);
    Nc = std::ceil(1.0/(4*radius));
    deltax = Lx / Nc;
    deltay = Ly / Nc;

    shakerDisplacement = 0.0;

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

      double r = i%2 == 0 ? radius : radius / 2.0;
      double m = i%2 == 0 ? mass : mass / 4.0;

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

  double reducedMass(float m1, float m2){
      return 1.0 / (1.0/m1 + 1.0/m2);
  }

  double damping(float m1, float m2){
      double meff = reducedMass(m1,m2);
      return 2*meff*(-std::log(coefficientOfRestitution)/collisionTime);
  }

  double restoration(float m1, float m2){
      double meff = reducedMass(m1,m2);
      return meff/(collisionTime*collisionTime)*(std::log(coefficientOfRestitution)+M_PI*M_PI);
  }

  void applyForce(double fx, double fy);

  void step();

  void addParticle(){
    int i = size();

    if (i == nParticles){return;}

    double x = U(generator)*(Lx-2*radius)+radius;
    double y = U(generator)*(Ly-2*radius)+radius;
    double theta = U(generator)*2.0*3.14;

    double r = i%2 == 0 ? radius : radius / 2.0;
    double m = i%2 == 0 ? mass : mass / 4.0;

    addParticle(x,y,theta,r,m);
  }

  void removeParticle(){removeParticle(size()-1);}

  uint64_t size(){
    return uint64_t(std::floor(state.size() / 3));
  }

  double orderParameter();

  void randomiseRadii(double propBig){
    int nBig = std::floor(propBig*size());
    int nSmall = size()-nBig;
    for (int i = 0; i < size(); i++){

      bool coin = U(generator) > 0.5;

      if (nSmall == 0 && nBig > 0){
        parameters[i*2] = radius;
        parameters[i*2+1] = mass;
        nBig--;
      }
      else if (nBig > 0 && coin){
        parameters[i*2] = radius;
        parameters[i*2+1] = mass;
        nBig--;
      }
      else if (nSmall > 0){
        parameters[i*2] = radius/2.0;
        parameters[i*2+1] = mass/4.0;
        nSmall--;
      }

      floatState[i*4+3] = parameters[i*2];

    }
  }

  void setTimeStep(double dt){ if(this->dt!=dt) {newTimeStepStates(this->dt,dt);} this->dt = dt; }

  double getshakerPeriod(){return shakerPeriod;}
  void setShakerPeriod(double p){
    if (p != shakerPeriod) {shakerTime = shakerTime*p/shakerPeriod;}
    shakerPeriod = p;
  }
  double setShakerAmplitude(double a){
    shakerAmplitude = a*radius;
  }

  void setCoeffientOfRestitution(double c);
  // GL public members
  void setProjection(glm::mat4 p);
  void draw(uint64_t frameId, float zoomLevel, float resX, float resY);

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

  double shakerDisplacement;
  double shakerPeriod;
  double shakerAmplitude;
  double shakerTime;

  float * floatState;

  void addParticle(double x, double y, double theta, double r, double m){

    int i = size();

    state.push_back(x);
    state.push_back(y);
    state.push_back(theta);

    floatState[i*4] = x;
    floatState[i*4+1] = y;
    floatState[i*4+2] = theta;
    floatState[i*4+3] = r;

    lastState.push_back(x);
    lastState.push_back(y);
    lastState.push_back(theta);

    parameters.push_back(r);
    parameters.push_back(m);

    forces.push_back(0.0);
    forces.push_back(0.0);

    velocities.push_back(0.0);
    velocities.push_back(0.0);

    noise.push_back(0.0);
    noise.push_back(0.0);
  }

  void removeParticle(uint64_t i){
    if (state.size() >= 3*i){
      state.erase(
        state.begin()+3*i,
        state.begin()+3*i+3
      );

      lastState.erase(
        lastState.begin()+3*i,
        lastState.begin()+3*i+3
      );

      parameters.erase(
        parameters.begin()+2*i,
        parameters.begin()+2*i+2
      );

      forces.erase(
        forces.begin()+2*i,
        forces.begin()+2*i+2
      );

      velocities.erase(
        velocities.begin()+2*i,
        velocities.begin()+2*i+2
      );

      noise.erase(
        noise.begin()+2*i,
        noise.begin()+2*i+2
      );
    }
  }

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

  uint64_t hash(uint64_t particle){
    return uint64_t(floor(state[particle*3]/deltax))*Nc + uint64_t(floor(state[particle*3+1]/deltay));
  }

  void newTimeStepStates(double oldDt, double newDt);
};

#endif
