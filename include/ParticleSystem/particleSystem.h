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
public:

  ParticleSystem(
    uint64_t N,
    double dt = 1.0/300.0,
    double density = 0.4,
    uint64_t seed = clock()
  )
  : nParticles(N), radius(std::sqrt(density/(N*M_PI))),speed(0),drag(0.0),
    rotationalDrag(.01),mass(0.001), momentOfInertia(0.01),
    rotationalDiffusion(0.01),dt(dt),damping(5.0),restoration(10000.0),
    alpha(0.0),beta(1.0),shakerPeriod(0.1),shakerAmplitude(0.1),shakerTime(0.0)
  {
    generator.seed(seed);
    Nc = std::ceil(1.0/(2.0*radius));
    delta = 1.0 / Nc;

    for (int c = 0; c < Nc*Nc; c++){
      cells.push_back(NULL_INDEX);
    }

    for (int i = 0; i < N; i++){
      double x = U(generator)*(1.0-2*radius)+radius;
      double y = U(generator)*(1.0-2*radius)+radius;
      double theta = U(generator)*2.0*3.14;

      double r = i%2 == 0 ? radius : radius / 2.0;
      double m = i%2 == 0 ? mass : mass / 4.0;

      addParticle(x,y,theta,r,mass);
      uint64_t c = hash(i);
      if (cells[c] == NULL_INDEX){
        cells[c] = i;
      }
      else{
        insert(cells[c],uint64_t(i));
      }
    }
    initialiseGL();
  }

  void applyForce(double fx, double fy);

  void step();

  void addParticle(double x, double y, double theta, double r, double m){
    state.push_back(x);
    state.push_back(y);
    state.push_back(theta);

    floatState.push_back(x);
    floatState.push_back(y);
    floatState.push_back(theta);
    floatState.push_back(r);

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

    list.push_back(NULL_INDEX);
  }

  void removeParticle(uint64_t i){
    if (state.size() >= 3*i){
      state.erase(
        state.begin()+3*i,
        state.begin()+3*i+3
      );

      floatState.erase(
        floatState.begin()+4*i,
        floatState.begin()+4*i+4
      );

      lastState.erase(
        lastState.begin()+3*i,
        lastState.begin()+3*i+3
      );

      parameters.erase(
        parameters.begin()+2*i,
        parameters.end()+2*i+1
      );

      forces.erase(
        forces.begin()+2*i,
        forces.begin()+2*i+1
      );

      velocities.erase(
        velocities.begin()+2*i,
        velocities.begin()+2*1+1
      );

      noise.erase(
        noise.begin()+2*i,
        noise.begin()+2*i+1
      );

      list.erase(list.begin()+i);
    }
  }

  uint64_t size(){
    return uint64_t(std::floor(state.size() / 3));
  }

  void setTimeStep(double dt){ if(this->dt!=dt) {newTimeStepStates(this->dt,dt);} this->dt = dt; }

  // GL public members
  void setProjection(glm::mat4 p);
  void draw(uint64_t frameId, float zoomLevel, float resX, float resY);

  ~ParticleSystem(){
    // kill some GL stuff
    glDeleteProgram(particleShader);

    glDeleteBuffers(1,&offsetVBO);
    glDeleteBuffers(1,&vertVBO);

    glDeleteVertexArrays(1,&vertVAO);
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

  uint64_t Nc;
  double delta;

  uint64_t nParticles;

  double damping;
  double restoration;
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

  double shakerPeriod;
  double shakerAmplitude;
  double shakerTime;

  // GL data members
  std::vector<float> floatState;
  GLuint particleShader, offsetVBO, vertVAO, vertVBO;
  glm::mat4 projection;

  float vertices[3] = {0.0,0.0,0.0};

  GLuint shakerShader, shakerVAO, shakerVBO;

  float shakerVertices[6*2] = {
    -1.0, 0.0,
    -1.0, 1.0,
     0.0, 1.0,
    -1.0, 0.0,
     0.0, 1.0,
     0.0, 0.0
  };

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
    return uint64_t(floor(state[particle*3]/delta))*Nc + uint64_t(floor(state[particle*3+1]/delta));
  }

  void newTimeStepStates(double oldDt, double newDt);

  // GL private members
  void initialiseGL();
};

#endif
