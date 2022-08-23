#include <iostream>
#include <sstream>
#include <iomanip>
#include <glew.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>

#include <SFML/Window.hpp>
#include <SFML/Graphics.hpp>
#include <SFML/OpenGL.hpp>

#include <orthoCam.h>

#include <glUtils.h>
#include <utils.h>
#include <shaders.h>

#include <ParticleSystem/particleSystem.cpp>

#include <time.h>
#include <random>
#include <iostream>
#include <math.h>
#include <vector>

const int subSamples = 40;
const float dt = (1.0 / 60.0) / subSamples;

const int N = 1024;
// motion parameters

// for smoothing delta numbers
uint8_t frameId = 0;
double deltas[60];
double physDeltas[60];
double renderDeltas[60];

float speed = 1.0;

int main(){

  for (int i = 0; i < 60; i++){deltas[i] = 0.0;}

  glewInit();

  uint8_t debug = 0;

  ParticleSystem particles(N,dt);

  sf::Clock clock;
  sf::Clock physClock, renderClock;

  // for rendering particles using gl_point instances
  glEnable(GL_PROGRAM_POINT_SIZE);
  glEnable(GL_POINT_SPRITE);
  glEnable( GL_BLEND );
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  glDisable(GL_DEPTH_TEST);

  double shakerMaxPeriod = 1.0;
  double propBig = 0.5;
  double maxAmplitude = 10.0; // measured in particle radius units!

  physClock.restart();

  for (int s = 0; s < subSamples; s++){
    particles.step();
  }


  physDeltas[frameId] = physClock.getElapsedTime().asSeconds();


  deltas[frameId] = clock.getElapsedTime().asSeconds();

  clock.restart();

  if (frameId == 60){
    frameId = 0;
  }
  else{
    frameId++;
  }

  return 0;
}
