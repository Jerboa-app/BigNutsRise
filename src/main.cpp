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
#include <Text/textRenderer.cpp>
#include <Text/popup.cpp>

#include <Menu/slider.cpp>

#include <time.h>
#include <random>
#include <iostream>
#include <math.h>
#include <vector>

const int resX = 720;
const int resY = 720;

const int subSamples = 60;
const float dt = (1.0 / 60.0) / subSamples;

const int N = 1024;
// motion parameters

// for smoothing delta numbers
uint8_t frameId = 0;
double deltas[60];
double physDeltas[60];
double renderDeltas[60];

float speed = 1.0;

void speedPopUp(Popup & popups){
  popups.clear("speed");
  std::string pos = fixedLengthNumber(std::ceil(100.0*speed),3);
  popups.post(
    FadingText(
      "Speed: "+pos+" %",
      3.0,
      resX-256.,
      resY-64*6,
      glm::vec3(0.0,0.0,0.0),
      "speed"
    )
  );
}

int main(){

  for (int i = 0; i < 60; i++){deltas[i] = 0.0;}

  sf::ContextSettings contextSettings;
  contextSettings.depthBits = 24;
  contextSettings.antialiasingLevel = 0;

  sf::RenderWindow window(
    sf::VideoMode(resX,resY),
    "Particles",
    sf::Style::Close|sf::Style::Titlebar,
    contextSettings
  );
  window.setVerticalSyncEnabled(true);
  window.setFramerateLimit(60);
  window.setActive();

  glewInit();

  uint8_t debug = 0;

  ParticleSystem particles(N,dt);

  sf::Clock clock;
  sf::Clock physClock, renderClock;

  glm::mat4 defaultProj = glm::ortho(0.0,double(resX),0.0,double(resY),0.1,100.0);
  glm::mat4 textProj = glm::ortho(0.0,double(resX),0.0,double(resY));

  // for rendering particles using gl_point instances
  glEnable(GL_PROGRAM_POINT_SIZE);
  glEnable(GL_POINT_SPRITE);
  glEnable( GL_BLEND );
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  glDisable(GL_DEPTH_TEST);

  // for freetype rendering
  glPixelStorei(GL_UNPACK_ALIGNMENT, 1);

  // must be initialised before so the shader is in use..?
  TextRenderer textRenderer(textProj);

  Type OD("resources/fonts/","OpenDyslexic-Regular.otf",48);

  Popup popups;

  OrthoCam camera(resX,resY,glm::vec2(0.0,0.0));

  glViewport(0,0,resX,resY);

  Slider shakerSlider(resX-256.0,resY-64.0,128.0,16.0,"Shaker Period");
  shakerSlider.setPosition(0.5);
  shakerSlider.setProjection(textProj);

  Slider particlesSlider(resX-256.0,resY-64.0*2,128.0,16.0,"Particles");
  particlesSlider.setPosition(0.5);
  particlesSlider.setProjection(textProj);

  Slider proportionBigSlider(resX-256.0,resY-64.0*3,128.0,16.0,"Proportion Big");
  proportionBigSlider.setPosition(0.5);
  proportionBigSlider.setProjection(textProj);

  double oldMouseX = 0.0;
  double oldMouseY = 0.0;

  double mouseX = resX/2.0;
  double mouseY = resY/2.0;

  bool moving = false;

  bool pause = false;

  double shakerMaxPeriod = 2.0;
  double propBig = 0.5;

  while (window.isOpen()){


    sf::Event event;
    while (window.pollEvent(event)){
      if (event.type == sf::Event::Closed){
        return 0;
      }

      if (event.type == sf::Event::KeyPressed && event.key.code == sf::Keyboard::Escape){
        return 0;
      }

      if (event.type == sf::Event::KeyPressed && event.key.code == sf::Keyboard::F1){
        debug = !debug;
      }

      if (event.type == sf::Event::KeyPressed && event.key.code == sf::Keyboard::Space){
        pause = !pause;
      }


      if (event.type == sf::Event::KeyPressed && event.key.code == sf::Keyboard::W){
        speed *= 2.0;
        if (speed > 1){
          speed = 1;
        }
        speedPopUp(popups);
      }

      if (event.type == sf::Event::KeyPressed && event.key.code == sf::Keyboard::S){
        speed /= 2.0;
        if (speed < 0.01){
          speed = 0.01;
        }
        speedPopUp(popups);
      }


      if (event.type == sf::Event::MouseWheelScrolled){
        mouseX = event.mouseWheelScroll.x;
        mouseY = event.mouseWheelScroll.y;
        double z = event.mouseWheelScroll.delta;

        camera.incrementZoom(z);
      }

      if (event.type == sf::Event::MouseButtonPressed && event.mouseButton.button == sf::Mouse::Middle){
        mouseX = event.mouseButton.x;
        mouseY = event.mouseButton.y;

        glm::vec4 worldPos = camera.screenToWorld(mouseX,mouseY);

        camera.setPosition(worldPos.x,worldPos.y);
      }

      if (event.type == sf::Event::MouseButtonPressed && event.mouseButton.button == sf::Mouse::Left){
        sf::Vector2i pos = sf::Mouse::getPosition(window);
        bool c = shakerSlider.clicked(pos.x,resY-pos.y);
        particlesSlider.clicked(pos.x,resY-pos.y);
        proportionBigSlider.clicked(pos.x,resY-pos.y);
        // multiply by inverse of current projection
        glm::vec4 worldPos = camera.screenToWorld(pos.x,pos.y);

        oldMouseX = pos.x;
        oldMouseY = pos.y;
      }

      if (event.type == sf::Event::MouseMoved && sf::Mouse::isButtonPressed(sf::Mouse::Left)){
        sf::Vector2i pos = sf::Mouse::getPosition(window);
        shakerSlider.drag(pos.x,resY-pos.y);
        particlesSlider.drag(pos.x,resY-pos.y);
        proportionBigSlider.drag(pos.x,resY-pos.y);
      }

      if (event.type == sf::Event::MouseButtonReleased && event.mouseButton.button == sf::Mouse::Left){
        sf::Vector2i pos = sf::Mouse::getPosition(window);

        shakerSlider.mouseUp();
        particlesSlider.mouseUp();
        proportionBigSlider.mouseUp();
      }

    }

    window.clear(sf::Color::White);
    glClearColor(1.0f,1.0f,1.0f,1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    physClock.restart();

    if (!pause){

      int n = std::floor(particlesSlider.getPosition()*float(N));
      if (n < particles.size()){
        while (n < particles.size()){
          particles.removeParticle();
        }
      }
      else if (n > particles.size()){
        int i = particles.size()-1;
        while (i < n){
          particles.addParticle();
          i++;
        }
      }

      double val = proportionBigSlider.getPosition();

      if (val != propBig){
        particles.randomiseRadii(val);
        propBig = val;
      }

      particles.setShakerPeriod(std::max(0.5,double(shakerSlider.getPosition())*shakerMaxPeriod));
      particles.setTimeStep(dt*speed);
      for (int s = 0; s < subSamples; s++){
        particles.step();
      }
    }

    physDeltas[frameId] = physClock.getElapsedTime().asSeconds();

    renderClock.restart();

    glm::mat4 proj = camera.getVP();

    particles.setProjection(proj);
    particles.draw(
      frameId,
      camera.getZoomLevel(),
      resX,
      resY
    );

    if (debug){
      double delta = 0.0;
      double renderDelta = 0.0;
      double physDelta = 0.0;
      for (int n = 0; n < 60; n++){
        delta += deltas[n];
        renderDelta += renderDeltas[n];
        physDelta += physDeltas[n];
      }
      delta /= 60.0;
      renderDelta /= 60.0;
      physDelta /= 60.0;
      std::stringstream debugText;

      sf::Vector2i mouse = sf::Mouse::getPosition(window);

      float cameraX = camera.getPosition().x;
      float cameraY = camera.getPosition().y;

      debugText << "Particles: " << N <<
        "\n" <<
        "Delta: " << fixedLengthNumber(delta,6) <<
        " (FPS: " << fixedLengthNumber(1.0/delta,4) << ")" <<
        "\n" <<
        "Render/Physics: " << fixedLengthNumber(renderDelta,6) << "/" << fixedLengthNumber(physDelta,6) <<
        "\n" <<
        "Mouse (" << fixedLengthNumber(mouse.x,4) << "," << fixedLengthNumber(mouse.y,4) << ")" <<
        "\n" <<
        "Camera [world] (" << fixedLengthNumber(cameraX,4) << ", " << fixedLengthNumber(cameraY,4) << ")" <<
        "\n";
      textRenderer.renderText(
        OD,
        debugText.str(),
        64.0f,resY-64.0f,
        0.5f,
        glm::vec3(0.0f,0.0f,0.0f)
      );
    }

    shakerSlider.draw(
      textRenderer,
      OD
    );

    particlesSlider.draw(
      textRenderer,
      OD
    );

    proportionBigSlider.draw(
      textRenderer,
      OD
    );

    popups.draw(
      textRenderer,
      OD,
      clock.getElapsedTime().asSeconds()
    );

    if(pause){
      textRenderer.renderText(
        OD,
        "Space to resume",
        resX/3.,
        resY/2.,
        0.5,
        glm::vec3(0.,0.,0.),
        1.0
      );
    }

    window.display();

    deltas[frameId] = clock.getElapsedTime().asSeconds();
    renderDeltas[frameId] = renderClock.getElapsedTime().asSeconds();

    clock.restart();

    if (frameId == 60){
      frameId = 0;
    }
    else{
      frameId++;
    }
  }

  return 0;
}
