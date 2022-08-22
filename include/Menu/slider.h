#ifndef SLIDER_H
#define SLIDER_H

#include <glm/glm.hpp>
#include <string>

class Slider {
public:

  Slider(float x, float y, float w, float h, std::string l)
  : xPosition(x), yPosition(y), width(w), height(h),
  position(0.0), label(l), dragging(false)
  {
    initialiseGL();
  }

  void initialiseGL();
  void draw(
    TextRenderer & text,
    Type & type
  );

  void setProjection(glm::mat4 p){projection=p;}

  float getPosition(){return position;}

  float setPosition(float p){
    if(p>1){position=1;}
    else if (p<0){position=0;}
    else{position=p;}
  }

  bool clicked(float x, float y);
  void drag(float x, float y);
  void mouseUp(){dragging = false; clickX = xPosition+position*width;}

private:

  std::string label;

  float position;

  float xPosition;
  float yPosition;
  float width;
  float height;

  glm::mat4 projection;

  float clickX;
  float clickY;
  bool dragging;

  GLuint sliderShader, sliderVAO, sliderVBO;

};

#endif
