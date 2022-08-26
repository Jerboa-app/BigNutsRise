#ifndef BUTTON_H
#define BUTTON_H

class Button {
public:
  Button(float x, float y, float w, float h, std::string l)
  : xPosition(x), yPosition(y), width(w), height(h),
   label(l)
  {
    initialiseGL();
  }

  void initialiseGL();
  void draw(
    TextRenderer & text,
    Type & type
  );

  void setProjection(glm::mat4 p){projection=p;}

  bool getState(){return set;}

  void setState(bool p){
    set = p;
  }

  void setLabel(std::string s){label=s;}

  bool clicked(float x, float y);

private:

  std::string label;

  bool set;

  float xPosition;
  float yPosition;
  float width;
  float height;

  glm::mat4 projection;

  float clickX;
  float clickY;

  GLuint buttonShader, buttonVAO, buttonVBO;

};

#endif
