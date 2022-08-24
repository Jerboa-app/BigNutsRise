#ifndef PARTICLESYSTEMRENDERER_H
#define PARTICLESYSTEMRENDERER_H

class ParticleSystemRenderer {
public:
  ParticleSystemRenderer(int sizeHint)
  : nParticles(sizeHint)
  {
    initialiseGL();
  }

  void setProjection(glm::mat4 p);
  void draw(ParticleSystem & p, uint64_t frameId, float zoomLevel, float resX, float resY);

  ~ParticleSystemRenderer(){
    // kill some GL stuff
    glDeleteProgram(particleShader);
    glDeleteProgram(shakerShader);

    glDeleteBuffers(1,&offsetVBO);
    glDeleteBuffers(1,&vertVBO);
    glDeleteBuffers(1,&shakerVBO);

    glDeleteVertexArrays(1,&vertVAO);
    glDeleteVertexArrays(1,&shakerVAO);
  }

private:
  int nParticles;
  // GL data members
  float * floatState;
  GLuint particleShader, offsetVBO, vertVAO, vertVBO;
  glm::mat4 projection;

  float vertices[3] = {0.0,0.0,0.0};

  GLuint shakerShader, shakerVAO, shakerVBO;

  float shakerVertices[6*2] = {
     0.0, -1.0,
     0.0,  0.0,
     0.5,  0.0,
     0.0, -1.0,
     0.5, -1.0,
     0.5, 0.0
  };

  void initialiseGL();

};

#endif
