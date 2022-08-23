#include <ParticleSystem/particleSystemRenderer.h>

void ParticleSystemRenderer::setProjection(glm::mat4 p){
  projection = p;
  glUseProgram(particleShader);
  glUniformMatrix4fv(
    glGetUniformLocation(particleShader,"proj"),
    1,
    GL_FALSE,
    &projection[0][0]
  );

  glUseProgram(shakerShader);
  glUniformMatrix4fv(
    glGetUniformLocation(shakerShader,"proj"),
    1,
    GL_FALSE,
    &projection[0][0]
  );
}

void ParticleSystemRenderer::initialiseGL(){
  // a buffer of particle states
  glGenBuffers(1,&offsetVBO);
  glBindBuffer(GL_ARRAY_BUFFER,offsetVBO);
  glBufferData(GL_ARRAY_BUFFER,sizeof(float)*nParticles*4,NULL,GL_DYNAMIC_DRAW);
  glBindBuffer(GL_ARRAY_BUFFER,0);

  // setup an array object
  glGenVertexArrays(1,&vertVAO);
  glGenBuffers(1,&vertVBO);
  glBindVertexArray(vertVAO);
  glBindBuffer(GL_ARRAY_BUFFER,vertVBO);
  glBufferData(GL_ARRAY_BUFFER,sizeof(vertices),vertices,GL_STATIC_DRAW);
  glEnableVertexAttribArray(0);
  // place dummy vertices for instanced particles
  glVertexAttribPointer(0,3,GL_FLOAT,GL_FALSE,3*sizeof(float),(void*)0);

  glEnableVertexAttribArray(1);
  glBindBuffer(GL_ARRAY_BUFFER, offsetVBO);
  // place states
  glVertexAttribPointer(1,4,GL_FLOAT,GL_FALSE,4*sizeof(float),(void*)0);
  glBindBuffer(GL_ARRAY_BUFFER,0);
  glVertexAttribDivisor(1,1);

  glError("initialised particles");

  particleShader = glCreateProgram();
  compileShader(particleShader,particleVertexShader,particleFragmentShader);
  glUseProgram(particleShader);

  glUniformMatrix4fv(
    glGetUniformLocation(particleShader,"proj"),
    1,
    GL_FALSE,
    &projection[0][0]
  );

  // shaker

  shakerShader = glCreateProgram();
  compileShader(shakerShader,shakerVertexShader,shakerFragmentShader);
  glUseProgram(shakerShader);

  glUniformMatrix4fv(
    glGetUniformLocation(shakerShader,"proj"),
    1,
    GL_FALSE,
    &projection[0][0]
  );

  glUniform4f(
    glGetUniformLocation(shakerShader,"u_colour"),
    0,0,0,0.33
  );

  glGenVertexArrays(1,&shakerVAO);
  glGenBuffers(1,&shakerVBO);
  glBindVertexArray(shakerVAO);
  glBindBuffer(GL_ARRAY_BUFFER,shakerVBO);
  glBufferData(GL_ARRAY_BUFFER,sizeof(float)*6*2,shakerVertices,GL_STATIC_DRAW);
  glEnableVertexAttribArray(0);
  glVertexAttribPointer(0,2,GL_FLOAT,GL_FALSE,2*sizeof(float),0);
  glBindBuffer(GL_ARRAY_BUFFER,0);
  glBindVertexArray(0);

  glError();
  glBufferStatus();
}

void ParticleSystemRenderer::draw(
  ParticleSystem & p,
  uint64_t frameId,
  float zoomLevel,
  float resX,
  float resY
){
  glUseProgram(particleShader);

  glUniform1f(
    glGetUniformLocation(particleShader,"zoom"),
    zoomLevel
  );

  glUniform1f(
    glGetUniformLocation(particleShader,"scale"),
    resX*2.0
  );

  glBindBuffer(GL_ARRAY_BUFFER,offsetVBO);
  glBufferSubData(GL_ARRAY_BUFFER,0,sizeof(float)*nParticles*4,&p.floatState[0]);
  glBindBuffer(GL_ARRAY_BUFFER,0);

  glError("particles buffers");

  glBindVertexArray(vertVAO);
  glDrawArraysInstanced(GL_POINTS,0,1,p.size());
  glBindVertexArray(0);

  glError("draw particles");

  glUseProgram(shakerShader);

  glUniform1f(
    glGetUniformLocation(shakerShader,"offset"),
    p.shakerDisplacement+p.shakerAmplitude
  );

  glBindVertexArray(shakerVAO);
  glDrawArraysInstanced(GL_TRIANGLES,0,6,2);
  glBindVertexArray(0);

  glError("draw shaker");
}
