#include <ParticleSystem/particleSystem.h>
#include <time.h>

void ParticleSystem::resetLists(){
  for (int i = 0; i < Nc*Nc; i++){
    cells[i] = NULL_INDEX;
  }
  for (int i = 0; i < nParticles; i++){
    list[i] = NULL_INDEX;
  }
}

void ParticleSystem::insert(uint64_t next, uint64_t particle){
  uint64_t i = next;
  while (list[i] != NULL_INDEX){
    i = list[i];                                                                  // someone here!
  }
  list[i] = particle;                                                             // it's free realestate
}

void ParticleSystem::populateLists(
){
  for (int i = 0; i < nParticles; i++){
    uint64_t c = hash(i);                                                    // flat index for the particle's cell
    if (cells[c] == NULL_INDEX){
      cells[c] = uint64_t(i);                                                     // we are the head!
    }
    else{
      insert(cells[c],uint64_t(i));
    }
  }
}

void ParticleSystem::handleCollision(uint64_t i, uint64_t j){
  if (i == j){return;}
  double rx,ry,dd,d,ddot,mag,fx,fy,nx,ny,vx,vy;
  rx = state[j*3]-state[i*3];
  ry = state[j*3+1]-state[i*3+1];
  dd = rx*rx+ry*ry;
  if (dd < 4.0*radius*radius){
    d = std::sqrt(dd);

    nx = rx / d;
    ny = ry / d;

    d = 2*radius-d;

    vx = velocities[i*2]-velocities[j*2];
    vy = velocities[i*2+1]-velocities[j*2+1];

    ddot = vx*nx+vy*ny;

    mag = -damping*ddot*std::pow(d,alpha)-restoration*std::pow(d,beta);

    fx = mag*nx;
    fy = mag*ny;

    forces[i*2] += fx;
    forces[i*2+1] += fy;

    forces[j*2] -= fx;
    forces[j*2+1] -= fy;
  }
}

// void ParticleSystem::handleCollision(uint64_t i, uint64_t j){
//   if (i == j){return;}
//   double rx,ry,dd,d,mag,fx,fy;
//   rx = state[j*3]-state[i*3];
//   ry = state[j*3+1]-state[i*3+1];
//   dd = rx*rx+ry*ry;
//   if (dd < 4.0*radius*radius){
//     d = std::sqrt(dd);
//     mag = restoration*(2.0*radius-d)/d;
//     fx = mag*rx;
//     fy = mag*ry;
//
//     forces[i*2] -= fx;
//     forces[i*2+1] -= fy;
//
//     forces[j*2] += fx;
//     forces[j*2+1] += fy;
//   }
// }

void ParticleSystem::cellCollisions(
  uint64_t a1,
  uint64_t b1,
  uint64_t a2,
  uint64_t b2
){
  if (a1 < 0 || a1 >= Nc || b1 < 0 || b1 >= Nc || a2 < 0 || a2 >= Nc || b2 < 0 || b2 >= Nc){
    return;                                                                      // not a cell
  }
  uint64_t p1 = cells[a1*Nc+b1];
  uint64_t p2 = cells[a2*Nc+b2];

  if (p1 == NULL_INDEX || p2 == NULL_INDEX){
    return;                                                                      // nobody here!
  }

  while (p1 != NULL_INDEX){
    p2 = cells[a2*Nc+b2];                                                        // flat index
    while(p2 != NULL_INDEX){
        handleCollision(p1,p2);                                                  // check this potential collision
        p2 = list[p2];                                                           // p2 points to a new particle in the same box (or null)
    }
    p1 = list[p1];                                                               // p1 points to a new particle in the same box (or null)
  }
}

void ParticleSystem::applyForce(double fx, double fy){
  for (int i = 0; i < nParticles; i++){
    forces[i*2] += fx;
    forces[i*2+1] += fy;
  }
}

void ParticleSystem::newTimeStepStates(double oldDt, double newDt){
  for (int i = 0; i < nParticles; i++){
    for (int k = 0; k < 3; k++){
      double delta = state[i*3+k]-lastState[i*3+k];
      lastState[i*3+k] = state[i*3+k]-(newDt/oldDt)*delta;
    }
  }
}

void ParticleSystem::step(){
  clock_t tic = clock();
  resetLists();
  populateLists();
  float setup = (clock()-tic)/float(CLOCKS_PER_SEC);
  tic = clock();
  for (int a = 0; a < Nc; a++){
    for (int b = 0; b < Nc; b++){
      cellCollisions(a,b,a,b);
      cellCollisions(a,b,a-1,b-1);
      cellCollisions(a,b,a-1,b+1);
      cellCollisions(a,b,a+1,b+1);
      cellCollisions(a,b,a+1,b-1);
      cellCollisions(a,b,a-1,b);
      cellCollisions(a,b,a+1,b);
      cellCollisions(a,b,a,b-1);
      cellCollisions(a,b,a,b+1);
    }
  }
  float col = (clock()-tic)/float(CLOCKS_PER_SEC);
  tic = clock();

  double D = std::sqrt(2.0*rotationalDiffusion/dt);
  double dtdt = dt*dt;

  double cr = (rotationalDrag*dt)/(2.0*momentOfInertia);
  double br = 1.0 / (1.0 + cr);
  double ar = (1.0-cr)*br;

  double ct = (drag*dt)/(2.0*mass);
  double bt = 1.0 / (1.0 + ct);
  double at = (1.0-ct)*bt;

  double shakerDisplacement = shakerAmplitude*std::cos(2.0*M_PI*shakerTime/shakerPeriod);

  glUseProgram(shakerShader);

  glUniform2f(
    glGetUniformLocation(shakerShader,"offsets"),
    shakerDisplacement+shakerAmplitude,
    shakerDisplacement-shakerAmplitude
  );

  for (int i = 0; i < nParticles; i++){

    noise[i*2+1] = noise[i*2];
    noise[i*2] = normal(generator);

    double x = state[i*3];
    double y = state[i*3+1];
    double theta = state[i*3+2];

    double xp = lastState[i*3];
    double yp = lastState[i*3+1];
    double thetap = lastState[i*3+2];

    // d = std::sqrt(dd);
    //
    // nx = rx / d;
    // ny = ry / d;
    //
    // d = 2*radius-d;
    //
    // vx = velocities[i*2]-velocities[j*2];
    // vy = velocities[i*2+1]-velocities[j*2+1];
    //
    // ddot = vx*nx+vy*ny;
    //
    // mag = -damping*ddot*std::pow(d,alpha)-restoration*std::pow(d,beta);

    if (x - radius <= (shakerDisplacement + shakerAmplitude)){
      double mag = restoration*((shakerDisplacement + shakerAmplitude)+radius-x);
      double f = std::abs(mag) - mag*damping*velocities[i*2];
      //forces[i*2] += f;
      // little kick, up, or else particles get stuck on the bottom and mash together
      //forces[i*2+1] += 0.1*std::abs(f);
    }

    if (x + radius >= 1.0+shakerDisplacement-shakerAmplitude){
      double mag = restoration*(x+radius-(1.0+shakerDisplacement-shakerAmplitude));
      double f = -std::abs(mag) + mag*damping*velocities[i*2];
      //forces[i*2] += f;
      // little kick, up, or else particles get stuck on the bottom and mash together
      //forces[i*2+1] += 0.1*std::abs(f);
    }

    double ax = drag*speed*cos(theta)+forces[i*2];
    double ay = drag*speed*sin(theta)+forces[i*2+1]-mass*9.81;

    //if(i==0){std::cout << std::setprecision(20) << (bt*dtdt/mass)*ay << ", " << 2.0*bt*y << ", " << at*yp << "\n";}
    state[i*3] = 2.0*bt*x - at*xp + (bt*dtdt/mass)*ax;
    state[i*3+1] = 2.0*bt*y - at*yp + (bt*dtdt/mass)*ay;
    state[i*3+2] = 2.0*br*theta - ar*thetap + (br*dt/(2.0*momentOfInertia))*(noise[i*2]+noise[i*2+1])*dt*rotationalDrag*D;

    lastState[i*3] = x;
    lastState[i*3+1] = y;
    lastState[i*3+2] = theta;

    double vx = state[i*3]-lastState[i*3];
    double vy = state[i*3+1]-lastState[i*3+1];

    velocities[i*2] = vx;
    velocities[i*2+1] = vy;

    double ux = 0.0; double uy = 0.0;
    double ang = state[i*3+2];
    bool flag = false;

    // kill the particles movement if it's outside the box
    if (state[i*3]-radius < 0 || state[i*3]+radius > 1.0){
      ux = -vx;
      ang = std::atan2(vy,ux);
      flag = true;
    }

    if (state[i*3+1]-radius < 0 || state[i*3+1]+radius > 1.0){
      uy = -1.5*vy;
      if (flag){
        ang = std::atan2(uy,ux);
      }
      else{
        ang = std::atan2(uy,vx);
        flag = true;
      }
    }

    if (flag){
      state[i*3+2] = ang;
      state[i*3+1] += uy;
      state[i*3] += ux;

      lastState[i*3+2] = ang;
      lastState[i*3+1] = state[i*3+1]-0.5*uy;
      lastState[i*3] = state[i*3]-0.5*ux;
    }

    if (state[i*3] == 1.0){ state[i*3] -= 0.001;}
    if (state[i*3+1] == 1.0){ state[i*3+1] -= 0.001;}
  }

  for (int i = 0; i < nParticles; i++){
    forces[i*2] = 0.0;
    forces[i*2+1] = 0.0;

    for (int k = 0; k < 3; k++){floatState[i*3+k] = float(state[i*3+k]);}
  }

  shakerTime += dt;

  double updates = (clock()-tic)/double(CLOCKS_PER_SEC);
  tic = clock();
}

void ParticleSystem::setProjection(glm::mat4 p){
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

void ParticleSystem::initialiseGL(){
  // a buffer of particle states
  glGenBuffers(1,&offsetVBO);
  glBindBuffer(GL_ARRAY_BUFFER,offsetVBO);
  glBufferData(GL_ARRAY_BUFFER,sizeof(float)*nParticles*3,&floatState[0],GL_DYNAMIC_DRAW);
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
  glVertexAttribPointer(1,3,GL_FLOAT,GL_FALSE,3*sizeof(float),(void*)0);
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
    0,0,0,1
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

void ParticleSystem::draw(
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
    resX*radius*2.0
  );

  glBindBuffer(GL_ARRAY_BUFFER,offsetVBO);
  glBufferSubData(GL_ARRAY_BUFFER,0,sizeof(float)*nParticles*3,&floatState[0]);
  glBindBuffer(GL_ARRAY_BUFFER,0);

  glError("particles buffers");

  glBindVertexArray(vertVAO);
  glDrawArraysInstanced(GL_POINTS,0,1,size());
  glBindVertexArray(0);

  glError("draw particles");

  glUseProgram(shakerShader);

  glBindVertexArray(shakerVAO);
  //glDrawArraysInstanced(GL_TRIANGLES,0,6,2);
  glBindVertexArray(0);

  glError("draw shaker");
}
