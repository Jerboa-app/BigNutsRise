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
  for (int i = 0; i < size(); i++){
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
  double rc = parameters[i*2]+parameters[j*2];
  if (dd < rc*rc){
    d = std::sqrt(dd);

    nx = rx / d;
    ny = ry / d;

    d = rc-d;

    vx = velocities[i*2]-velocities[j*2];
    vy = velocities[i*2+1]-velocities[j*2+1];

    ddot = vx*nx+vy*ny;

    double damping, restoration;

    if (parameters[i*2+1] == mass && parameters[j*2+1] == mass){
      damping = dampingBB; restoration = restorationBB;
    }
    else if (parameters[i*2+1] == mass && parameters[j*2+1] < mass){
      damping = dampingSB; restoration = restorationSB;
    }
    if (parameters[i*2+1] < mass && parameters[j*2+1] == mass){
      damping = dampingSB; restoration = restorationSB;
    }
    if (parameters[i*2+1] < mass && parameters[j*2+1] < mass){
      damping = dampingSS; restoration = restorationSS;
    }

    mag = -damping*ddot-restoration*std::pow(d,beta);

    fx = mag*nx;
    fy = mag*ny;

    forces[i*2] += fx;
    forces[i*2+1] += fy;

    forces[j*2] -= fx;
    forces[j*2+1] -= fy;
  }
}

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

double ParticleSystem::orderParameter(){
  double step = radius;
  int nl = 0;
  int ns = 0;

  for (int i = 0; i < size(); i++){
    if (parameters[i*2] == radius){
      nl += 1;
    }
    else{
      ns += 1;
    }
  }

  double mu = nl / float(nl+ns);
  double y = 0.0;
  double sigma = 0.0;
  double A = 0.0;
  while (y < Ly){
    int l = 0;
    int s = 0;
    for (int i = 0; i < size(); i++){
      if (state[i*3+1] >= y && state[i*3+1] <= y+step){
        if (parameters[i*2] == radius){
          l += 1;
        }
        else{
          s += 1;
        }
      }
    }

    if (l==0 && s==0){
      y+=step; continue;
    }

    double fi = l / float(l+s);
    double Ai = l+s;
    sigma += Ai*(fi-mu)*(fi-mu);
    A += Ai;
    y += step;
  }

  return (sigma/A)/mu;
}

void ParticleSystem::applyForce(double fx, double fy){
  for (int i = 0; i < size(); i++){
    forces[i*2] += fx;
    forces[i*2+1] += fy;
  }
}

void ParticleSystem::setCoeffientOfRestitution(double c){

  if (coefficientOfRestitution == c){
    return;
  }

  coefficientOfRestitution = c;

  dampingBB = damping(mass,mass);
  restorationBB = restoration(mass,mass);

  dampingSS = damping(mass/4.0,mass/4.0);
  restorationSS = restoration(mass/4.0,mass/4.0);

  dampingSB = damping(mass,mass/4.0);
  restorationSB = restoration(mass,mass/4.0);
}

void ParticleSystem::newTimeStepStates(double oldDt, double newDt){
  for (int i = 0; i < size(); i++){
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

  //std::cout << shakerPeriod << "\n";
  //shakerDisplacement += shakerAmplitude*2.0*M_PI/shakerPeriod * std::sin(2.0*M_PI*shakerTime/shakerPeriod)*dt;
  shakerDisplacement = shakerAmplitude*std::cos(2.0*M_PI*shakerTime/shakerPeriod);
  double xShakerAmplitude = 0.05*shakerAmplitude;
  double xShakerPeriod = 0.1*shakerPeriod;
  double xShakerDisplacement = xShakerAmplitude*std::cos(2.0*M_PI*shakerTime/(xShakerPeriod));

  for (int i = 0; i < size(); i++){

    double damping, restoration;

    if (parameters[i*2+1] == mass){
      damping = dampingBB; restoration = restorationBB;
    }
    else{
      damping = dampingSS; restoration = restorationSS;
    }

    double ct = (drag*dt)/(2.0*parameters[i*2+1]);
    double bt = 1.0 / (1.0 + ct);
    double at = (1.0-ct)*bt;

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

    // if (x - radius <= (xShakerDisplacement + xShakerAmplitude)){
    //   double mag = restoration*((xShakerDisplacement + xShakerAmplitude)+radius-x);
    //   double f = std::abs(mag); //- mag*damping*velocities[i*2];
    //   forces[i*2] += f;
    //   // little kick, up, or else particles get stuck on the bottom and mash together
    //   forces[i*2+1] += 0.1*std::abs(f);
    // }
    //
    // if (x + radius >= Lx+xShakerDisplacement-xShakerAmplitude){
    //   double mag = restoration*(x+radius-(Lx+xShakerDisplacement-xShakerAmplitude));
    //   double f = -std::abs(mag);// + mag*damping*velocities[i*2];
    //   forces[i*2] += f;
    //   // little kick, up, or else particles get stuck on the bottom and mash together
    //   forces[i*2+1] += 0.1*std::abs(f);
    // }

    if (y - parameters[2*i] <= (shakerDisplacement + shakerAmplitude)){
      double mag = (shakerDisplacement + shakerAmplitude)+parameters[2*i]-y;
      double f = 10*std::abs(mag)*restoration - damping*velocities[i*2+1];
      forces[i*2+1] += f;
    }

    double ax = drag*speed*cos(theta)+forces[i*2];
    double ay = drag*speed*sin(theta)+forces[i*2+1]-9.81*parameters[i*2+1];

    state[i*3] = 2.0*bt*x - at*xp + (bt*dtdt/parameters[i*2+1])*ax;
    state[i*3+1] = 2.0*bt*y - at*yp + (bt*dtdt/parameters[i*2+1])*ay;

    lastState[i*3] = x;
    lastState[i*3+1] = y;
    lastState[i*3+2] = theta;

    double vx = state[i*3]-lastState[i*3];
    double vy = state[i*3+1]-lastState[i*3+1];

    velocities[i*2] = vx/dt;
    velocities[i*2+1] = vy/dt;

    double ux = 0.0; double uy = 0.0;
    double newX = state[i*3]; double newY = state[i*3+1];
    double ang = state[i*3+2];
    bool flag = false;

    // kill the particles movement if it's outside the box
    if (state[i*3]-parameters[2*i] < 0 || state[i*3]+parameters[2*i] > Lx){
      ux = -0.5*vx;
      ang = std::atan2(vy,ux);

      if (state[i*3]-parameters[2*i] < 0){
        newX = parameters[2*i];
      }
      else{
        newX = Lx-parameters[2*i];
      }

      flag = true;
    }

    if (state[i*3+1]-parameters[2*i] < 0 || state[i*3+1]+parameters[2*i] > Ly){
      uy = -0.5*vy;
      if (flag){
        ang = std::atan2(uy,ux);
      }
      else{
        ang = std::atan2(uy,vx);
        flag = true;
      }
      if (state[i*3+1]-parameters[2*i] < 0){
        newY = parameters[2*i];
      }
      else{
        newY = Ly-parameters[2*i];
      }
    }

    if (flag){
      state[i*3+2] = ang;
      state[i*3+1] = newY+uy;
      state[i*3] = newX+ux;
    }

    if (state[i*3] == Lx){ state[i*3] -= 0.001;}
    if (state[i*3+1] == Ly){ state[i*3+1] -= 0.001;}

  }

  for (int i = 0; i < size(); i++){
    forces[i*2] = 0.0;
    forces[i*2+1] = 0.0;

    for (int k = 0; k < 3; k++){floatState[i*4+k] = float(state[i*3+k]);}
  }

  shakerTime += dt;

  double updates = (clock()-tic)/double(CLOCKS_PER_SEC);
  tic = clock();
}
