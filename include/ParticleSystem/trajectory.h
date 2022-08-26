#ifndef TRAJECTORY_H
#define TRAJECTORY_H

struct State {
  float x;
  float y;
  float radius;
  time uint64_t;
}

class Trajectory {
public:
  Trajectory(){
  }

  void takeReading(ParticleSystem & p);
  void save(std::string file);

  void clear(){trajectory.clear();}

private:

  // timestep implicitly defined by use of takeReading()
  uint64_t currentFrameId;
  std::vector<State> trajectory;
}

#endif
