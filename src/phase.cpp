#include <mpi.h>
#include <iostream>
#include <sstream>
#include <iomanip>

#include "particleSystem.cpp"

#include <chrono>
#include <random>
#include <iostream>
#include <math.h>
#include <vector>

const double T = 60.0;
const int subSamples = 40;
const float dt = (1.0 / 60.0) / subSamples;

const int N = 1024;
// motion parameters

// for smoothing delta numbers
uint8_t frameId = 0;
double deltas[60];

float speed = 1.0;

double P = 1.0;
double A = 10.0;
double C = 0.5;

int nPs = 20;
int nAs = 20;
int nCs = 10;

struct job {
		double period;
		double amplitude;
		double cor;
};

int main(){

	// collect jobs
	std::vector<job> jobs;
	double pm = 0.005;
	double am = 0.0;
	double cm = 0.01;
	double dp = (P-pm)/nPs;
	double da = (A-am)/nAs;
	double dc = (C-cm)/nCs;
	for (int i = 0; i < nPs; i++){
		am = 0.0;
		for (int j = 0; j < nAs; j++){
			cm = 0.01;
			for (int k = 0; k < nCs; k++){
				jobs.push_back(job(pm,am,cm));
				cm += dc;
			}
			am += da;
		}
		pm += dp;
	}

  int rank, p;
  MPI_Init(NULL,NULL);
  MPI_Comm_rank(MPI_COMM_WORLD,&rank);
  MPI_Comm_size(MPI_COMM_WORLD,&p);

  MPI_Group worldGroup;
  MPI_Comm world;
  MPI_Comm_group(MPI_COMM_WORLD,&worldGroup);
  MPI_Comm_create(MPI_COMM_WORLD,worldGroup,&world);

  std::vector< std::vector<int> > elementsOnProcess(p);

	int i = 0;
	int proc = 0;
	while (i < jobs.size()){
		elementsOnProcess[proc].push_back(i);
		i++;
		proc++;
		if (proc >= p){
			proc = 0;
		}
	}

  for (int i = 0; i < 60; i++){deltas[i] = 0.0;}

  uint8_t debug = 0;

  if (rank == 0){
    std::cout << "Coef. Res. | Shaker Period | Shaker Amplitude | Order after " << T << " s\n";
  }

  for (int i = 0; i < elementsOnProcess[rank].size(); i++){

	ParticleSystem particles(N,dt);

    P = jobs[i].period;
    A = jobs[i].amplitude;
    C = jobs[i].cor;

    particles.randomiseRadii(0.5);
    particles.setShakerPeriod(P);
    particles.setShakerAmplitude(A);
    particles.setCoeffientOfRestitution(C);

    for (int k = 0; k < T/(dt*subSamples); k++){

      auto physClock = std::chrono::high_resolution_clock::now();

      for (int s = 0; s < subSamples; s++){
        particles.step();
      }

      deltas[frameId] = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::high_resolution_clock::now()-physClock
      ).count();

      if (frameId == 60){
        frameId = 0;
      }
      else{
        frameId++;
      }
    }
    std::cout << C << ", " << particles.getshakerPeriod() << ", " << particles.getShakerAmplitude() << ", " << particles.orderParameter() << "\n";
    // double m = 0.0;
    // for (int k = 0; k < 60; k++){
    //   m += deltas[k];
    // }
    // std::cout << "Frame time: " << m / 60.0 << " ms\n";
  }

  MPI_Finalize();
  return 0;
}
