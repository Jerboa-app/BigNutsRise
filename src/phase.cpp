/*

	This file demonstrates using MPI to scan trhough some parameters,
		to be honest without access to 100's of cpus you'll struggle to
		get much out of this.

		The following Julia code would parse the output file
		(e.g got from: mpirun -np 4 phase > output.txt)
		"output.txt" for you:

				nP = 20
				nA = 20
				nC = 10

				O = zeros(nP,nA,nC)

				d = readlines("output.txt");
				@show d[1]
				d = d[2:end]

				Ps = unique(map(x->parse(Float64,x[2]),split.(d,",")))
				As = unique(map(x->parse(Float64,x[3]),split.(d,",")))
				Cs = unique(map(x->parse(Float64,x[1]),split.(d,",")));

				pac = (y) -> (
				    parse(Float64,split(y,",")[2]),
				    parse(Float64,split(y,",")[3]),
				    parse(Float64,split(y,",")[1])
				)

				o = (y) -> parse(Float64,split(y,",")[4])

				index = (pac) -> (
				    findall(x->x.==pac[1],Ps)[1],
				    findall(x->x.==pac[2],As)[1],
				    findall(x->x.==pac[3],Cs)[1]
				)

				for l in d
				    O[index(pac(l))...] = o(l)
				end

*/
#include <mpi.h>
#include <iostream>
#include <sstream>
#include <iomanip>

#include <ParticleSystem/particleSystem.cpp>

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
	job(double p, double a, double c)
	: period(p), amplitude(a), cor(c)
	{}
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

    particles.randomise(0.5);
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
  }

  MPI_Finalize();
  return 0;
}
