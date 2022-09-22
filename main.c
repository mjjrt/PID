#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <complex.h>

#include "PID.h"

int main(int argc, char** argv)
{
    if (argc < 2 || argc > 5)
    {
        fprintf(stderr, "Usage: %s <Vr> <Tn> <Tv> <Tr1>\n", argv[0]);
        exit(1);
    }

    float initVr  = atof(argv[1]);
    float initTn  = atof(argv[2]);
    float initTv  = atof(argv[3]);
    float initTr1 = atof(argv[4]);

    PID pid;
    Plant plant;
    plant.state = 0;

    ResetController(&pid);

    InitController(&pid, initVr, initTn, initTv, initTr1, 1.0f);
    
#define MAX_CYCLES 1000
	int cycles = MAX_CYCLES;
    float deltat = 0.0001f;
    
	FILE* fp;
	fp = fopen("output.csv", "w");
	fprintf(fp, "Cycle, Time, Deviation, Controller Output, Output\n");
	while(cycles > 0)
    {
        /* printf("%f\n", FeedbackLoop(&pid, deltat)); */
        fprintf(fp, "%d, %.3f, %.3f, %.3f, %.3f\n", MAX_CYCLES - cycles, deltat*(MAX_CYCLES-cycles), pid.Deviation, pid.Output, FeedbackLoop(&pid, &plant, deltat));
		cycles--;
    }

	fclose(fp);
    return 0;
}
