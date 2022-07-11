#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <complex.h>

typedef struct PID{
    float Vr;
    float Tn;
    float Tv;
    float Tr1;

    float pidP;
    float pidI;
    float pidD;

    float Deviation;
    float Setpoint;
    float Output;
    float Feedback;
}PID;

float CalculateState(PID *pid, float Delta_T);

void ResetController(PID *pid)
{
    pid->Vr     = 0.0f;
    pid->Tn     = 0.0f;
    pid->Tv     = 0.0f;
    pid->Tr1    = 0.0f;

    pid->Setpoint = 0.0f;
    pid->Output   = 0.0f;
    pid->Feedback = 0.0f;
}


void InitController(PID *pid, float initVr, float initTn, float initTv, float initTr1, float setpt)
{
    pid->Vr     = initVr;
    pid->Tn     = initTn;
    pid->Tv     = initTv;
    pid->Tr1    = initTr1;

    pid->pidP   = 0.0f;
    pid->pidI   = 0.0f;
    pid->pidD   = 0.0f;

    pid->Setpoint = setpt;
    pid->Deviation = pid->Setpoint - CalculateState(pid, 0.01f);
    pid->Output   = 0.0f;
    pid->Feedback = 0.0f;
}

/*void SetController(PID *pid, float setVr, float setTn, float setTv, float setTr1, float setpt)
{
    pid->Vr     = setVr;
    pid->Tn     = setTn;
    pid->Tv     = setTv;
    pid->Tr1    = setTr1;

    pid->Setpoint = setpt;
    pid->Output = 0.0f;
}*/

float CalculateState(PID *pid, float Delta_T)
{
    // Calculate the difference between setpoint and plant state
    float e = pid->Setpoint - pid->Feedback;
    

    pid->pidI = pid->pidI + pid->Deviation * Delta_T / pid->Tn; // Integral Value
    pid->pidD = (e - pid->Deviation) * (pid->Tv/pid->Tr1) + exp(- Delta_T / pid->Tr1) *pid->pidD; // Differentiator Value
    pid->pidP = 1.0f * e; // Gain Value

    // Calculate the Output Value
    pid->Output = pid->Vr * (pid->pidP+ pid->pidI + pid->pidD);

    // Update the values
    pid->Deviation = e;
    pid->Feedback = pid->Output;

    return pid->Output;

}

float FeedbackLoop(PID *pid, float deltat)
{
    CalculateState(pid, deltat);
    return pid->Output;
}

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
    ResetController(&pid);

    InitController(&pid, initVr, initTn, initTv, initTr1, 1.0f);
    
#define MAX_CYCLES 100
	int cycles = MAX_CYCLES;
    float deltat = 0.01f;
    
	FILE* fp;
	fp = fopen("output.csv", "w");
	fprintf(fp, "Cycle, Output\n");
	while(cycles > 0)
    {
        /* printf("%f\n", FeedbackLoop(&pid, deltat)); */
        fprintf(fp, "%d, %.3f\n", MAX_CYCLES - cycles,  FeedbackLoop(&pid, deltat));
		cycles--;
    }

	fclose(fp);
    return 0;
}
