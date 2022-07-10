#include <stdio.h>
#include <stdlib.h>
#include <math.h>

typedef struct{
    float Vr;
    float Tn;
    float Tv;
    float Tr1;

    float Output;
}PID;

void ResetController(PID *pid)
{
    pid->Vr     = 0.0f;
    pid->Tn     = 0.0f;
    pid->Tv     = 0.0f;
    pid->Tr1    = 0.0f;

    pid->Output = 0.0f;
}


void InitController(PID *pid, float setVr, float setTn, float setTv, float setTr1)
{
    pid->Vr     = setVr;
    pid->Tn     = setTn;
    pid->Tv     = setTv;
    pid->Tr1    = setTr1;

    pid->Output = 0.0f;
}

void CalculateState();

int main(int argc, char** argv)
{
    if (argc < 2 || argc > 5)
    {
        fprintf(stderr, "Usage: %s <Vr> <Tn> <Tv> <Tr1>\n", argv[0]);
        exit(1);
    }

    float setVr  = atof(argv[1]);
    float setTn  = atof(argv[2]);
    float setTv  = atof(argv[3]);
    float setTr1 = atof(argv[4]);

    PID pid;
    ResetController(&pid);

    InitController(&pid, setVr, setTn, setTv, setTr1);
    
    int cycles = 0;
    while(cycles > 0)
    {
        
    }
    return 0;
}