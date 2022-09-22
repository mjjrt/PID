// PID Controller Transfer Function
// G = Vr * (1 + 1/Tn*s + (Tv*s+1)/(Tr1s + 1))
typedef struct PID{
    float Vr;   // Common Controller Gain
    float Tn;   // Integral Time Constant
    float Tv;   // Derivative Time Constant
    float Tr1;  // Parasitic Time Constant

    float pidP; // Gain Output
    float pidI; // Integral Term Output
    float pidD; // Derivative Term Output

    float Deviation;    // Deviation from Setpoint
    float Setpoint;     // Setpoint 
    float Output;       // Controller Output
    float Feedback;     // Feedback from Sensor
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

    pid->pidP = 1.0f * e; // Gain Value
    pid->pidI = pid->pidI + e * Delta_T / pid->Tn; // Integral Value
    pid->pidD = (e - pid->Deviation) * (pid->Tv/pid->Tr1) + exp(- Delta_T / pid->Tr1) * pid->pidD; // Differentiator Value
    // Calculate the Output Value
    pid->Output = pid->Vr * (pid->pidP + pid->pidI + pid->pidD);

    // Update the values
    pid->Deviation = e;

    return pid->Output;

}

typedef struct Plant{
    float state;
}Plant;

float PlantUpdate(Plant *p, float input)
{
    float output = p->state + input*0.5f;
    p->state = output;
    return output;
}

float FeedbackLoop(PID *pid, Plant *p, float deltat)
{
    CalculateState(pid, deltat);
    float output = PlantUpdate(p, pid->Output);
    pid->Feedback = output;
    return output;
}