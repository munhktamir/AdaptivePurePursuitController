#include <stdio.h>
#include "Motion.h"


int main() {
    motionState_t currentState, nextState;
    double t, acc;

    currentState.t = 1.02;
    currentState.pos = 11.2;
    currentState.vel = 3.5;
    currentState.acc = 2.1;

    t = 1.025;
    acc = 2.0;

    // Test Extrapolate
    nextState = Extrapolate(&currentState, t, acc);
    printf("NextState\n");
    printf("    %3s,%2.3f\n", "t", nextState.t);


    return 0;
}