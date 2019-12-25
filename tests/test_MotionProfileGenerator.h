#include <check.h>
#include "../motion/Motion.h"


START_TEST(test_GenerateProfile) {
    motionProfileConstraints_t constraints;
    motionProfileGoal_t goalState;
    motionState_t prevState;
    motionProfileList_t profile;

    constraints.maxAbsAcc = 10.0;
    constraints.maxAbsVel = 5.0;
    
    goalState.completionBehavior = OVERSHOOT;//VIOLATE_MAX_ABS_VEL;
    goalState.maxAbsVel = 0.0;
    goalState.pos = 6.3;
    goalState.posTolerance = 0.05;
    goalState.velTolerance = 0.1;
    
    // Full trapezoidal profile
    prevState.t = 0.0;
    prevState.pos = 0.0;
    prevState.vel = 0.0;
    prevState.acc = 0.0;
    profile = GenerateProfile(&constraints, &goalState, &prevState);
    PrintProfile(&profile);

    // Already at cruise
    prevState.t = 1.0;
    prevState.pos = 2.0;
    prevState.vel = 5.0;
    prevState.acc = 0.0;
    profile = GenerateProfile(&constraints, &goalState, &prevState);
    PrintProfile(&profile);

    // Require flipped
    goalState.pos = -6.3;
    prevState.t = 0.0;
    prevState.pos = 0.0;
    prevState.vel = 0.0;
    prevState.acc = 0.0;
    profile = GenerateProfile(&constraints, &goalState, &prevState);
    PrintProfile(&profile);



} END_TEST


Suite *motionProfileGenerator_suite(void) {
    Suite *s;
    TCase *tc;

    s = suite_create("MotionProfileGenerator");
    tc = tcase_create("Core");

    tcase_add_test(tc, test_GenerateProfile);
    suite_add_tcase(s, tc);
    return s;
}

