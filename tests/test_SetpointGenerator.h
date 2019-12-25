#include <check.h>
#include <stdio.h>
#include <stdlib.h>
#include "../motion/Motion.h"


START_TEST(test_ClearSetpointGenerator) {
    motionProfileConstraints_t *constraints;
    motionProfileGoal_t *goalState;
    motionState_t prevState;
    motionProfileList_t *profile;
    setpointGenerator_t setpointGenerator;

    constraints = (motionProfileConstraints_t *) malloc( sizeof ( motionProfileConstraints_t ) );
    goalState = (motionProfileGoal_t *) malloc( sizeof ( motionProfileGoal_t ) );
    profile = (motionProfileList_t *) malloc( sizeof ( motionProfileList_t ) );

    constraints->maxAbsAcc = 10.0;
    constraints->maxAbsVel = 5.0;
    
    goalState->completionBehavior = OVERSHOOT;//VIOLATE_MAX_ABS_VEL;
    goalState->maxAbsVel = 0.0;
    goalState->pos = 6.3;
    goalState->posTolerance = 0.05;
    goalState->velTolerance = 0.1;
    
    prevState.t = 0.0;
    prevState.pos = 0.0;
    prevState.vel = 0.0;
    prevState.acc = 0.0;

    (*profile) = GenerateProfile(constraints, goalState, &prevState);

    setpointGenerator.constraints = constraints;
    setpointGenerator.goal = goalState;
    setpointGenerator.profile = profile;

    ClearSetpointGenerator(&setpointGenerator);
    ck_assert_ptr_null(setpointGenerator.constraints);
    ck_assert_ptr_null(setpointGenerator.goal);
    ck_assert_ptr_null(setpointGenerator.profile);

 
 } END_TEST


START_TEST(test_SetSetpointGenerator) {
    setpointGenerator_t setpointGenerator;
    motionProfileConstraints_t constraints;
    motionProfileGoal_t goalState;
    motionState_t prevState;
    

    // constraints = (motionProfileConstraints_t *) malloc( sizeof ( motionProfileConstraints_t ) );
    // goalState = (motionProfileGoal_t *) malloc( sizeof ( motionProfileGoal_t ) );
    // prevState = (motionState_t *) malloc( sizeof ( motionState_t ) );
    constraints.maxAbsAcc = 10.0;
    constraints.maxAbsVel = 5.0;
    goalState.completionBehavior = OVERSHOOT;//VIOLATE_MAX_ABS_VEL;
    goalState.maxAbsVel = 0.0;
    goalState.pos = 6.3;
    goalState.posTolerance = 0.05;
    goalState.velTolerance = 0.1;
    prevState.t = 0.0;
    prevState.pos = 0.0;
    prevState.vel = 0.0;
    prevState.acc = 0.0;

    SetSetpointGenerator(&setpointGenerator, &constraints, &goalState, &prevState);
    ck_assert_double_eq(10.0, setpointGenerator.constraints->maxAbsAcc);
    ck_assert_double_eq(5.0, setpointGenerator.constraints->maxAbsVel);
    ck_assert_double_eq(0.0, setpointGenerator.goal->maxAbsVel);
    ck_assert_double_eq(6.3, setpointGenerator.goal->pos);
    ck_assert_double_eq(0.05, setpointGenerator.goal->posTolerance);
    ck_assert_double_eq(0.1, setpointGenerator.goal->velTolerance);

 
 } END_TEST


START_TEST(test_GetSetpoint) {
    setpointGenerator_t setpointGenerator;
    motionProfileConstraints_t constraints;
    motionProfileGoal_t goalState;
    motionState_t prevState;
    setpoint_t setpoint;

    constraints.maxAbsAcc = 10.0;
    constraints.maxAbsVel = 5.0;
    goalState.completionBehavior = OVERSHOOT;//VIOLATE_MAX_ABS_VEL;
    goalState.maxAbsVel = 0.0;
    goalState.pos = 6.3;
    goalState.posTolerance = 0.05;
    goalState.velTolerance = 0.1;
    prevState.t = 0.0;
    prevState.pos = 0.0;
    prevState.vel = 0.0;
    prevState.acc = 0.0;
    SetSetpointGenerator(&setpointGenerator, &constraints, &goalState, &prevState);

    // No regenerate
    setpoint = GetSetpoint(&setpointGenerator, &constraints, &goalState, &prevState, 0.1);
    PrintProfile(setpointGenerator.profile);
    ck_assert_int_eq(0, setpoint.finalSetpoint);
    ck_assert_ldouble_eq(0.1, setpoint.motionState.t);
    ck_assert_ldouble_eq(0.05, setpoint.motionState.pos);
    ck_assert_ldouble_eq(1.0, setpoint.motionState.vel);
    ck_assert_ldouble_eq(10.0, setpoint.motionState.acc);

 
 } END_TEST


Suite *setpointGenerator_suite(void) {
    Suite *s;
    TCase *tc;

    s = suite_create("SetpointGenerator");
    tc = tcase_create("Core");

    tcase_add_test(tc, test_ClearSetpointGenerator);
    tcase_add_test(tc, test_SetSetpointGenerator);
    tcase_add_test(tc, test_GetSetpoint);
    suite_add_tcase(s, tc);
    return s;
}

