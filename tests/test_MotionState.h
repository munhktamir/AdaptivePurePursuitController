#include <check.h>
#include <stdio.h>
#include "../motion/Motion.h"


START_TEST(test_Extrapolate) {
    motionState_t currentState, nextState;
    double t, acc;

    currentState.t = 1.02;
    currentState.pos = 11.2;
    currentState.vel = 3.5;
    currentState.acc = 2.1;
    t = 1.025;
    acc = 2.0;

    nextState = Extrapolate(&currentState, t, acc);
    ck_assert_double_eq(nextState.t, t);
    ck_assert_double_eq(nextState.pos, 11.217525);
    ck_assert_double_eq(nextState.vel, 3.51);
    ck_assert_double_eq(nextState.acc, 2.0);

} END_TEST


START_TEST(test_NextTimeAtPos) {
    motionState_t currentState;
    double t;

    currentState.t = 1.5;
    currentState.pos = 7.5;
    currentState.vel = 2.0;

    // Already at position
    t = NextTimeAtPos(&currentState, 7.5);
    ck_assert_double_eq(currentState.t, t);
    
    // Zero acceleration - constant velocty towards position
    currentState.acc = 0.0;
    t = NextTimeAtPos(&currentState, 8.5);
    ck_assert_double_eq(2.0, t);

    // Zero acceleration - no velocity or going away from position
    t = NextTimeAtPos(&currentState, 6.5);
    ck_assert_double_nan(t);

    // Never reach position
    currentState.acc = -3.0;
    t = NextTimeAtPos(&currentState, 8.5);
    ck_assert_double_nan(t);

    // Add max time delta
    currentState.acc = 2.5;
    t = NextTimeAtPos(&currentState, 8.5);
    ck_assert_double_eq(1.9, t);

} END_TEST


START_TEST(test_FlippedState) {
    motionState_t currentState, flippedState;

    currentState.t = 1.5;
    currentState.pos = 7.5;
    currentState.vel = 2.0;
    currentState.acc = 3.0;

    // Flipped state
    flippedState = FlippedState(&currentState);
    ck_assert_double_eq(currentState.t, flippedState.t);
    ck_assert_double_eq(-currentState.pos, flippedState.pos);
    ck_assert_double_eq(-currentState.vel, flippedState.vel);
    ck_assert_double_eq(-currentState.acc, flippedState.acc);

} END_TEST


START_TEST(test_Coincident) {
    motionState_t currentState, flippedState;
    int equal;

    currentState.t = 1.5;
    currentState.pos = 7.5;
    currentState.vel = 2.0;
    currentState.acc = 3.0;
    flippedState = currentState;
    flippedState.acc = 10.0;
    equal = Coincident(&currentState, &flippedState);
    ck_assert_int_eq(1, equal);

} END_TEST


START_TEST(test_MotionStatesAreEqual) {
    motionState_t currentState, flippedState;
    int equal;

    currentState.t = 1.5;
    currentState.pos = 7.5;
    currentState.vel = 2.0;
    currentState.acc = 3.0;
    flippedState = currentState;
    flippedState.acc = currentState.acc;
    equal = MotionStatesAreEqual(&currentState, &flippedState);
    ck_assert_int_eq(1, equal);

} END_TEST


Suite *motionState_suite(void) {
    Suite *s;
    TCase *tc;

    s = suite_create("MotionState");
    tc = tcase_create("Core");

    tcase_add_test(tc, test_Extrapolate);
    tcase_add_test(tc, test_NextTimeAtPos);
    tcase_add_test(tc, test_FlippedState);
    tcase_add_test(tc, test_Coincident);
    tcase_add_test(tc, test_MotionStatesAreEqual);
    suite_add_tcase(s, tc);
    return s;
}

