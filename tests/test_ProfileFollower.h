#include <check.h>
#include "../motion/Motion.h"

START_TEST(test_SetProfileFollowerGains) {
    profileFollower_t *profileFollower;
    double kp, ki, kv, kffv, kffa;

    kp = 1.0;
    ki = 0.01;
    kv = 0.1;
    kffv = 1.5;
    kffa = 2.0;
    profileFollower = CreateProfileFollower();
    SetProfileFollowerGains(profileFollower, kp, ki, kv, kffv, kffa);
    ck_assert_ldouble_eq(kp, profileFollower->kP);
    ck_assert_ldouble_eq(ki, profileFollower->kI);
    ck_assert_ldouble_eq(kv, profileFollower->kV);
    ck_assert_ldouble_eq(kffv, profileFollower->kFFV);
    ck_assert_ldouble_eq(kffa, profileFollower->kFFA);

} END_TEST


START_TEST(test_SetProfileFollowerGoalAndConstraints) {
    profileFollower_t *profileFollower;
    motionProfileConstraints_t constraints;
    motionProfileGoal_t goalState;


    constraints.maxAbsAcc = 10.0;
    constraints.maxAbsVel = 5.0;
    goalState.completionBehavior = OVERSHOOT;//VIOLATE_MAX_ABS_VEL;
    goalState.maxAbsVel = 2.0;
    goalState.pos = 6.3;
    goalState.posTolerance = 0.05;
    goalState.velTolerance = 0.1;
    profileFollower = CreateProfileFollower();
    SetProfileFollowerGoalAndConstraints(profileFollower, &goalState, &constraints);
    ck_assert_ldouble_eq(2.0, profileFollower->goal->maxAbsVel);
    ck_assert_ldouble_eq(6.3, profileFollower->goal->pos);
    ck_assert_ldouble_eq(0.05, profileFollower->goal->posTolerance);
    ck_assert_ldouble_eq(0.1, profileFollower->goal->velTolerance);
    ck_assert_ldouble_eq(5.0, profileFollower->constraints->maxAbsVel);
    ck_assert_ldouble_eq(10.0, profileFollower->constraints->maxAbsAcc);

} END_TEST


START_TEST(test_ClearProfileFollower) {
    profileFollower_t *profileFollower;

    profileFollower = CreateProfileFollower();

    ClearProfileFollower(profileFollower);
    ck_assert_ptr_null(profileFollower->constraints);
    ck_assert_ptr_null(profileFollower->goal);
    ck_assert_ptr_null(profileFollower->latestSetpoint);
    ck_assert_ptr_null(profileFollower->setpointGenerator->constraints);
    ck_assert_ptr_null(profileFollower->setpointGenerator->goal);


} END_TEST


START_TEST(test_ProfileFollowerUpdate) {
    // Need a something more than a simple unit test
} END_TEST


START_TEST(test_IsProfileFinished) {
    profileFollower_t *profileFollower;
    int finished;
    
    profileFollower = CreateProfileFollower();

    // Not finished
    finished = IsProfileFinished( profileFollower );
    ck_assert_int_eq(0, finished);

    // Finished
    profileFollower->latestSetpoint->finalSetpoint = 1;
    finished = IsProfileFinished( profileFollower );
    ck_assert_int_eq(1, finished);


} END_TEST


START_TEST(test_IsProfileOnTarget) {
    profileFollower_t *profileFollower;
    int onTarget;
    
    profileFollower = CreateProfileFollower();

    // On-target
    profileFollower->goal->pos = 10.0;
    profileFollower->goal->maxAbsVel = 5.0;
    profileFollower->goal->velTolerance = 0.1;
    profileFollower->goal->posTolerance = 0.25;
    profileFollower->initialState.pos = 9.0;
    profileFollower->latestActualState.pos = 9.99;
    profileFollower->latestActualState.vel = 0.5;
    onTarget = IsProfileOnTarget( profileFollower );
    ck_assert_int_eq(1, onTarget);

    // Not on-target
    profileFollower->latestActualState.vel = 9.5;
    onTarget = IsProfileOnTarget( profileFollower );
    ck_assert_int_eq(0, onTarget);


} END_TEST



Suite *profileFollower_suite(void) {
    Suite *s;
    TCase *tc;

    s = suite_create("ProfileFollower");
    tc = tcase_create("Core");

    tcase_add_test(tc, test_SetProfileFollowerGains);
    tcase_add_test(tc, test_SetProfileFollowerGoalAndConstraints);
    tcase_add_test(tc, test_ClearProfileFollower);
    tcase_add_test(tc, test_ProfileFollowerUpdate);
    tcase_add_test(tc, test_IsProfileFinished);
    tcase_add_test(tc, test_IsProfileOnTarget);
    suite_add_tcase(s, tc);
    return s;
}

