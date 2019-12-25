#include <check.h>
#include <stdio.h>
#include "../motion/Motion.h"


START_TEST(test_FlippedGoal) {
    motionProfileGoal_t goal, flippedGoal;

    goal.completionBehavior = OVERSHOOT;
    goal.maxAbsVel = 8.5;
    goal.pos = 7.1;
    goal.posTolerance = 0.05;
    goal.velTolerance = 0.1;
    flippedGoal = FlippedGoal(&goal);
    ck_assert_double_eq(-goal.pos, flippedGoal.pos);
    ck_assert_double_eq(goal.maxAbsVel, flippedGoal.maxAbsVel);
    ck_assert_double_eq(goal.posTolerance, flippedGoal.posTolerance);
    ck_assert_double_eq(goal.velTolerance, flippedGoal.velTolerance);

} END_TEST


START_TEST(test_AtGoalState) {
    motionProfileGoal_t goal;
    motionState_t state;
    int done;

    goal.completionBehavior = OVERSHOOT;
    goal.maxAbsVel = 8.5;
    goal.pos = 7.1;
    goal.posTolerance = 0.05;
    goal.velTolerance = 0.1;
    state.t = 9.5;
    state.pos = 7.1;
    state.vel = 0.5;
    state.acc = -2.0;

    // Done
    done = AtGoalState(&goal, &state);
    ck_assert_int_eq(1, done);

    // Not done
    state.vel = 10.5;
    done = AtGoalState(&goal, &state);
    ck_assert_int_eq(0, done);

} END_TEST


START_TEST(test_AtGoalPosition) {
    motionProfileGoal_t goal;
    int done;

    goal.completionBehavior = OVERSHOOT;
    goal.maxAbsVel = 8.5;
    goal.pos = 7.1;
    goal.posTolerance = 0.05;
    goal.velTolerance = 0.1;

    // Done
    done = AtGoalPosition(&goal, 7.1);
    ck_assert_int_eq(1, done);

    // Not done
    done = AtGoalPosition(&goal, 5.2);
    ck_assert_int_eq(0, done);

} END_TEST


START_TEST(test_GoalsAreEqual) {
    motionProfileGoal_t goalA, goalB;
    int equal;

    goalA.completionBehavior = OVERSHOOT;
    goalA.maxAbsVel = 8.5;
    goalA.pos = 7.1;
    goalA.posTolerance = 0.05;
    goalA.velTolerance = 0.1;
    goalB.completionBehavior = OVERSHOOT;
    goalB.maxAbsVel = 8.5;
    goalB.pos = 7.1;
    goalB.posTolerance = 0.05;
    goalB.velTolerance = 0.1;

    // Equal
    equal = GoalsAreEqual(&goalA, &goalB);
    ck_assert_int_eq(1, equal);

    // Not equal
    goalB.posTolerance = 0.1;
    equal = GoalsAreEqual(&goalA, &goalB);
    ck_assert_int_eq(0, equal);


} END_TEST


START_TEST(test_ConstraintsAreEqual) {
    motionProfileConstraints_t constraintsA, constraintsB;
    int equal;

    constraintsA.maxAbsVel = 10.2;
    constraintsA.maxAbsAcc = 13.3;
    constraintsB.maxAbsVel = 10.2;
    constraintsB.maxAbsAcc = 13.3;

    // Equal
    equal = ConstraintsAreEqual(&constraintsA, &constraintsB);
    ck_assert_int_eq(1, equal);

    // Not equal
    constraintsB.maxAbsVel = 1.2;
    equal = ConstraintsAreEqual(&constraintsA, &constraintsB);
    ck_assert_int_eq(0, equal);


} END_TEST


Suite *motionProfileGoal_suite(void) {
    Suite *s;
    TCase *tc;

    s = suite_create("MotionProfileGoal");
    tc = tcase_create("Core");

    tcase_add_test(tc, test_FlippedGoal);
    tcase_add_test(tc, test_AtGoalState);
    tcase_add_test(tc, test_AtGoalPosition);
    tcase_add_test(tc, test_GoalsAreEqual);
    tcase_add_test(tc, test_ConstraintsAreEqual);
    suite_add_tcase(s, tc);
    return s;
}

