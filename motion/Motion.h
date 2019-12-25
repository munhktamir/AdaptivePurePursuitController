#ifndef MOTION_H
#define MOTION_H
#include <math.h>
#include <stdlib.h>

typedef struct motionState {
    double t;
    double pos;
    double vel;
    double acc;
} motionState_t;

static const motionState_t kInvalidMotionState = {NAN, NAN, NAN, NAN};

// OVERSHOOT
// Overshoot the goal if necessary (at a velocity greater than max_abs_vel) and come back. Only valid if the goal velocity is
// 0.0 (otherwise VIOLATE_MAX_ACCEL will be used).
//
// VIOLATE_MAX_ACCEL
// If we cannot slow down to the goal velocity before crossing the goal, allow exceeding the max accel constraint.
//
// VIOLATE_MAX_ABS_VEL
// If we cannot slow down to the goal velocity before crossing the goal, allow exceeding the goal velocity.
enum completionBehavior_e {OVERSHOOT, VIOLATE_MAX_ACCEL, VIOLATE_MAX_ABS_VEL, INVALID};

typedef struct motionProfileGoal {
    double pos;
    double maxAbsVel;
    enum completionBehavior_e completionBehavior;
    double posTolerance;  //1e-3
    double velTolerance;  //1e-2
} motionProfileGoal_t;

static const motionProfileGoal_t kInvalidMotionProfileGoal = {NAN, NAN, INVALID, NAN, NAN};

typedef struct motionProfileConstraints {
    double maxAbsVel;
    double maxAbsAcc;
} motionProfileConstraints_t;

static const motionProfileConstraints_t kInvalidMotionProfileConstraints = {NAN, NAN};

typedef struct motionSegment {
    motionState_t start;
    motionState_t end;
} motionSegment_t;

typedef struct motionProfileNode {
    motionSegment_t segment;
    struct motionProfileNode *next;
} motionProfileNode_t;

typedef struct motionProfileList {
    motionProfileNode_t *head;
    motionProfileNode_t *tail;
    int length;
} motionProfileList_t;

typedef struct setpoint {
    motionState_t motionState;
    int finalSetpoint;
} setpoint_t;

static const setpoint_t kInvalidSetpoint = {{NAN, NAN, NAN, NAN}, 0};

typedef struct setpointGenerator {
    motionProfileList_t *profile;
    motionProfileGoal_t *goal;
    motionProfileConstraints_t *constraints;
} setpointGenerator_t;

static const setpointGenerator_t kInvalidSetpointGenerator = {NULL, NULL, NULL};

typedef struct profileFollower {
    double kP;
    double kI;
    double kV;
    double kFFV;
    double kFFA;
    double minOutput;
    double maxOutput;
    motionState_t latestActualState;
    motionState_t initialState;
    double latestPosError;
    double latestVelError;
    double totalError;
    motionProfileGoal_t *goal;
    motionProfileConstraints_t *constraints;
    setpointGenerator_t *setpointGenerator;
    setpoint_t *latestSetpoint;
} profileFollower_t;

// MotionState.c
motionState_t Extrapolate (motionState_t *state, double t, double acc);
double NextTimeAtPos (motionState_t *state, double pos);
motionState_t FlippedState (motionState_t *state);
int Coincident (motionState_t *a, motionState_t *b);
int MotionStatesAreEqual (motionState_t *stateA, motionState_t *stateB);

// MotionSegment.c
int IsSegmentValid (motionSegment_t *segment);
int ContainsTime (motionSegment_t *segment, double t);
int ContainsPosition (motionSegment_t *segment, double pos);

//MotionProfileGoal.c
motionProfileGoal_t FlippedGoal (motionProfileGoal_t *goal);
int AtGoalState (motionProfileGoal_t *goal, motionState_t *state);
int AtGoalPosition (motionProfileGoal_t *goal, double pos);
int GoalsAreEqual (motionProfileGoal_t *goalA, motionProfileGoal_t *goalB);
int ConstraintsAreEqual (motionProfileConstraints_t *constraintsA, motionProfileConstraints_t *constraintsB);

//MotionProfile.c
void PrintProfile (motionProfileList_t *profile);
int IsProfileValid (motionProfileList_t *profile);
motionState_t StateByTime (motionProfileList_t *profile, double t);
motionState_t StateByTimeClamped (motionProfileList_t *profile, double t);
motionState_t FirstStateByPosition (motionProfileList_t *profile, double pos);
void TrimBeforeTime(motionProfileList_t *profile, double t);
void ClearProfile (motionProfileList_t *profile);
void ResetProfile (motionProfileList_t *profile, motionState_t *initialState);
void AppendSegment (motionProfileList_t *profile, motionSegment_t *segment);
void AppendControl (motionProfileList_t *profile, double acc, double dt);
void Consolidate (motionProfileList_t *profile);
void AppendProfile (motionProfileList_t *currentProfile, motionProfileList_t *addProfile);

// MotionProfileGenerator.c
motionProfileList_t GenerateFlippedProfile (motionProfileConstraints_t *constraints, motionProfileGoal_t *goalState, motionState_t *prevState);
motionProfileList_t GenerateProfile (motionProfileConstraints_t *constraints, motionProfileGoal_t *goalState, motionState_t *prevState);

// SetpointGenerator.c
void ClearSetpointGenerator (setpointGenerator_t *setpointGenerator);
void SetSetpointGenerator (setpointGenerator_t *setpointGenerator, motionProfileConstraints_t *constraints, motionProfileGoal_t *goal, motionState_t *prevState);
setpoint_t GetSetpoint (setpointGenerator_t *setpointGenerator, motionProfileConstraints_t *constraints, motionProfileGoal_t *goal, motionState_t *prevState, double t);

// ProfileFollower.c
profileFollower_t * CreateProfileFollower ();
void ClearSetpointGenerator (setpointGenerator_t *setpointGenerator);
void SetProfileFollowerGains (profileFollower_t *profileFollower, double kp, double ki, double kv, double kffv, double kffa);
void SetProfileFollowerGoalAndConstraints (profileFollower_t *profileFollower, motionProfileGoal_t *goal, motionProfileConstraints_t *constraints);
void ClearProfileFollower (profileFollower_t *profileFollower);
double ProfileFollowerUpdate (profileFollower_t *profileFollower, motionState_t *latestState, double t);
int IsProfileFinished (profileFollower_t *profileFollower);
int IsProfileOnTarget (profileFollower_t *profileFollower);
motionState_t GetProfileSetpoint (profileFollower_t *profileFollower);

#endif