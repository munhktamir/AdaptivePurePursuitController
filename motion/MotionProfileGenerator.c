#include <math.h>
#include <stdlib.h>
#include "Motion.h"
#include "../utils/Utils.h"
#include "../robot/RobotMap.h"


/******************************************************************************************************************************** 
**  GenerateFlippedProfile
**
**      Input:
**
**      Output:
**
********************************************************************************************************************************/
motionProfileList_t GenerateFlippedProfile (motionProfileConstraints_t *constraints, motionProfileGoal_t *goalState, motionState_t *prevState) {
    motionProfileList_t flippedProfile;
    motionProfileGoal_t flippedGoal;
    motionState_t flippedState;
    motionProfileNode_t *profileNode;

    flippedGoal = FlippedGoal(goalState);
    flippedState = FlippedState(prevState);
    flippedProfile = GenerateProfile( constraints, &flippedGoal, &flippedState);

    profileNode = flippedProfile.head;
    while ( profileNode ) {
        profileNode->segment.start = FlippedState(&profileNode->segment.start);
        profileNode->segment.end = FlippedState(&profileNode->segment.end);
        profileNode = profileNode->next;
    }

    return flippedProfile;
}


/******************************************************************************************************************************** 
**  GenerateProfile
**
**      Input:
**
**      Output:
**
********************************************************************************************************************************/
motionProfileList_t GenerateProfile (motionProfileConstraints_t *constraints, motionProfileGoal_t *goalState, motionState_t *prevState) {
    motionProfileList_t profile, flippedProfile;
    motionState_t startState;
    motionSegment_t segment;
    double deltaPos, stoppingTime, minAbsVelAtGoalSqr, minAbsVelAtGoal, maxAbsVelAtGoal, goalVel, maxAcc, vMax, accelTime, distanceDecel, distanceCruise, cruiseTime, decelTime;

    profile.head = NULL;
    profile.tail = NULL;
    profile.length = 0;
    flippedProfile.head = NULL;
    flippedProfile.tail = NULL;
    flippedProfile.length = 0;

    deltaPos = goalState->pos - prevState->pos;
    if ( deltaPos < 0.0 || ( deltaPos == 0.0 && prevState->vel < 0.0) ) {
        // For simplicity, we always assume the goal requires positive movement. If negative, we flip to solve, then
        // flip the solution.
        flippedProfile = GenerateFlippedProfile(constraints, goalState, prevState);
        return flippedProfile;
    }

    // Invariant from this point on: delta_pos >= 0.0.  Clamp the start state to be valid.
    startState.t = prevState->t;
    startState.pos = prevState->pos;
    startState.vel = SignNum( prevState->vel ) * fmin( fabs ( prevState->vel ), constraints->maxAbsVel );
    startState.acc = SignNum( prevState->acc ) * fmin( fabs ( prevState->acc ), constraints->maxAbsAcc );
    
    ResetProfile(&profile, &startState);

    // If our velocity is headed away from the goal, the first thing we need to do is to stop.
    if (startState.vel < 0.0 && deltaPos > 0.0) {
        stoppingTime = fabs( startState.vel / constraints->maxAbsAcc );
        AppendControl( &profile, constraints->maxAbsAcc, stoppingTime );
        startState = profile.tail->segment.end;
        deltaPos = goalState->pos - startState.pos;
    }

    // Invariant from this point on: start_state.vel() >= 0.0
    minAbsVelAtGoalSqr = startState.vel * startState.vel - 2.0 * constraints->maxAbsAcc * deltaPos;
    minAbsVelAtGoal = sqrt( fabs( minAbsVelAtGoalSqr ) );
    maxAbsVelAtGoal = sqrt( startState.vel * startState.vel + 2.0 * constraints->maxAbsAcc * deltaPos);
    goalVel = goalState->maxAbsVel;
    maxAcc = constraints->maxAbsAcc;

    if ( minAbsVelAtGoalSqr > 0.0 && minAbsVelAtGoal > ( goalState->maxAbsVel + goalState->velTolerance ) ) {

        // Overshoot is unavoidable with the current constraints. Look at completion_behavior to see what we should do.
        if (goalState->completionBehavior == VIOLATE_MAX_ABS_VEL) {
            // Adjust the goal velocity.
            goalVel = minAbsVelAtGoal;

        } else if ( goalState->completionBehavior == VIOLATE_MAX_ACCEL ) {
            if ( fabs( deltaPos ) < goalState->posTolerance ) {
                // Special case: We are at the goal but moving too fast. This requires 'infinite' acceleration,
                // which will result in NaNs below, so we can return the profile immediately.
                segment.start.t = profile.tail->segment.end.t;
                segment.start.pos = profile.tail->segment.end.pos;
                segment.start.vel = profile.tail->segment.end.vel;
                segment.start.acc = -INFINITY;
                segment.end.t = profile.tail->segment.end.t;
                segment.end.pos = profile.tail->segment.end.pos;
                segment.end.vel = goalVel;
                segment.end.acc = -INFINITY;
                AppendSegment( &profile, &segment );
                Consolidate( &profile );
                return profile;
            }
            // Adjust the max acceleration.
            maxAcc = fabs( goalVel * goalVel - startState.vel * startState.vel ) / (2.0 * deltaPos);

        } else {
            // We are going to overshoot the goal, so the first thing we need to do is come to a stop.
            stoppingTime = fabs( startState.vel / constraints->maxAbsAcc );
            AppendControl( &profile, -constraints->maxAbsAcc, stoppingTime );
      
            // Now we need to travel backwards, so generate a flipped profile.
            flippedProfile = GenerateFlippedProfile( constraints, goalState, &profile.tail->segment.end );
            AppendProfile( &profile, &flippedProfile );
            Consolidate( &profile );
            return profile;
        }
    }

    goalVel = fmin( goalVel, maxAbsVelAtGoal);
    // Invariant from this point forward: We can achieve goal_vel at goal_state.pos exactly using no more than +/- max_acc.
    // What is the maximum velocity we can reach (Vmax)? This is the intersection of two curves: one accelerating
    // towards the goal from profile.finalState(), the other coming from the goal at max vel (in reverse). If Vmax
    // is greater than constraints.max_abs_vel, we will clamp and cruise.
    // Solve the following three equations to find Vmax (by substitution):
    // Vmax^2 = Vstart^2 + 2*a*d_accel
    // Vgoal^2 = Vmax^2 - 2*a*d_decel
    // delta_pos = d_accel + d_decel
    vMax = fmin( constraints->maxAbsVel, sqrt( ( startState.vel * startState.vel + goalVel * goalVel ) / 2.0 + deltaPos * maxAcc ) );
      
    // Accelerate to v_max
    if ( vMax > startState.vel ) {
        accelTime = ( vMax - startState.vel ) / maxAcc;
        AppendControl( &profile, maxAcc, accelTime );
        startState = profile.tail->segment.end;
    }

    // Figure out how much distance will be covered during deceleration.
    distanceDecel = fmax( 0.0, ( startState.vel * startState.vel - goalVel * goalVel) / (2.0 * constraints->maxAbsAcc ) );
    distanceCruise = fmax( 0.0, goalState->pos - startState.pos - distanceDecel );

    // Cruise at constant velocity.
    if ( distanceCruise > 0.0 ) {
        cruiseTime = distanceCruise / startState.vel;
        AppendControl( &profile, 0.0, cruiseTime );
        startState = profile.tail->segment.end;
    }

    // Decelerate to goal velocity.
    if ( distanceDecel > 0.0 ) {
        decelTime = ( startState.vel - goalVel ) / maxAcc;
        AppendControl( &profile, -maxAcc, decelTime );
    }

    Consolidate( &profile );
    return profile;
}

