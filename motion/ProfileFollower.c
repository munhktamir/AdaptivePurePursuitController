#include <stdlib.h>
#include <math.h>
#include "Constants.h"
#include "Utils.h"
#include "Motion.h"


/******************************************************************************************************************************** 
**  SetProfileFollowerGains
**
**      Input:
**
**      Output: 
**
********************************************************************************************************************************/
void SetProfileFollowerGains (profileFollower_t *profileFollower, double kp, double ki, double kv, double kffv, double kffa) {
    profileFollower->kP = kp;
    profileFollower->kI = ki;
    profileFollower->kV = kv;
    profileFollower->kFFV = kffv;
    profileFollower->kFFA = kffa;
}


/******************************************************************************************************************************** 
**  SetProfileFollowerGoalAndConstraints
**
**      Input:
**
**      Output: 
**
********************************************************************************************************************************/
void SetProfileFollowerGoalAndConstraints (profileFollower_t *profileFollower, motionProfileGoal_t *goal, motionProfileConstraints_t *constraints) {
    if ( profileFollower->goal && !GoalsAreEqual( profileFollower->goal, goal ) && profileFollower->latestSetpoint ) {
        // Clear the final state bit since the goal has changed.
        profileFollower->latestSetpoint->finalSetpoint = 0;
    }
    *(profileFollower->goal) = *goal;
    *(profileFollower->constraints) = *constraints;
}


/******************************************************************************************************************************** 
**  ClearProfileFollower
**
**      Completely clear all state related to the current profile (min and max outputs are maintained).
**
**      Input:
**
**      Output: 
**
********************************************************************************************************************************/
void ClearProfileFollower (profileFollower_t *profileFollower) {
    profileFollower->totalError = 0.0;
    profileFollower->initialState = kInvalidMotionState;
    profileFollower->latestActualState = kInvalidMotionState;
    profileFollower->latestPosError = NAN;
    profileFollower->latestVelError = NAN;
    free( profileFollower->constraints );
    profileFollower->constraints = NULL;
    free( profileFollower->goal );
    profileFollower->goal = NULL;
    ClearSetpointGenerator( profileFollower->setppointGenerator );
    free( profileFollower->latestSetpoint );
    profileFollower->latestSetpoint = NULL;
}


/******************************************************************************************************************************** 
**  ProfileFollowerUpdate
**
**      Completely clear all state related to the current profile (min and max outputs are maintained).
**
**      Input:
**
**      Output: 
**
********************************************************************************************************************************/
double ProfileFollowerUpdate (profileFollower_t *profileFollower, motionState_t *latestState, double t) {
    motionState_t prevState;
    double dt, output;

    profileFollower->latestActualState = *latestState;
    prevState = *latestState;
    if ( profileFollower->latestSetpoint ) {
        prevState = profileFollower->latestSetpoint->motionState;
    } else {
        profileFollower->initialState = prevState;
    }
    dt = fmax( 0.0, prevState.t );
    *(profileFollower->latestSetpoint) = GetSetpoint(profileFollower->setppointGenerator, profileFollower->constraints, profileFollower->goal, &prevState, t);
   
    // Update error
    profileFollower->latestPosError = profileFollower->latestSetpoint->motionState.pos - latestState->pos;
    profileFollower->latestVelError = profileFollower->latestSetpoint->motionState.vel - latestState->vel;

    // Calculate the feedforward and proportional terms
    output = profileFollower->kP * profileFollower->latestPosError + profileFollower->kV * profileFollower->latestVelError + profileFollower->kFFV * profileFollower->latestSetpoint->motionState.vel + ( isnan( profileFollower->latestSetpoint->motionState.acc ) ? 0.0 : profileFollower->kFFA * profileFollower->latestSetpoint->motionState.acc );
    if ( output >= profileFollower->minOutput && output <= profileFollower->maxOutput ) {
        // Update integral.
        profileFollower->totalError += profileFollower->latestPosError * dt;
        output += profileFollower->kI * profileFollower->totalError;
    } else {
        // Reset integral windup.
        profileFollower->totalError = 0.0;
    }

    // Clamp to limits.
    output = fmax( profileFollower->minOutput, fmin( profileFollower->maxOutput, output ) );
    return output;
}




/******************************************************************************************************************************** 
**  GetProfileFollowerUpdate
**
**      We are finished the profile when the final setpoint has been generated. Note that this does not check whether we
**      are anywhere close to the final setpoint, however.
**
**      Input:
**
**      Output: 
**
********************************************************************************************************************************/
int IsProfileFinished (profileFollower_t *profileFollower) {
    int rv;

    rv = profileFollower->goal && profileFollower->latestSetpoint && profileFollower->latestSetpoint->finalSetpoint;
    return rv;

}


/******************************************************************************************************************************** 
**  IsProfileOnTarget
**
**      We are finished the profile when the final setpoint has been generated. Note that this does not check whether we
**      are anywhere close to the final setpoint, however.
**
**      Input:
**
**      Output: 
**
********************************************************************************************************************************/
int IsProfileOnTarget (profileFollower_t *profileFollower) {
    int rv, pastGoalState;
    double goalToStart, goalToActual;

    if ( !profileFollower->goal || !profileFollower->latestSetpoint ) {
        return 0;
    }
    goalToStart = profileFollower->goal->pos - profileFollower->initialState.pos;
    goalToActual = profileFollower->goal->pos - profileFollower->latestActualState.pos;
    pastGoalState =  SignNum( goalToStart ) * SignNum( goalToActual ) < 0.0;
    rv =  AtGoalState( profileFollower->goal, &profileFollower->latestActualState ) || (profileFollower->goal->completionBehavior != OVERSHOOT && pastGoalState );
    return  rv;
}


/******************************************************************************************************************************** 
**  GetMotionProfileSetpoint
**
**
**      Input:
**
**      Output: 
**
********************************************************************************************************************************/
motionState_t GetProfileSetpoint (profileFollower_t *profileFollower) {
    motionState_t rv;

    rv = !profileFollower->latestSetpoint ? kInvalidMotionState : profileFollower->latestSetpoint->motionState;
    return rv;
}
















//   /**
//    * Specify a goal and constraints for achieving the goal.
//    */
//   public void setGoalAndConstraints(MotionProfileGoal goal, MotionProfileConstraints constraints) {
//     if (mGoal != null && !mGoal.equals(goal) && mLatestSetpoint != null) {
//       // Clear the final state bit since the goal has changed.
//       mLatestSetpoint.final_setpoint = false;
//     }
//     mGoal = goal;
//     mConstraints = constraints;
//   }

//   public void setGoal(MotionProfileGoal goal) {
//     setGoalAndConstraints(goal, mConstraints);
//   }

//   /**
//    * @return The current goal (null if no goal has been set since the latest call to reset()).
//    */
//   public MotionProfileGoal getGoal() {
//     return mGoal;
//   }

//   public void setConstraints(MotionProfileConstraints constraints) {
//     setGoalAndConstraints(mGoal, constraints);
//   }



//   /**
//    * Reset just the setpoint. This means that the latest_state provided to update() will be used rather than feeding
//    * forward the previous setpoint the next time update() is called. This almost always forces a MotionProfile update,
//    * and may be warranted if tracking error gets very large.
//    */
//   public void resetSetpoint() {
//     mLatestSetpoint = null;
//   }

//   public void resetIntegral() {
//     mTotalError = 0.0;
//   }


//   public void setMinOutput(double min_output) {
//     mMinOutput = min_output;
//   }

//   public void setMaxOutput(double max_output) {
//     mMaxOutput = max_output;
//   }

//   public double getPosError() {
//     return mLatestPosError;
//   }

//   public double getVelError() {
//     return mLatestVelError;
//   }


  
// }