//#include <math.h>
#include "Motion.h"
#include "../utils/Utils.h"

/******************************************************************************************************************************** 
**  AtGoalPosition
**
**      Input:
**
**      Output:
**
********************************************************************************************************************************/
motionProfileGoal_t FlippedGoal (motionProfileGoal_t *goal) {
    motionProfileGoal_t rv;

    rv = *goal;
    rv.pos = -goal->pos;

    return rv;
}


/******************************************************************************************************************************** 
**  AtGoalState
**
**      Input:
**
**      Output:
**
********************************************************************************************************************************/
int AtGoalState (motionProfileGoal_t *goal, motionState_t *state) {
    int rv;

    rv = AtGoalPosition( goal, state->pos ) && ( fabs( state->vel ) < ( goal->maxAbsVel + goal->velTolerance ) || goal->completionBehavior == VIOLATE_MAX_ABS_VEL );
    
    return rv;
}


/******************************************************************************************************************************** 
**  AtGoalPosition
**
**      Input:
**
**      Output:
**
********************************************************************************************************************************/
int AtGoalPosition (motionProfileGoal_t *goal, double pos) {
    int rv;

    rv = EpsilonEquals( pos, goal->pos, goal->posTolerance );
    
    return rv;
}


/******************************************************************************************************************************** 
**  SanityCheck
**
**      Input:
**
**      Output:
**
********************************************************************************************************************************/
enum completionBehavior_e SanityCheck (motionProfileGoal_t *goal) {
    enum completionBehavior_e rv = goal->completionBehavior;

    if ( goal->maxAbsVel > goal->velTolerance && goal->completionBehavior == OVERSHOOT ) {
        rv = VIOLATE_MAX_ACCEL;
    }

    return rv;
}


/******************************************************************************************************************************** 
**  GoalsAreEqual
**
**      Input:
**
**      Output:
**
********************************************************************************************************************************/
int GoalsAreEqual (motionProfileGoal_t *goalA, motionProfileGoal_t *goalB) {
    int rv;

    rv = ( goalA->completionBehavior == goalB->completionBehavior ) && ( goalA->pos == goalB->pos )  && ( goalA->maxAbsVel == goalB->maxAbsVel ) && ( goalA->posTolerance == goalB->posTolerance ) && ( goalA->velTolerance == goalB->velTolerance );
    return rv;
  }

/******************************************************************************************************************************** 
**  ConstraintsAreEqual
**
**      Input:
**
**      Output:
**
********************************************************************************************************************************/
int ConstraintsAreEqual (motionProfileConstraints_t *constraintsA, motionProfileConstraints_t *constraintsB) {
    int rv;

    rv = ( constraintsA->maxAbsAcc == constraintsB->maxAbsAcc ) && ( constraintsA->maxAbsVel == constraintsB->maxAbsVel );
    return rv;
}
