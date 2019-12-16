#include <stdlib.h>
#include <math.h>
#include "Motion.h"
#include "Utils.h"

/******************************************************************************************************************************** 
**  ClearSetpointGenerator
**      
**      Input:
**
**      Output:
**
********************************************************************************************************************************/
void ClearSetpointGenerator (setpointGenerator_t *setpointGenerator) {
    free( setpointGenerator->constraints );
    setpointGenerator->constraints = NULL;
    free( setpointGenerator->goal );
    setpointGenerator->goal = NULL;
    if ( setpointGenerator->profile ) {
        motionProfileNode_t *node, *removeNode;
        node = setpointGenerator->profile->head;
        while ( node ) {
            removeNode = node;
            node = node->next;
            free (removeNode);
        }
    }
    free ( setpointGenerator->profile );
    setpointGenerator->profile = NULL;
}


/******************************************************************************************************************************** 
**  SetSetpointGenerator
**      
**      Input:
**
**      Output:
**
********************************************************************************************************************************/
void SetSetpointGenerator (setpointGenerator_t *setpointGenerator, motionProfileConstraints_t *constraints, motionProfileGoal_t *goal, motionState_t *prevState) {
    setpointGenerator->constraints = malloc( sizeof( motionProfileConstraints_t ) );
    *(setpointGenerator->constraints) = *constraints;
    setpointGenerator->goal = malloc( sizeof( motionProfileGoal_t ) );
    *(setpointGenerator->goal) = *goal;
    setpointGenerator->profile = malloc( sizeof( motionProfileList_t ) );
    *(setpointGenerator->profile) = GenerateProfile( constraints, goal, prevState );
}


/******************************************************************************************************************************** 
**  GetSetpoint
**      
**      Input:
**
**      Output:
**
********************************************************************************************************************************/
setpoint_t GetSetpoint (setpointGenerator_t *setpointGenerator, motionProfileConstraints_t *constraints, motionProfileGoal_t *goal, motionState_t *prevState, double t) {
    setpoint_t rv;
    int regenerate;
    motionState_t expectedState;

    regenerate = !setpointGenerator->constraints || ConstraintsAreEqual( setpointGenerator->constraints, constraints ) || !setpointGenerator->goal || GoalsAreEqual( setpointGenerator->goal, goal ) || !setpointGenerator->profile;

    if ( !regenerate && setpointGenerator->profile->length ) {
        expectedState = StateByTime( setpointGenerator->profile, prevState->t );
        regenerate = expectedState.t == NAN || !MotionStatesAreEqual( &expectedState, prevState ) ;
    }
    if ( regenerate ) {
        // Regenerate the profile, as our current profile does not satisfy the inputs.
        ClearSetpointGenerator( setpointGenerator );
        SetSetpointGenerator( setpointGenerator, constraints, goal, prevState);
    }

    rv.finalSetpoint = -1;
    // Sample the profile at time t.
    if ( setpointGenerator->profile && IsProfileValid( setpointGenerator->profile ) ) {
        if ( t > setpointGenerator->profile->tail->segment.end.t ) {
            rv.motionState = setpointGenerator->profile->tail->segment.end;
        } else if ( t < setpointGenerator->profile->tail->segment.start.t ) {
            rv.motionState = setpointGenerator->profile->tail->segment.start;
        } else {
            rv.motionState = StateByTime( setpointGenerator->profile, t );
        } 
        // Shorten the profile and return the new setpoint.
        TrimBeforeTime( setpointGenerator->profile, t );
        rv.finalSetpoint = !setpointGenerator->profile->length || AtGoalState( setpointGenerator->goal, &rv.motionState );
    }

    // Invalid or empty profile - just output the same state again.
    if ( rv.finalSetpoint == -1 ) {
      rv.motionState = *prevState;
      rv.finalSetpoint = 1;
    }

    if (rv.finalSetpoint) {
      // Ensure the final setpoint matches the goal exactly.
      rv.motionState.pos = setpointGenerator->goal->pos;
      rv.motionState.vel = SignNum( rv.motionState.vel ) * fmax( setpointGenerator->goal->maxAbsVel, fabs( rv.motionState.vel ) );
      rv.motionState.acc = 0.0;
    }

    return rv;
}

