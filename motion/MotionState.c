#include <math.h>
#include "../utils/Utils.h"
#include "Constants.h"
#include "Motion.h"


/******************************************************************************************************************************** 
**  Extrapolate
**
**      Extrapolates the given motion state to the specified time by applying a given acceleration to the (t, pos, vel) portion
**      of this motion state.  A motion state that is a valid predecessor (if t<=0) or successor (if t>=0) of this state (with
**      the specified accel).
**
**      Input:
**          motionState_t state     The motion state to use for extrapolating
**          double t                The time to exatrapolate the state to
**          double acc              The acceleration used during the extrapolation
**
**      Output:
**          motionState_t           Return a new motino state which is the extrapolation of the input motion state
**
********************************************************************************************************************************/
motionState_t Extrapolate (motionState_t *state, double t, double acc) {
    motionState_t rv;
    double dt;
    
    dt = t - state->t;
    rv.t = t;
    rv.pos = state->pos + state->vel * dt + 0.5 * acc * dt * dt;
    rv.vel = state->vel + acc * dt;
    rv.acc = acc;

    return rv;
}


/******************************************************************************************************************************** 
**  NextTimeAtPos
**
**      Find the next time (first time > MotionState.t()) that this motion state will be at pos. This is an inverse of the
**      extrapolate() method.  The time when we are next at pos() if we are extrapolating with a positive dt. NaN if we never
**      reach pos.
**
**      Input:
**          motionState_t state     The motion state used to determine when the position will be reached
**          double pos              The position in question
**
**      Output: 
**          double                  Return the time when the motion state will reach the input position
**
********************************************************************************************************************************/
double NextTimeAtPos (motionState_t *state, double pos) {
    double deltaPos, disc, sqrtDisc, maxDt, minDt;

    // Already at position
    if ( EpsilonEquals( pos, state->pos, kEpsilon ) ) {
        return state->t;
    }

    // Zero acceleration
    if ( EpsilonEquals( state->acc, 0.0, kEpsilon ) ) {
        
        deltaPos = pos - state->pos;
        if ( !EpsilonEquals( state->vel, 0.0, kEpsilon ) && ( deltaPos * state->vel ) >= 0.0 ) {
            
            // Constant velocity heading towards position
            return deltaPos / state->vel + state->t;
        }
        return NAN;
    }

    // Solve the quadratic formula.
    // ax^2 + bx + c == 0
    // x = dt
    // a = .5 * acc
    // b = vel
    // c = this.pos - pos
    disc = state->vel * state->vel - 2.0 * state->acc * (state->pos - pos);
    if (disc < 0.0) {

        // Extrapolating this motion state never reaches the desired position
        return NAN;
    }

    sqrtDisc = sqrt(disc);
    maxDt = (-state->vel + sqrtDisc) / state->acc;
    minDt = (-state->vel - sqrtDisc) / state->acc;
    if (minDt >= 0.0 && (maxDt < 0.0 || minDt < maxDt)) {
        return state->t + minDt;
    }
    if (maxDt >= 0.0) {
        return state->t + maxDt;
    }

    // We only reach the desired position in the past
    return NAN;
}


/******************************************************************************************************************************** 
**  FlippedState
**
**      Input:
**          motionState_t state     The motion state to flip
**
**      Output: 
**          motionState_t           Return the flipped (inverted) motion state
**
********************************************************************************************************************************/
motionState_t FlippedState (motionState_t *state) {
    motionState_t rv;

    rv.t = state->t;
    rv.pos = -state->pos;
    rv.vel = -state->vel;
    rv.acc = -state->acc;

    return rv;
}

/******************************************************************************************************************************** 
**  Coincident
**
**      Checks if two motion states are coincident (t, pos, and vel are equal within a nominal tolerance, but acceleration may be
**      different).
**
**      Input:
**          motionState_t a         Motion state A
**          motionState_t b         Motion state B
**
**      Output:
**          int                     Return 1 if they're coincident, else 0
**
********************************************************************************************************************************/
int Coincident (motionState_t *a, motionState_t *b) {
    int rv;

    rv = EpsilonEquals( a->t, b->t, kEpsilon ) && EpsilonEquals( a->pos, b->pos, kEpsilon ) && EpsilonEquals( a->vel, b->vel, kEpsilon );

    return rv;
}

/******************************************************************************************************************************** 
**  MotionStatesAreEqual
**
**      Checks if two motion states are equal (t, pos, vel, acc are equal within a nominal tolerance).
**
**      Input:
**          motionState_t a         Motion state A
**          motionState_t b         Motion state B
**
**      Output:
**          int                     Return 1 if they're equal, else 0
**
********************************************************************************************************************************/
int MotionStatesAreEqual (motionState_t *stateA, motionState_t *stateB) {
    int rv;

    rv = Coincident( stateA, stateB ) && EpsilonEquals( stateA->acc, stateB->acc, kEpsilon);
    return rv;
}
