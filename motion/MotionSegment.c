#include <math.h>
#include "Utils.h"
#include "Constants.h"
#include "Motion.h"

/******************************************************************************************************************************** 
**  IsSegmentValid
** 
**    Checks if the given MotionSegment is valid. This checks that:
**      1. All segments have a constant acceleration.
**      2. All segments have monotonic position (sign of velocity doesn't change).
**      3. The time, position, velocity, and acceleration of the profile are consistent.
**
**      Input:
**
**      Output:
**
********************************************************************************************************************************/
int IsSegmentValid (motionSegment_t *segment) {
    motionState_t state;

    if ( !EpsilonEquals( segment->start.acc, segment->end.acc, kEpsilon ) ) {
        // Acceleration is not constant within the segment.
        //System.err.println("Segment acceleration not constant! Start acc: " + start().acc() + ", End acc: " + end().acc());
        return 0;
    }
    if ( SignNum( segment->start.vel ) * SignNum ( segment->end.vel ) < 0.0 && !EpsilonEquals( segment->start.vel, 0.0, kEpsilon ) && !EpsilonEquals( segment->end.vel, 0.0, kEpsilon ) ) {
        // Velocity direction reverses within the segment.
        //System.err.println("Segment velocity reverses! Start vel: " + start().vel() + ", End vel: " + end().vel());
        return 0;
    }

    state = Extrapolate( &segment->start, segment->end.t, segment->start.acc);
    if ( !MotionStatesAreEqual( &state, &segment->end ) ) {
        // A single segment is not consistent. 
        if ( segment->start.t == segment->end.t && isinf( segment->start.acc ) ) {
            // One allowed exception: If acc is infinite and dt is zero.
            return 1;
        }
        //System.err.println("Segment not consistent! Start: " + start() + ", End: " + end());
        return 0;
    }

    return 1;
}


/******************************************************************************************************************************** 
**  ContainsTime
**
**      Input:
**
**      Output:
**
********************************************************************************************************************************/
int ContainsTime (motionSegment_t *segment, double t) {
    int rv;

    rv = t >= segment->start.t && t <= segment->end.t;

    return rv;
}

/******************************************************************************************************************************** 
**  ContainsPosition
**
**      Input:
**
**      Output:
**
********************************************************************************************************************************/
int ContainsPosition (motionSegment_t *segment, double pos) {
    int rv;

    rv = ( pos >= segment->start.pos && pos <= segment->end.pos ) || (pos <= segment->start.pos && pos >= segment->end.pos);
    
    return rv;
}
