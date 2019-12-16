#include <math.h>
#include <stdlib.h>
#include "Motion.h"
#include "Utils.h"
#include "Constants.h"


/******************************************************************************************************************************** 
**  ClearProfile
**
**      Input:
**
**      Output:
**
********************************************************************************************************************************/
void ClearProfile (motionProfileList_t *profile) {
    motionProfileNode_t *profileNode, *removeNode;
    
    profileNode = profile->head;
    while ( profileNode ) {
        removeNode = profileNode;    
        profileNode = profileNode->next;
        free (removeNode);
    }
    profile->head = NULL;
    profile->tail = NULL;
    profile->length = 0;
}

/******************************************************************************************************************************** 
**  ResetProfile
**
**      Input:
**
**      Output:
**
********************************************************************************************************************************/
void ResetProfile (motionProfileList_t *profile, motionState_t *initialState) {
    motionProfileNode_t *newSegment;

    ClearProfile( profile );
    newSegment = malloc( sizeof( motionProfileNode_t ));
    
    newSegment->segment.start = *initialState;
    newSegment->segment.end = *initialState;
    newSegment->next = NULL;
    profile->head = newSegment;
    profile->tail = newSegment;
    profile->length = 1;
}



/******************************************************************************************************************************** 
**  AppendSegment
**
**      Input:
**
**      Output:
**
********************************************************************************************************************************/
void AppendSegment (motionProfileList_t *profile, motionSegment_t *segment) {
    motionProfileNode_t *newSegment;
    
    newSegment = malloc( sizeof( motionProfileNode_t ));
    newSegment->segment = *segment;
    newSegment->next = NULL;
    profile->tail->next = newSegment;
    profile->tail = newSegment;
    profile->length += 1;
}


/******************************************************************************************************************************** 
**  AppendControl
**
**      Input:
**
**      Output:
**
********************************************************************************************************************************/
void AppendControl (motionProfileList_t *profile, double acc, double dt) {
    motionState_t lastEndState, newStartState, newEndState;
    motionSegment_t newSegment;

    lastEndState = profile->tail->segment.end;
    newStartState.t = lastEndState.t;
    newStartState.pos = lastEndState.pos;
    newStartState.vel = lastEndState.vel;
    newStartState.acc = acc;

    newEndState = Extrapolate(&newStartState, newStartState.t + dt, acc);
    
    newSegment.start = newStartState;
    newSegment.end = newEndState;
    AppendSegment( profile, &newSegment );

}


/******************************************************************************************************************************** 
**  Consolidate
**
**      Input:
**
**      Output:
**
********************************************************************************************************************************/
void Consolidate (motionProfileList_t *profile) {
    motionProfileNode_t *currentNode, *nextNode;
    
    currentNode = profile->head;
    nextNode = currentNode->next;
    while ( nextNode && profile->length > 1 ) {
        if ( Coincident( &nextNode->segment.start, &nextNode->segment.end ) ) {
            currentNode->next = nextNode->next;
            free (nextNode);
            nextNode = currentNode->next;
        } else {
            currentNode = nextNode;
            nextNode = currentNode->next;
        }
    }
}


/******************************************************************************************************************************** 
**  AppendProfile
**
**      Input:
**
**      Output:
**
********************************************************************************************************************************/
void AppendProfile (motionProfileList_t *currentProfile, motionProfileList_t *addProfile) {
    motionProfileNode_t *addNode;
    motionSegment_t newSegment;
    
    addNode = addProfile->head;
    while ( addNode ) {
        newSegment = addNode->segment;
        AppendSegment( currentProfile, &newSegment);
        addNode = addNode->next;
    }
}



/******************************************************************************************************************************** 
**  TrimBeforeTime
**
**      Input:
**
**      Output:
**
********************************************************************************************************************************/
void TrimBeforeTime(motionProfileList_t *profile, double t) {
    motionProfileNode_t *currentNode, *deleteNode;
    motionState_t trimmedState;

    currentNode = profile->head;
    while ( currentNode ) {
        if ( currentNode->segment.end.t <= t ) {
            // Segment is fully before t. 
            deleteNode = currentNode;
            currentNode = currentNode->next;
            profile->head = currentNode;
            free (deleteNode);
            continue;
        }
        if ( currentNode->segment.start.t <= t ) {
            // Segment begins before t; let's shorten the segment.
            trimmedState = Extrapolate( &currentNode->segment.start, t, currentNode->segment.start.acc );
            currentNode->segment.start = trimmedState;
        }
        break;
    }
}



/******************************************************************************************************************************** 
**  IsProfileValid
**
**    Checks if the given MotionProfile is valid. This checks that:
**      1. All segments are valid.
**      2. Successive segments are C1 continuous in position and C0 continuous in velocity.
**      Input:
**
**      Output:
**
********************************************************************************************************************************/
int IsProfileValid (motionProfileList_t *profile) {
    motionProfileNode_t *currentNode, *prevNode;

    currentNode = profile->head;
    prevNode = NULL;
    while ( currentNode ) {
        if ( !IsSegmentValid( &currentNode->segment ) ) {
            return 0;
        }
        if ( !prevNode ) {
            if ( !Coincident( &currentNode->segment.start, &prevNode->segment.end ) ) {      
              // Adjacent segments are not continuous.
              //System.err.println("Segments not continuous! End: " + prev_segment.end() + ", Start: " + s.start());
              return 0;
            }
        }
        prevNode = currentNode;
        currentNode = currentNode->next;
    }
    return 1;
}

/******************************************************************************************************************************** 
**  StateByTime
**
**      Input:
**
**      Output:
**
********************************************************************************************************************************/
motionState_t StateByTime (motionProfileList_t *profile, double t) {
    motionState_t rv = {NAN, NAN, NAN, NAN};
    motionProfileNode_t *profileNode;

    if ( t < profile->head->segment.start.t && t + kEpsilon >= profile->head->segment.start.t ) {
        rv = profile->head->segment.start;
    
    } else if ( t > profile->tail->segment.end.t && t - kEpsilon <= profile->tail->segment.end.t ) {
        rv = profile->head->segment.end;
    
    } else {
        profileNode = profile->head;
        while ( profileNode ) {
            if ( ContainsTime( &profileNode->segment, t ) ) {
                rv = Extrapolate( &profileNode->segment.start, t, profileNode->segment.start.acc );
                break;
            }
            profileNode = profileNode->next;
        }
    }

    return rv;
}


/******************************************************************************************************************************** 
**  MotionProfileStateByTimeClamped
**
**      Input:
**
**      Output:
**
********************************************************************************************************************************/
motionState_t StateByTimeClamped (motionProfileList_t *profile, double t) {
    motionState_t rv = {NAN, NAN, NAN, NAN};
    motionProfileNode_t *profileNode;

    if ( t < profile->head->segment.start.t ) {
        rv = profile->head->segment.start;
    
    } else if ( t > profile->tail->segment.end.t ) {
        rv = profile->head->segment.end;
    
    } else {
        profileNode = profile->head;
        while ( profileNode ) {
            if ( ContainsTime( &profileNode->segment, t ) ) {
                rv = Extrapolate( &profileNode->segment.start, t, profileNode->segment.start.acc );
                break;
            }
            profileNode = profileNode->next;
        }
    }

    return rv;
}


/******************************************************************************************************************************** 
**  FirstStateByPosition
**
**      Input:
**
**      Output:
**
********************************************************************************************************************************/
motionState_t FirstStateByPosition (motionProfileList_t *profile, double pos) {
    motionState_t rv = {NAN, NAN, NAN, NAN};
    motionProfileNode_t *profileNode;
    double t;

    profileNode = profile->head;
    while ( profileNode ) {
        if ( ContainsPosition( &profileNode->segment, pos ) ) {
            if ( EpsilonEquals( profileNode->segment.end.pos , pos, kEpsilon ) ) {
                rv = profileNode->segment.end;
                return rv;
            }
            t = fmin( NextTimeAtPos( &profileNode->segment.start, pos ), profileNode->segment.end.t );
            if ( isnan( t ) ) {
                // Print an error
                return rv;
            }
            rv = Extrapolate( &profileNode->segment.start, t, profileNode->segment.start.acc );
            return rv;

        }
        profileNode = profileNode->next;
    }

    return rv;
}


