#include <stdlib.h>
#include "Geometry.h"
#include "Motion.h"
#include "Path.h"


/******************************************************************************************************************************** 
**  CreateLine
**
**      Input:
**
**      Output:
**
********************************************************************************************************************************/
line_t CreateLine (waypoint_t *a, waypoint_t *b) {
    line_t rv;
    translation2d_t scaledSlope;

    rv.waypointA = *a;
    rv.waypointB = *b;
    rv.slope = TranslationDelta( &a->position, &b->position );
    rv.speed_ips = b->speed_ips;
    scaledSlope = TranslationScale( &rv.slope, a->radius / TranslationNormal( &rv.slope ) );
    rv.start = TranslateAbyB( &a->position, &scaledSlope );
    scaledSlope = TranslationScale( &rv.slope, -b->radius / TranslationNormal( &rv.slope ) );
    rv.end = TranslateAbyB( &b->position, &scaledSlope );

    return rv;
}


/******************************************************************************************************************************** 
**  CreateLine
**
**      Input:
**
**      Output:
**
********************************************************************************************************************************/
translation2d_t Intersect (line_t *lineA, line_t *lineB) {
    translation2d_t rv;
    transform2d_t transformA, transformB;

    transformA.translation = lineA->end;
    transformA.rotation.cosTheta_rad = lineA->slope.x_in;
    transformA.rotation.sinTheta_rad = lineA->slope.y_in; 
    transformA.rotation = RotationNormalize ( &transformA.rotation );      
    transformB.translation = lineB->start;
    transformB.rotation.cosTheta_rad = lineB->slope.x_in;
    transformB.rotation.sinTheta_rad = lineB->slope.y_in; 
    transformB.rotation = RotationNormalize ( &transformB.rotation );      
    rv = Intersection( &transformA, &transformB);

    return rv;
}


/******************************************************************************************************************************** 
**  CreateLine
**
**      Input:
**
**      Output:
**
********************************************************************************************************************************/
arc_t CreateArc(waypoint_t *a, waypoint_t *b, waypoint_t *c) {
    arc_t rv;
    line_t lineA, lineB;
    translation2d_t delta;
    
    lineA = CreateLine(a, b);
    lineB = CreateLine(b, c);
    rv.lineA = lineA;
    rv.lineB = lineB;
    rv.speed_ips = ( a->speed_ips + b->speed_ips ) / 2.0;
    rv.center = Intersect( &lineA, &lineB);
    delta = TranslationDelta( &rv.center, &lineA.end );
    rv.radius = TranslationNormal( &delta );
 
    return rv;
}

/******************************************************************************************************************************** 
**  AddPathSegment
**
**      Input:
**
**      Output:
**
********************************************************************************************************************************/
void AddPathSegment (pathSegmentsList_t *segments, pathSegmentNode_t *segment) {
    if ( !segments ) {
        segment->next = NULL;
        segment->prev = NULL;    
        segments = malloc( sizeof( pathSegmentsList_t ) );
        segments->head = segment;
        segments->tail = segment;
        segments->length = 1;                    
    } else {
        segment->next = NULL;
        segment->prev = segments->tail;
        segments->tail = segment;
        segments->length += 1;    
    }
}

/******************************************************************************************************************************** 
**  BuildPathFromWaypoints
**
**      Input:
**
**      Output:
**
********************************************************************************************************************************/
pathSegmentsList_t BuildPathFromWaypoints (waypoint_t *wps[], int size) {
    pathSegmentsList_t path = {NULL, NULL, 0};
    pathSegmentNode_t *nextSegment;
    arc_t arc;
    line_t line;
    translation2d_t deltaStart, deltaEnd;
    motionProfileList_t speedController;
    motionState_t startState;
    int i = 0;

    if (size > 2) {
        do {
            arc = CreateArc( wps[i], wps[i+1], wps[i+2] );  
            deltaStart = TranslationDelta( &arc.lineA.start, &arc.lineA.end );
            if ( TranslationNormal( &deltaStart ) > 1e-9 ) {
                nextSegment = malloc( sizeof( pathSegmentNode_t ) );
                nextSegment->segment.start = arc.lineA.start;
                nextSegment->segment.end = arc.lineA.end;
                nextSegment->segment.deltaStart = deltaStart;
                nextSegment->segment.maxSpeed_ips = arc.lineA.speed_ips;
                nextSegment->segment.isLine = 1;
                nextSegment->segment.center.x_in = 0.0;
                nextSegment->segment.center.y_in = 0.0;
                nextSegment->segment.deltaEnd.x_in = 0.0;
                nextSegment->segment.deltaEnd.y_in = 0.0;
                nextSegment->segment.extrapolateLookahead = 0;
                startState = GetLastMotionState( &path );
                speedController = CreateMotionProfiler( &startState, arc.speed_ips, arc.lineA.waypointB.speed_ips, GetLength( &nextSegment->segment ) );
                nextSegment->segment.speedController = &speedController;
                AddPathSegment( &path, nextSegment );
            }

            if ( arc.radius > 1e-9 && arc.radius < 1e9 ) {
                nextSegment = malloc( sizeof( pathSegment_t ) );
                deltaStart = TranslationDelta( &arc.center, &arc.lineA.end );
                deltaEnd = TranslationDelta( &arc.center, &arc.lineB.start );
                nextSegment->segment.start = arc.lineA.end;
                nextSegment->segment.end = arc.lineB.start;
                nextSegment->segment.deltaStart = deltaStart;
                nextSegment->segment.deltaEnd = deltaEnd;
                nextSegment->segment.maxSpeed_ips = arc.speed_ips;
                nextSegment->segment.isLine = 0;
                nextSegment->segment.center = arc.center;
                nextSegment->segment.extrapolateLookahead = 0;
                startState = GetLastMotionState( &path );
                speedController = CreateMotionProfiler( &startState, arc.lineB.speed_ips, arc.speed_ips, GetLength( &nextSegment->segment ) );
                nextSegment->segment.speedController = &speedController;
                AddPathSegment( &path, nextSegment );
            }
            ++i;
        } while (i < size - 2);
    }

    line = CreateLine( wps[size - 2], wps[size - 1] );
    deltaStart = TranslationDelta( &line.start, &line.end );
    if ( TranslationNormal( &deltaStart ) > 1e-9 ) {
        nextSegment = malloc( sizeof( pathSegment_t ) );
        nextSegment->segment.start = line.start;
        nextSegment->segment.end = line.end;
        nextSegment->segment.deltaStart = deltaStart;
        nextSegment->segment.maxSpeed_ips = line.speed_ips;
        nextSegment->segment.isLine = 1;
        nextSegment->segment.center.x_in = 0.0;
        nextSegment->segment.center.y_in = 0.0;
        nextSegment->segment.deltaEnd.x_in = 0.0;
        nextSegment->segment.deltaEnd.y_in = 0.0;
        nextSegment->segment.extrapolateLookahead = 0;
        startState = GetLastMotionState( &path );
        speedController = CreateMotionProfiler( &startState, 0.0, line.waypointB.speed_ips, GetLength( &nextSegment->segment ) );
        nextSegment->segment.speedController = &speedController;
        AddPathSegment( &path, nextSegment );
    }    

    return path;
}
