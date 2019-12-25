#include "Geometry.h"
#include "../robot/RobotMap.h"
#include "../motion/Motion.h"
#include "Path.h"


/******************************************************************************************************************************** 
**  CreateMotionProfiler
**
**      Input:
**
**      Output:
**
********************************************************************************************************************************/
motionProfileList_t CreateMotionProfiler (motionState_t *startState, double endSpeed, double maxSpeed, double length) {
    motionProfileList_t rv;
    motionProfileConstraints_t motionConstraints;
    motionProfileGoal_t goalState;

    motionConstraints.maxAbsVel = maxSpeed;
    motionConstraints.maxAbsAcc = kPathFollowingMaxAccel;
    goalState.completionBehavior = OVERSHOOT;
    goalState.maxAbsVel = endSpeed;
    goalState.pos = length;
    goalState.posTolerance = 1e-3;
    goalState.velTolerance = 1e-2;
    if ( motionConstraints.maxAbsVel > goalState.velTolerance && goalState.completionBehavior == OVERSHOOT ) {
        goalState.completionBehavior = VIOLATE_MAX_ACCEL;
    }
    rv = GenerateProfile( &motionConstraints, &goalState, startState );

    return rv;
}


/******************************************************************************************************************************** 
**  GetLength
**
**      Input:  The segment used to calculate the length.
**
**      Output: Returns the length of segment.
**
********************************************************************************************************************************/
double GetLength (pathSegment_t *segment) {
    double normal;

    if ( segment->isLine ) {
        normal = TranslationNormal( &segment->deltaStart );
    } else {
        normal = TranslationNormal( &segment->deltaStart ) * TranslationGetAngle( &segment->deltaStart, &segment->deltaEnd );
    }
    return normal;
}


/******************************************************************************************************************************** 
**  GetClosestPoint
**
**      Input:  Input the current 2D translational position of the robot.  This function will always operate on the first segment
**              pointed to by the global gPATH.
**
**      Output: The closest point on the segment to the robot's position.  For an arc, this point is 180deg out-of-phase with the
**              true point.  This means the the closest point for an arc will need to 
**
********************************************************************************************************************************/
translation2d_t GetClosestPoint (pathSegment_t *segment, translation2d_t *robotPosition) {
    translation2d_t delta, closestPoint, startDist, endDist;
    double scale;

    // Solve for the case where the path segment is a line by projecting the vector from the start of the segment to the robot's
    // position onto the vector from the segment start to the segment end.
    if ( segment->isLine ) {
        delta = TranslationDelta( &segment->start, &segment->end );
        scale = ( ( robotPosition->x_in - segment->start.x_in ) * delta.x_in + ( robotPosition->y_in - segment->start.y_in ) * delta.y_in ) / ( delta.x_in * delta.x_in + delta.y_in * delta.y_in );
        if(scale >= 0 && scale <= 1) {
            closestPoint.x_in = segment->start.x_in + scale * delta.x_in;
            closestPoint.y_in = segment->start.y_in + scale * delta.y_in; 
        } else {
            closestPoint = ( scale < 0.0 ) ? segment->start : segment->end;
        }

    // Solve for the case where the path segment is an arc.  The delta between the robot position and the segments center is
    // scaled by the normal of the start->center vector divided by the normal of the robot-position->center vector.  The
    // segments center is translated by this delta and the closest point will fall on an arc which is 180deg out of phase.  
    } else {
        delta = TranslationDelta( &segment->center, robotPosition );            
        scale = TranslationNormal( &segment->deltaStart) / TranslationNormal( &delta );
        delta = TranslationScale( &delta, scale );

        if ( TranslationCross( &delta, &segment->deltaStart ) * TranslationCross( &delta, &segment->deltaEnd ) < 0.0) {
            closestPoint = TranslateAbyB( &segment->center, &delta );

        } else {
            startDist = TranslateAbyB( &segment->start, robotPosition );
            endDist = TranslateAbyB( &segment->end, robotPosition );
            closestPoint = ( TranslationNormal( &endDist ) < TranslationNormal( &startDist ) ) ? segment->end : segment->start;
        }
    }

    return closestPoint;
}


/******************************************************************************************************************************** 
**  GetRemainingDistance
**
**      Input:
**
**      Output: Returns the remaining distance left to travel.
**
********************************************************************************************************************************/
double GetRemainingDistance (pathSegment_t *segment, translation2d_t *position) {
    translation2d_t deltaPosition;
    double currentAngle_rad, totalAngle_rad, remaingingDistance;

    if ( segment->isLine ) {
        deltaPosition = TranslationDelta( &segment->end, position );
        remaingingDistance = TranslationNormal( &deltaPosition );

    } else {
        deltaPosition = TranslationDelta( &segment->center, position );
        currentAngle_rad = TranslationGetAngle( &segment->deltaEnd, &deltaPosition );
        totalAngle_rad = TranslationGetAngle( &segment->deltaStart, &segment->deltaEnd );
        remaingingDistance = currentAngle_rad / totalAngle_rad * GetLength( segment );
    }

    return remaingingDistance;
}


/******************************************************************************************************************************** 
**  GetPointByDistance
**
**      Input:
**
**      Output: Returns the point which is the given distance along the segment.
**
********************************************************************************************************************************/
translation2d_t GetPointByDistance (pathSegment_t *segment, double dist) {
    translation2d_t point, deltaStart;
    rotation2d_t rot;
    double length, deltaAngle;
    
    length = GetLength( segment );
    if ( !segment->extrapolateLookahead && dist > length ) {
        dist = length;
    }
    if ( segment->isLine ) {
        deltaStart = TranslationScale( &segment->deltaStart, dist / length );
        point = TranslateAbyB( &segment->start, &deltaStart );
    
    } else {
        deltaAngle = TranslationGetAngle( &segment->deltaStart, &segment->deltaEnd ) * ( ( TranslationCross( &segment->deltaStart, &segment->deltaEnd ) >= 0.0 ) ? 1.0 : -1.0 );
        deltaAngle *= dist / length;
        rot.cosTheta_rad = cos(deltaAngle);
        rot.sinTheta_rad = sin(deltaAngle);
        deltaStart = TranslationRotate( &segment->deltaStart, &rot );
        point = TranslateAbyB( &segment->center, &deltaStart);
    }

    return point;
}


/******************************************************************************************************************************** 
**  getDistanceTravelled
**
**      Input:
**
**      Output: Returns the point which is the given distance along the segment.
**
********************************************************************************************************************************/
double GetDistanceTravelled (pathSegment_t *segment, translation2d_t *robotPosition) {
    double rv, remainingDistance;
    translation2d_t pathPosition;

    pathPosition = GetClosestPoint( segment, robotPosition );
    remainingDistance = GetRemainingDistance( segment, &pathPosition );
    rv = GetLength( segment ) - remainingDistance;
    return rv;
}


/******************************************************************************************************************************** 
**  GetSpeedByDistance
**
**      Input:  The path segment and the current 2D translational position of the robot.
**
**      Output:
**
********************************************************************************************************************************/
double GetSpeedByDistance(pathSegment_t *segment, double dist) {
    double rv;
    motionState_t state;

    if ( dist < segment->speedController->head->segment.start.pos ) {
        dist = segment->speedController->head->segment.start.pos;

    } else if ( dist > segment->speedController->tail->segment.end.pos ) {
        dist = segment->speedController->tail->segment.end.pos;

    }
    state = FirstStateByPosition( segment->speedController, dist );
    if ( isnan( state.vel ) ) {
        rv = 0.0;
    } else {
        rv = state.vel;
    }

    return rv;
}


/******************************************************************************************************************************** 
**  GetSpeedByClosePoint
**
**      Input:
**
**      Output: Returns the point which is the given distance along the segment.
**
********************************************************************************************************************************/
double GetSpeedByClosePoint (pathSegment_t *segment, translation2d_t *robotPosition) {
    double rv;
    
    rv = GetSpeedByDistance( segment, GetDistanceTravelled( segment, robotPosition ) );
    return rv;
}




//   /****************************************************************************************************************************** 
//   **  Constructor for a linear segment
//   ******************************************************************************************************************************/
//   public PathSegment(double x1, double y1, double x2, double y2, double maxSpeed, MotionState startState, double endSpeed) {
//     this.start = new Translation2d(x1, y1);
//     this.end = new Translation2d(x2, y2);
//     this.deltaStart = new Translation2d(start, end);
//     this.maxSpeed = maxSpeed;
//     extrapolateLookahead = false;
//     isLine = true;
//     createMotionProfiler(startState, endSpeed);
//   }

//   /****************************************************************************************************************************** 
//   **  Constructor for an arc segment
//   ******************************************************************************************************************************/
//   public PathSegment(double x1, double y1, double x2, double y2, double cx, double cy, double maxSpeed, MotionState startState, double endSpeed) {
//     this.start = new Translation2d(x1, y1);
//     this.end = new Translation2d(x2, y2);
//     this.center = new Translation2d(cx, cy);
//     this.deltaStart = new Translation2d(center, start);
//     this.deltaEnd = new Translation2d(center, end);
//     this.maxSpeed = maxSpeed;
//     extrapolateLookahead = false;
//     isLine = false;
//     createMotionProfiler(startState, endSpeed);
//   }
