#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include "Constants.h"
#include "Geometry.h"
#include "Path.h"


/******************************************************************************************************************************** 
**  ExtrapolateLast
**
**      Input:
**
**      Output:
**
********************************************************************************************************************************/
void ExtrapolateLast (pathSegmentsList_t *segments) {
    segments->tail->segment.extrapolateLookahead = 1;
}  


/******************************************************************************************************************************** 
**  GetLastMotionState
**
**      Input:
**
**      Output:
**
********************************************************************************************************************************/
motionState_t GetLastMotionState (pathSegmentsList_t *segments) {
    motionState_t rv;

    if ( segments->length > 0 ) {
        rv.t = 0.0;
        rv.pos = 0.0;
        rv.vel = segments->tail->segment.speedController->tail->segment.end.vel;
        rv.acc = segments->tail->segment.speedController->tail->segment.end.acc;
    } else {
        rv.t = 0.0;
        rv.pos = 0.0;
        rv.vel = 0.0;
        rv.acc = 0.0;
    }
    
    return rv;
}


/******************************************************************************************************************************** 
**  CheckSegmentDone
**
**      Input:
**
**      Output: 
**
********************************************************************************************************************************/
void CheckSegmentDone (pathSegmentsList_t *segments, translation2d_t *closestPoint) {
    pathSegmentNode_t *nextSgmentNode, *removeSegmentNode;
    double remainingDist;

    remainingDist = GetRemainingDistance( &segments->head->segment, closestPoint );
    if (remainingDist < kSegmentCompletionTolerance) {
        removeSegmentNode = segments->head;
        nextSgmentNode = segments->head->next;
        nextSgmentNode->prev = NULL;
        segments->head = nextSgmentNode;
        segments->length -= 1;
        ClearProfile ( removeSegmentNode->segment.speedController );
        free( removeSegmentNode->segment.speedController );
        free ( removeSegmentNode );
    }
}

/******************************************************************************************************************************** 
**  GetTargetPoint
**
**      Input:  Input the current 2D translational position of the robot.
**
**      Output: This function will calculate and return the data for targetPoint_t.
**
**                  closestPoint:                   The point on the segment closest to the robot.
**                  closestPointDistance_in:        The distance from the robot to the closest point.
**                  closestPointSpeed_ips:          The target speed at closest point on the segment.
**                  remainingSegmentDistance_in:    The distance from the closest point to the end of the segment.
**                  remainingPathDistance_in:       The distance from the closest point to the end of the path.
**                  maxSpeed_ips:                   The max speed of the lookahead point(segment).
**                  lookaheadPoint:                 The point at a lookahead distance down the path.
**                  lookaheadPointSpeed_ips         The target speed at the lookahead point.
**
********************************************************************************************************************************/
targetPoint_t GetTargetPoint (pathSegmentsList_t *segments, lookahead_t *lookahead, translation2d_t *robotPosition) {
    pathSegmentNode_t *segmentNode;    
    pathSegment_t currentSegment;
    targetPoint_t targetPoint;
    translation2d_t closestPointDistance_in;
    double lookaheadDistance, length;
    int numSegments = 1;

    currentSegment = segments->head->segment;

    targetPoint.closestPoint = GetClosestPoint( &currentSegment, robotPosition );
    closestPointDistance_in = TranslationDelta( robotPosition, &targetPoint.closestPoint );
    targetPoint.closestPointDistance_in = TranslationNormal( &closestPointDistance_in );
    targetPoint.closestPointSpeed_ips = GetSpeedByDistance( &currentSegment, GetLength( &currentSegment ) - targetPoint.remainingSegmentDistance_in );
    targetPoint.remainingSegmentDistance_in = GetRemainingDistance( &currentSegment, &targetPoint.closestPoint );
    
    targetPoint.remainingPathDistance_in = targetPoint.remainingSegmentDistance_in;
    segmentNode = segments->head->next;
    while (segmentNode != NULL) {
        targetPoint.remainingPathDistance_in += GetLength( &segmentNode->segment );
        segmentNode = segmentNode->next;
        ++numSegments;
    }

    // Calclate the lookahead distance as a funtion of target speed at the closest point on the segment
    lookaheadDistance = GetLookaheadForSpeed( lookahead, targetPoint.closestPointSpeed_ips) + targetPoint.closestPointDistance_in;

    // The lookahead distance extends beyond the end of the current segment, find which segment the lookahead distance ends in
    if ( targetPoint.remainingSegmentDistance_in < lookaheadDistance && numSegments > 1 ) {
        lookaheadDistance -= targetPoint.remainingSegmentDistance_in;
        segmentNode = segments->head->next;
        while (segmentNode != NULL) {
            currentSegment = segmentNode->segment;
            length = GetLength( &segmentNode->segment );
            if ( length < lookaheadDistance && segmentNode->next ) {
                lookaheadDistance -= length;
            } else {
                break;
            }
            segmentNode = segmentNode->next;
        }

    // The lookahead is within the length of the current segment.
    } else {
        lookaheadDistance += GetLength( &currentSegment ) - targetPoint.remainingSegmentDistance_in;
    }
    targetPoint.maxSpeed_ips = currentSegment.maxSpeed_ips;
    targetPoint.lookaheadPoint = GetPointByDistance( &currentSegment, lookaheadDistance );
    targetPoint.lookaheadPointSpeed_ips = GetSpeedByDistance( &currentSegment, lookaheadDistance );
    CheckSegmentDone( segments, &targetPoint.closestPoint );

    return targetPoint;
}




//   /****************************************************************************************************************************** 
//   **  verifySpeeds
//   **    Ensures that all speeds in the path are attainable and robot can slow down in time.
//   ******************************************************************************************************************************/
//   public void verifySpeeds() {
//     double maxStartSpeed = 0.0;
//     double[] startSpeeds = new double[segments.size() + 1];
//     startSpeeds[segments.size()] = 0.0;
//     for (int i = segments.size() - 1; i >= 0; i--) {
//       PathSegment segment = segments.get(i);
//       maxStartSpeed += Math.sqrt(maxStartSpeed * maxStartSpeed + 2 * RobotMap.kPathFollowingMaxAccel * segment.getLength());
//       startSpeeds[i] = segment.getStartState().vel();
//       if (startSpeeds[i] > maxStartSpeed) {
//         startSpeeds[i] = maxStartSpeed;
//       }
//       maxStartSpeed = startSpeeds[i];
//     }
//     for (int i = 0; i < segments.size(); i++) {
//       PathSegment segment = segments.get(i);
//       double endSpeed = startSpeeds[i + 1];
//       MotionState startState = (i > 0) ? segments.get(i - 1).getEndState() : new MotionState(0, 0, 0, 0);
//       startState = new MotionState(0, 0, startState.vel(), startState.vel());
//       segment.createMotionProfiler(startState, endSpeed);
//     }
//   }
  





















































