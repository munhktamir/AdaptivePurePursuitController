// /******************************************************************************************************************************** 
// ** AdaptivePurePursuitController
// **   Implements an adaptive pure pursuit controller. See:
// **   https://www.ri.cmu.edu/pub_files/pub1/kelly_alonzo_1994_4/kelly_alonzo_1994_4.pdf
// ********************************************************************************************************************************/

#include <math.h>
#include "Utils.h"
#include "Geometry.h"
#include "Path.h"
#include "RobotStateEstimator.h"


/******************************************************************************************************************************** 
**  GetCenter
**
**      Input:
**
**      Output: Returns the center of the circle joining the lookahead point and the robots position.
**
********************************************************************************************************************************/
translation2d_t GetCenter (transform2d_t *robotPose, translation2d_t *lookaheadPoint) {
    translation2d_t poseToPointHalfway, normalTrans, center;
    rotation2d_t normalRot;
    transform2d_t perpendicularBisector, normalFromPose, perpendicularBisectorNormal;

    poseToPointHalfway = TranslationInterpolate( &robotPose->translation, lookaheadPoint, 0.5 );
    normalTrans = TranslationInverse(  &robotPose->translation );
    normalTrans = TranslateAbyB( &normalTrans, &poseToPointHalfway );
    normalRot = TranslationDirection( &normalTrans );
    normalRot = RotationNormal( &normalRot );

    perpendicularBisector.translation = poseToPointHalfway;
    perpendicularBisector.rotation = normalRot;
    perpendicularBisectorNormal = TransformNormal( &perpendicularBisector );

    normalFromPose.translation = robotPose->translation;
    normalFromPose.rotation = RotationNormal( &robotPose->rotation );
  
    // Center is the poseToPointHalfway
    if ( IsColinear( &normalFromPose, &perpendicularBisectorNormal ) ) {
        return poseToPointHalfway;
    } else {
        center = Intersection( &normalFromPose, &perpendicularBisector );
        return center;
    }
}


/******************************************************************************************************************************** 
**  GetDirection
**
**      Input:
**
**      Output: The direction of the robot should turn, -1 is left, +1 is right.
**
********************************************************************************************************************************/
double GetDirection (transform2d_t *robotPose, translation2d_t *point) {
    translation2d_t robotPoseToPoint, robotRotToTrans;
    double cross;

    robotPoseToPoint = TranslationDelta( point, &robotPose->translation );
    robotRotToTrans = RotationToTranslation( &robotPose->rotation );
    cross = TranslationCross( &robotRotToTrans, &robotPoseToPoint );
    return (cross < 0.0) ? -1.0 : 1.0;
}

/******************************************************************************************************************************** 
**  GetLength
**
**      Input:
**
**      Output:
**
********************************************************************************************************************************/
double GetSteeringArcLength (transform2d_t *robotPose, translation2d_t *point, transform2d_t *center, double radius) {
    translation2d_t centerToPoint, centerToRobotPose, robotPoseToPoint, robotPoseRotNormal;
    rotation2d_t rot;
    double c, angle, length;
    
    if ( radius < 1E6 ) {
        centerToPoint = TranslationDelta( &center->translation, point );
        centerToRobotPose = TranslationDelta( &center->translation, &robotPose->translation );

        // If the point is behind pose, we want the opposite of this angle. To determine if the point is behind,
        // check the sign of the cross-product between the normal vector and the vector from pose to point.
        robotPoseToPoint = TranslationDelta( point, &robotPose->translation );
        rot = RotationNormal( &robotPose->rotation );
        robotPoseRotNormal = RotationToTranslation( &rot );
        //c = TranslationCross( &RotationNormal, &robotPoseToPoint );
        angle = TranslationGetAngle( &centerToRobotPose, &centerToPoint );
        length = radius * ( SignNum( c ) ? 2.0 * 3.14159265358979323846 - fabs(angle) : fabs(angle) );

    } else {
        length = TranslationNormal( &robotPoseToPoint );
    }

    return length;
}


/******************************************************************************************************************************** 
**  Update
**
**      Input:
**
**      Output:
**
********************************************************************************************************************************/
steeringComamnd_t GetSteeringUpdate (adaptivePurePursuitController_t *controller, transform2d_t *robotPose) {
    steeringArc_t arc;
    steeringComamnd_t steeringCommand;
    targetPoint_t targetPoint;
    translation2d_t radius;
    double scale = 1.0;

    targetPoint = GetTargetPoint( controller->path, &controller->lookahead, &robotPose->translation );

    if ( controller->atEndOfPath ) {
        steeringCommand.delta.dx_in = 0.0;
        steeringCommand.delta.dy_in = 0.0;
        steeringCommand.delta.dtheta_rad = 0.0; 
        steeringCommand.crossTrackError = targetPoint.closestPointDistance_in;
        steeringCommand.maxSpeed_ips = targetPoint.maxSpeed_ips;
        steeringCommand.endSpeed_ips = 0.0;
        steeringCommand.lookaheadPoint = targetPoint.lookaheadPoint;
        steeringCommand.remainingPathLength = targetPoint.remainingPathDistance_in;

    } else {
        arc.center = GetCenter( robotPose, &targetPoint.lookaheadPoint );
        radius = TranslationDelta(&arc.center, &targetPoint.lookaheadPoint);
        arc.radius = TranslationNormal( &radius );
        arc.length = GetSteeringArcLength( robotPose, &targetPoint.lookaheadPoint, &arc.center, arc.radius);
        
        // Ensure we don't overshoot the end of the path (once the lookahead speed drops to zero).
        if ( targetPoint.lookaheadPointSpeed_ips < 1E-6 && targetPoint.remainingPathDistance_in < arc.length ) {
            scale = fmax( 0.0, targetPoint.remainingPathDistance_in / arc.length );
            controller->atEndOfPath = 1;

        } else {
            controller->atEndOfPath = 0;

        }
        steeringCommand.delta.dx_in = scale * arc.length;
        steeringCommand.delta.dy_in = 0.0;
        steeringCommand.delta.dtheta_rad = arc.length * GetDirection( robotPose, &targetPoint.lookaheadPoint ) * fabs( scale ) / arc.radius; 
        steeringCommand.crossTrackError = targetPoint.closestPointDistance_in;
        steeringCommand.maxSpeed_ips = targetPoint.maxSpeed_ips;
        steeringCommand.endSpeed_ips = targetPoint.lookaheadPointSpeed_ips * SignNum( scale );
        steeringCommand.lookaheadPoint = targetPoint.lookaheadPoint;
        steeringCommand.remainingPathLength = targetPoint.remainingPathDistance_in;
    }

    return steeringCommand;
}

