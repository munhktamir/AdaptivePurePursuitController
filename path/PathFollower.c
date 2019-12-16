#include <math.h>
#include "Geometry.h"
#include "Motion.h"
#include "Path.h"


twist2d_t GetPathFollowerUpdate (pathFollower_t *pathFollower, double t, double displacement, double velocity, transform2d_t *robotPose) {
    twist2d_t rv;
    steeringComamnd_t steeringCmd;
    motionProfileConstraints_t constraints;
    motionProfileGoal_t goal;
    motionState_t lastMotionState, setpoint;
    double velocityCmd, curvature, dTheta_rad, absVelocitySetpoint, scale;

    if ( pathFollower->steeringController.atEndOfPath ) {
        steeringCmd = GetSteeringUpdate( &pathFollower->steeringController, robotPose );
        pathFollower->crossTrackError = steeringCmd.crossTrackError;
        pathFollower->lastSteeringDelta = steeringCmd.delta;
        goal.pos = displacement + steeringCmd.delta.dx_in;
        goal.maxAbsVel = fabs( steeringCmd.endSpeed_ips );
        goal.completionBehavior = VIOLATE_MAX_ACCEL;
        goal.posTolerance = pathFollower->goalPosTolerance;
        goal.velTolerance = pathFollower->goalVelTolerance;
        constraints.maxAbsVel = fmin( pathFollower->maxProfileVel, steeringCmd.maxSpeed_ips );
        constraints.maxAbsAcc = pathFollower->maxProfileAcc;
        SetProfileFollowerGoalAndConstraints( &pathFollower->velocityController, &goal, &constraints );
        if ( steeringCmd.remainingPathLength < pathFollower->stopSteeringDistance ) {
            pathFollower->doneSteering = 1;

        }
    }

    lastMotionState.t = t;
    lastMotionState.pos = displacement;
    lastMotionState.vel = velocity;
    lastMotionState.acc = 0.0;
    velocityCmd = ProfileFollowerUpdate( &pathFollower->velocityController, &lastMotionState, t );
    pathFollower->alongTrackError = pathFollower->velocityController.latestPosError;
    curvature = pathFollower->lastSteeringDelta.dtheta_rad / pathFollower->lastSteeringDelta.dx_in;
    dTheta_rad = pathFollower->lastSteeringDelta.dtheta_rad;
    if ( !isnan( curvature ) && fabs( curvature ) < 1E6 ) {
        // Regenerate angular velocity command from adjusted curvature.
        setpoint = GetProfileSetpoint( &pathFollower->velocityController );
        absVelocitySetpoint = fabs( setpoint.vel );
        dTheta_rad = pathFollower->lastSteeringDelta.dx_in * curvature * ( 1.0 + pathFollower->inertiaGain * absVelocitySetpoint );
    }
    scale = velocityCmd / pathFollower->lastSteeringDelta.dx_in;
    rv.dtheta_rad = dTheta_rad * scale;
    rv.dx_in = pathFollower->lastSteeringDelta.dx_in * scale;
    rv.dy_in = 0.0;

    return rv;
}
  
int PathFollowerIsFinished (pathFollower_t *pathFollower) {
    int rv;

    rv =  ( pathFollower->steeringController.atEndOfPath && IsProfileFinished( &pathFollower->velocityController ) && IsProfileOnTarget( &pathFollower->velocityController ) ) || pathFollower->overrideFinished;
    return rv;
}

//   /**
//    * Create a new PathFollower for a given path.
//    */
//   public PathFollower(Path path, boolean reversed, Parameters parameters) {
//     mSteeringController = new AdaptivePurePursuitController(path, reversed, parameters.lookahead);
//     mLastSteeringDelta = Twist2d.identity();
//     mVelocityController = new ProfileFollower(parameters.profile_kp, parameters.profile_ki, parameters.profile_kv, parameters.profile_kffv, parameters.profile_kffa);
//     mVelocityController.setConstraints(new MotionProfileConstraints(parameters.profile_max_abs_vel, parameters.profile_max_abs_acc));
//     mMaxProfileVel = parameters.profile_max_abs_vel;
//     mMaxProfileAcc = parameters.profile_max_abs_acc;
//     mGoalPosTolerance = parameters.goal_pos_tolerance;
//     mGoalVelTolerance = parameters.goal_vel_tolerance;
//     mInertiaGain = parameters.inertia_gain;
//     mStopSteeringDistance = parameters.stop_steering_distance;
//   }