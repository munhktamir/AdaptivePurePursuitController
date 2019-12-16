#ifndef PATH_H
#define PATH_H

#include "Geometry.h"
#include "Motion.h"

typedef struct waypoint {
    translation2d_t position;
    double radius;
    double speed_ips;
} waypoint_t;

typedef struct line {
    waypoint_t waypointA;
    waypoint_t waypointB;        
    translation2d_t start;
    translation2d_t end;
    translation2d_t slope;
    double speed_ips;
} line_t;

typedef struct arc {
    line_t lineA;
    line_t lineB;
    translation2d_t center;
    double radius;
    double speed_ips;
} arc_t;
 
typedef struct pathSegment {
    translation2d_t start;
    translation2d_t end;
    translation2d_t center;
    translation2d_t deltaStart;
    translation2d_t deltaEnd;
    double maxSpeed_ips;
    int isLine;
    motionProfileList_t *speedController;
    int extrapolateLookahead;
} pathSegment_t;  

typedef struct pathSegmentNode {
    pathSegment_t segment;
    struct pathSegmentNode *prev;
    struct pathSegmentNode *next;
} pathSegmentNode_t;

typedef struct pathSegmentsList {
    pathSegmentNode_t *head;
    pathSegmentNode_t *tail;
    int length;
} pathSegmentsList_t;

typedef struct targetPoint {
    translation2d_t closestPoint;
    double closestPointDistance_in;
    double closestPointSpeed_ips;
    double remainingSegmentDistance_in;
    double remainingPathDistance_in;
    double maxSpeed_ips;
    translation2d_t lookaheadPoint;
    double lookaheadPointSpeed_ips;
} targetPoint_t;

typedef struct lookahead {
    double minDistance_in;
    double maxDistance_in;
    double minSpeed_ips;
    double maxSpeed_ips;
    double deltaDistance_in;
    double deltaSpeed_ips;
} lookahead_t;

typedef struct steeringArc {
    translation2d_t center;
    double radius;
    double length;
} steeringArc_t;

typedef struct steeringCommand {
    twist2d_t delta;
    double crossTrackError;
    double maxSpeed_ips;
    double endSpeed_ips;
    translation2d_t lookaheadPoint;
    double remainingPathLength;
} steeringComamnd_t;

typedef struct adaptivePurePursuitController {
  pathSegmentsList_t *path;
  int atEndOfPath;
  int reversed;
  lookahead_t lookahead;
} adaptivePurePursuitController_t;

typedef struct pathFollowerParams {
    lookahead_t lookahead;
    double inertiaGain;
    double profile_kp;
    double profile_ki;
    double profile_kv;
    double profile_kffv;
    double profile_kffa;
    double profile_max_abs_vel;
    double profile_max_abs_acc;
    double goal_pos_tolerance;
    double goal_vel_tolerance;
    double stop_steering_distance;
} pathFollowerParams_t;

typedef struct pathFollower {
    adaptivePurePursuitController_t steeringController;
    profileFollower_t velocityController;
    twist2d_t lastSteeringDelta;
    double inertiaGain;
    int overrideFinished;
    int doneSteering;
    //DebugOutput mDebugOutput = new DebugOutput();
    double maxProfileVel;
    double maxProfileAcc;
    double goalPosTolerance;
    double goalVelTolerance;
    double stopSteeringDistance;
    double crossTrackError;
    double alongTrackError;
} pathFollower_t;


// PathSegment.c
motionProfileList_t CreateMotionProfiler (motionState_t *startState, double endSpeed, double maxSpeed, double length);
double GetLength (pathSegment_t *segment);
translation2d_t GetClosestPoint (pathSegment_t *segment, translation2d_t *robotPosition);
double GetRemainingDistance (pathSegment_t *segment, translation2d_t *position);
translation2d_t GetPointByDistance (pathSegment_t *segment, double dist);
double GetDistanceTravelled (pathSegment_t *segment, translation2d_t *robotPosition);
double GetSpeedByDistance(pathSegment_t *segment, double dist);
double GetSpeedByClosePoint (pathSegment_t *segment, translation2d_t *robotPosition);

// PathBuilder.c
pathSegmentsList_t BuildPathFromWaypoints (waypoint_t *wps[], int size);
line_t CreateLine (waypoint_t *a, waypoint_t *b);
translation2d_t Intersect (line_t *lineA, line_t *lineB);
arc_t CreateArc(waypoint_t *a, waypoint_t *b, waypoint_t *c);
void AddPathSegment (pathSegmentsList_t *segments, pathSegmentNode_t *segment);


// Lookahead.c
double GetLookaheadForSpeed (lookahead_t *lookahead, double speed_ips);

// Path.c
targetPoint_t GetTargetPoint (pathSegmentsList_t *segments, lookahead_t *lookahead, translation2d_t *robotPosition);
void ExtrapolateLast (pathSegmentsList_t *segments);
motionState_t GetLastMotionState (pathSegmentsList_t *segments);
void CheckSegmentDone (pathSegmentsList_t *segments, translation2d_t *closestPoint);


// AdaptivePurePursuit.c
steeringComamnd_t GetSteeringUpdate (adaptivePurePursuitController_t *controller, transform2d_t *robotPose);
double GetSteeringArcLength (transform2d_t *robotPose, translation2d_t *point, transform2d_t *center, double radius);
translation2d_t GetCenter (transform2d_t *robotPose, translation2d_t *lookaheadPoint);
double GetDirection (transform2d_t *robotPose, translation2d_t *point);
double GetSteeringArcLength (transform2d_t *robotPose, translation2d_t *point, transform2d_t *center, double radius);


// PathFollower.c



#endif