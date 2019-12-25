#ifndef CONSTANTS_H
#define CONSTANTS_H

static const double kEpsilon = 1e-6;
static const double kSegmentCompletionTolerance = 0.1;


// Lookahead
static const double kMinLookAhead_in = 12.0;
static const double kMaxLookAhead_in = 36.0;
static const double kMinLookAheadSpeed_ips = 4.0;
static const double kMaxLookAheadSpeed_ips = 12.0;  // 120.0

// Profile following
static const double kPathFollowingProfileKp = 0.0;           // 5.0
static const double kPathFollowingProfileKi = 0.0;           // 0.03
static const double kPathFollowingProfileKv = 0.0;           // 0.02
static const double kPathFollowingProfileKffv = 0.0;         // 1.0;
static const double kPathFollowingProfileKffa = 0.0;         //0.05;

static const double kPathFollowingMaxAccel = 10.0;

//   public static final double kMinLookAhead = 12.0;                    // inches  
//   public static final double kMaxLookAhead = 36.0;                    // inches 
//   public static final double kMinLookAheadSpeed = 4.0;                // inches per second
//   public static final double kMaxLookAheadSpeed = 12.0;//120.0;       // inches per second
  
//   public static final double kInertiaSteeringGain = 0.0;              // angular velocity command is multiplied by this gain (speed in inches per second)
  
//   public static final double kPathFollowingProfileKp = 0.5;           // 5.0
//   public static final double kPathFollowingProfileKi = 0.0;           // 0.03
//   public static final double kPathFollowingProfileKv = 0.0;           // 0.02
//   public static final double kPathFollowingProfileKffv = 0.0;//1.0;
//   public static final double kPathFollowingProfileKffa = 0.0;//0.05;
//   public static final double kPathFollowingMaxVel = 12.0;//120.0;            // inches per second 
//   public static final double kPathFollowingMaxAccel = 12.0;//120.0;          // inches per second^2
//   public static final double kPathFollowingGoalPosTolerance = 0.75;
//   public static final double kPathFollowingGoalVelTolerance = 12.0;
//   public static final double kPathStopSteeringDistance = 9.0;
  
//   public static final double kSegmentCompletionTolerance = 0.1;       // inches  
//   public static final double kDriveHighGearMaxSetpoint = 12.0;//120.0;



#endif