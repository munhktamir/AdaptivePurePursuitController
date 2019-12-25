#include <check.h>
#include "test_MotionState.h"
#include "test_MotionSegment.h"
#include "test_MotionProfileGoal.h"
#include "test_MotionProfile.h"
#include "test_MotionProfileGenerator.h"
#include "test_SetpointGenerator.h"
#include "test_ProfileFollower.h"


int main(void) {
    int no_failed = 0;                   
    SRunner *runner;                     

    runner = srunner_create(motionState_suite());
    srunner_add_suite(runner, motionSegment_suite());  
    srunner_add_suite(runner, motionProfileGoal_suite());
    srunner_add_suite(runner, motionProfile_suite());
    srunner_add_suite(runner, motionProfileGenerator_suite());
    srunner_add_suite(runner, setpointGenerator_suite());
    srunner_add_suite(runner, profileFollower_suite());
    srunner_set_fork_status(runner, CK_NOFORK);
    srunner_run_all(runner, CK_NORMAL);  
    no_failed = srunner_ntests_failed(runner); 
    srunner_free(runner);                      

    return (no_failed == 0) ? 0 : 1;  
}
