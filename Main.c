#include <stdio.h>
#include "Path.h"

void main() {

    int i;
    int size = 3;
    // FeederToCargoShip1Path
    waypoint_t wp1 = {  0,   0,  0, 120}; 
    waypoint_t wp2 = { 50,  50,  40, 120};
    waypoint_t wp3 = { 0, 100,  0, 100};
    waypoint_t *wps[3] = {&wp1, &wp2, &wp3};

    BuildPathFromWaypoints( wps, size );

}

