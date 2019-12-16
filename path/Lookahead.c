#include "Path.h"


/******************************************************************************************************************************** 
**  GetLookaheadForSpeed
**
**      Input:
**
**      Output:
**
********************************************************************************************************************************/
double GetLookaheadForSpeed (lookahead_t *lookahead, double speed_ips) {
    double lookaheadDistance, rv;

    lookaheadDistance = lookahead->deltaDistance_in * (speed_ips - lookahead->minSpeed_ips) / lookahead->deltaSpeed_ips + lookahead->minDistance_in;
    rv = isnan( lookaheadDistance ) ? lookahead->minDistance_in : fmax( lookahead->minDistance_in, fmin( lookahead->maxDistance_in, lookaheadDistance ) );

    return rv;
}
  
