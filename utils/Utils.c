#include "Utils.h"

double SignNum (double num) {
    return (num < 0) ? -1.0 : 1.0;     
}

int EpsilonEquals (double a, double b, double epsilon) {
    return ( a - epsilon <= b ) && ( a + epsilon >= b );
}