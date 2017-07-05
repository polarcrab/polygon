/**
    Header file from hull.cpp

    @author Harmandeep Singh
    @email harmandeep DOT singh1 AT delhivery DOT com
    @date 3 July 2017
*/

#ifndef HULL_H
#define HULL_H

#include <vector>

#include "common.h"
#include "constants.h"

PointVector get_convex_hull(PointVector& ip);
std::vector<Edge> get_concave_hull(PointVector& ip, double N = INF);

#endif