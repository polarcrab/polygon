#ifndef HULL_H
#define HULL_H

#include <vector>

#include "common.h"
#include "constants.h"

PointVector getConvexHull(PointVector& ip);
std::vector<Edge> getConcaveHull(PointVector& ip, double N = INF);

#endif