#ifndef CONVEXHULL_H
#define CONVEXHULL_H

#include <vector>

#include "common.h"
#include "constants.h"

PointVector getConvexHull(PointVector& ip);
std::vector<Edge> getConcaveHull(PointVector& ip, double N = INF);

#endif