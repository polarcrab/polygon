#ifndef UTILS_H
#define UTILS_H
#include <map>

#include "common.h"
bool pointComparator(const Point & a, const Point & b);
double calSlope(const Point &a, const Point &b);
bool isClose(double val1, double val2);
double calLength(Edge edge);
double calDecisionLength(Point &p, Edge &e);
Point closestPoint(Edge &e, PointVector& ip,
                   std::map<Point,
                   std::pair<Point, Point>> neighbours);
double calAngle(const Point& p1, const Point& p2, const Point& p3);
double calAngleWithXAxis(const Point &a, const Point &b);
double calDis(const Point& p1, const Point& p2);
class AngleSort
{
    Point refPoint;
public:
    AngleSort(const Point &refPoint);
    bool operator()(const Point &a, const Point &b);
};

#endif