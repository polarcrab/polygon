#ifndef UTILS_H
#define UTILS_H
#include <map>
#include <list>

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
bool inRange(double x1, double y1, double x2, double y2,
             double x0, double y0);
bool checkIntersections(std::list<Edge> edges, Edge e1, std::map<Point, std::pair<Point, Point>> neighbours);
class AngleSort
{
	Point refPoint;
public:
	AngleSort(const Point &refPoint);
	bool operator()(const Point &a, const Point &b);
};

#endif