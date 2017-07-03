#ifndef UTILS_H
#define UTILS_H
#include <map>
#include <list>

#include "common.h"

bool point_comparator(const Point & a, const Point & b);
double cal_slope(const Point &a, const Point &b);
bool is_close(double val1, double val2);
double cal_length(Edge edge);
double cal_decision_length(Point &p, Edge &e);
Point closest_point(Edge &e, PointVector& ip,
                   std::map<Point,
                   std::pair<Point, Point>> &neighbours);
double cal_angle(const Point& p1, const Point& p2, const Point& p3);
double cal_angle_with_X_axis(const Point &a, const Point &b);
double cal_dis(const Point& p1, const Point& p2);
bool in_range(double x1, double y1, double x2, double y2,
             double x0, double y0);
bool check_intersections(std::list<Edge> edges, Edge e1);
class AngleSort
{
	Point refPoint;
public:
	AngleSort(const Point &refPoint);
	bool operator()(const Point &a, const Point &b);
};

#endif