#include <cmath>
#include <iostream>
#include <map>

#include "include/common.h"
#include "include/constants.h"
#include "include/utils.h"

bool pointComparator(const Point & a, const Point & b)
{
    if (a.second > b.second)
        return false;
    if (a.second == b.second && a.first > b.first)
        return false;
    return true;
}

double calSlope(const Point &a, const Point &b)
{
    double x1 = a.first, y1 = a.second;
    double x2 = b.first, y2 = b.second;
    double slope;
    if (x1 == x2)
        slope = INF;
    else
        slope = (y1 - y2) / (x1 - x2);
    return slope;
}

double calAngleWithXAxis(const Point &a, const Point &b)
{
    double x1 = a.first, y1 = a.second;
    double x2 = b.first, y2 = b.second;
    double angle = atan2(y1 - y2, x1 - x2);
    // std::cout << angle << std::endl;
    return angle >= 0 ? angle : angle + 2 * M_PIl;
}

bool isClose(double val1, double val2)
{
    if (int(val1 * 1000) == int(val2 * 1000))
        return true;
    return false;
}

double calLength(Edge edge)
{
    Point p1 = edge.first;
    Point p2 = edge.second;
    double x1_x2 = p1.first - p2.first;
    double y1_y2 = p1.second - p2.second;
    return sqrt(x1_x2 * x1_x2 + y1_y2 * y1_y2);
}

double calDecisionLength(Point &p, Edge &e)
{

    return std::min(
               calLength(std::make_pair(p, e.first)),
               calLength(std::make_pair(p, e.second))
           );
}

bool inRange(double x1, double y1, double x2, double y2,
             double x0, double y0)
{
    double dx =     x2 - x1;
    double dy = y2 - y1;
    double innerProduct = (x0 - x1) * dx + (y0 - y1) * dy;
    return 0 <= innerProduct && innerProduct <= dx * dx + dy * dy;
}

double linePointDistance(Point &p, Edge e)
{
    Point p1 = e.first;
    Point p2 = e.second;
    double x0 = p.first, y0 = p.second;
    double x1 = p1.first, y1 = p1.second;
    double x2 = p2.first, y2 = p2.second;
    if (inRange(x1, y1, x2, y2, x0, y0))
        return  abs((y2 - y1) * x0 - (x2 - x1) * y0 + x2 * y1 - y2 * x1) / sqrt((y2 - y1) * (y2 - y1) + (x2 - x1) * (x2 - x1));
    return INF;
}

Point closestPoint(Edge &e, PointVector& ip,
                   std::map<Point, std::pair<Point, Point>> neighbours)
{
    Point closest = std::make_pair(INF, INF);
    double closestDis = INF;
    for (int i = 0; i < int(ip.size()); i++)
    {
        if (neighbours.find(ip[i]) == neighbours.end())
        {
            double dis = linePointDistance(ip[i], e);
            double neighbourDis1 = linePointDistance(
                                       ip[i],
                                       std::make_pair
                                       (
                                           e.first,
                                           neighbours[e.first].first == e.second ? neighbours[e.first].second : neighbours[e.first].first
                                       )
                                   );
            double neighbourDis2 = linePointDistance(ip[i], std::make_pair(e.second, neighbours[e.second].first == e.first ? neighbours[e.second].second : neighbours[e.second].first));

            if (dis < closestDis && dis < neighbourDis1 && dis < neighbourDis2)
            {
                closest = ip[i];
                closestDis = dis;
            }
        }
    }
    return closest;
}

// double calAngle(const Point& p1, const Point& p2, const Point& p3)
// {
//     DirVector v1 = std::make_pair(p3.first - p2.first, p3.second - p2.second);
//     DirVector v2 = std::make_pair(p1.first - p2.first, p1.second - p2.second);
//     double v1Mag = sqrt(v1.first * v1.first + v1.second * v1.second);
//     double v2Mag = sqrt(v2.first * v2.first + v2.second * v2.second);
//     double v1v2dot = v1.first * v2.first + v1.second * v2.second;
//     double v1v2cross = v1.first * v2.second - v2.first * v1.second;
//     double cosTheta = v1v2dot / (v1Mag * v2Mag);
//     double sinTheta = v1v2cross / (v1Mag * v2Mag);
//     double angle1 = asin(sinTheta);
//     double angle2 = angle1 > 0 ? M_PIl - angle1 : -M_PIl - angle1;
//     double cos1 = cos(angle1);
//     double res;
//     std::cout<<angle1<<" "<<angle2<<std::endl;
//     if (isClose(cos1, cosTheta))
//         res = angle2;
//     else
//         res = angle1;
//     return res > 0 ? res : 2 * M_PIl + res;
// }

double calAngle(const Point& p1, const Point& p2, const Point& p3)
{
    /*
    * Calculates to angle anti-clockwise
    */
    double x1 = p1.first - p2.first, y1 = p1.second - p2.second;
    double x2 = p3.first - p2.first, y2 = p3.second - p2.second;
    double dot = x1 * x2 + y1 * y2; // dot product between [x1, y1] and [x2, y2]
    double det = x1 * y2 - y1 * x2; // determinant
    double angle = atan2(det, dot);
    // std::cout<<angle<<std::endl;
    return angle >= 0 ? angle : angle + 2 * M_PIl;
}

double calDis(const Point& p1, const Point& p2)
{
    double x1_x2 = p1.first - p2.first;
    double y1_y2 = p1.second - p2.second;
    return sqrt(x1_x2 * x1_x2 + y1_y2 * y1_y2);
}

AngleSort::AngleSort(const Point &refPoint)
{
    this->refPoint = refPoint;
}
bool AngleSort::operator()(const Point &a, const Point &b)
{
    double angle_ar = calAngleWithXAxis(a, refPoint);
    double angle_br = calAngleWithXAxis(b, refPoint);
    // if (isClose(angle_ar, angle_br))
    // {
    //     if (calDis(refPoint, a) > calDis(refPoint, b))
    //         return false;
    //     return true;
    // }
    return angle_ar < angle_br;
}
