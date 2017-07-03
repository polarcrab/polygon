#include <cmath>
#include <iostream>
#include <list>
#include <map>

#include "common.h"
#include "constants.h"
#include "utils.h"

Line get_line(const Point& p1, const Point& p2)
{
    double slope = cal_slope(p1, p2);
    return std::make_pair(slope, p1.second - p1.first * slope);
}

bool get_sign(Line line, Point p)
{
    double val = p.second - line.first * p.first - line.second;
    if (val < 0)
        return false;
    return true;
}

bool check_signs(Point p1, Point p2, Point q1, Point q2)
{
    /*
    Will return true if lines intersect
    */
    Line line1 = get_line(p1, p2);
    Line line2 = get_line(q1, q2);
    bool s1 = get_sign(line1, q1);
    bool s2 = get_sign(line1, q2);
    bool s3 = get_sign(line2, p1);
    bool s4 = get_sign(line2, p2);
    if (s1 != s2 && s3 != s4)
        return true;
    return false;
}

bool check_intersections(std::list<Edge> edges, Edge e1)
{
    for (std::list<Edge>::iterator itr = edges.begin();
            itr != edges.end(); itr++)
    {
        if (*itr == e1 ||
                (
                    (*itr).first == e1.first ||
                    (*itr).first == e1.second ||
                    (*itr).second == e1.first ||
                    (*itr).second == e1.second
                )
           )
            continue;
        if (check_signs((*itr).first, (*itr).second, e1.first, e1.second))
            return true;
    }
    return false;
}
bool point_comparator(const Point & a, const Point & b)
{
    if (a.second > b.second)
        return false;
    if (a.second == b.second && a.first > b.first)
        return false;
    return true;
}

double cal_slope(const Point &a, const Point &b)
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

double cal_angle_with_X_axis(const Point &a, const Point &b)
{
    double x1 = a.first, y1 = a.second;
    double x2 = b.first, y2 = b.second;
    double angle = atan2(y1 - y2, x1 - x2);
    // std::cout << angle << std::endl;
    return angle >= 0 ? angle : angle + 2 * M_PIl;
}

bool is_close(double val1, double val2)
{
    if (int(val1 * 1000) == int(val2 * 1000))
        return true;
    return false;
}

double cal_length(Edge edge)
{
    Point p1 = edge.first;
    Point p2 = edge.second;
    double x1_x2 = p1.first - p2.first;
    double y1_y2 = p1.second - p2.second;
    return sqrt(x1_x2 * x1_x2 + y1_y2 * y1_y2);
}

double cal_decision_length(Point &p, Edge &e)
{

    return std::min(
               cal_length(std::make_pair(p, e.first)),
               cal_length(std::make_pair(p, e.second))
           );
}

bool in_range(double x1, double y1, double x2, double y2,
              double x0, double y0)
{
    double dx = x2 - x1;
    double dy = y2 - y1;
    double innerProduct = (x0 - x1) * dx + (y0 - y1) * dy;
    return 0 <= innerProduct && innerProduct <= dx * dx + dy * dy;
}

double line_point_distance(Point &p, Edge e)
{
    Point p1 = e.first;
    Point p2 = e.second;
    double x0 = p.first, y0 = p.second;
    double x1 = p1.first, y1 = p1.second;
    double x2 = p2.first, y2 = p2.second;
    if (in_range(x1, y1, x2, y2, x0, y0))
        return  abs((y2 - y1) * x0 - (x2 - x1) * y0 + x2 * y1 - y2 * x1) / sqrt((y2 - y1) * (y2 - y1) + (x2 - x1) * (x2 - x1));
    return INF;
}

Point closest_point(Edge &e, PointVector& ip,
                    std::map<Point, std::pair<Point, Point>> &neighbours)
{
    Point closest = std::make_pair(INF, INF);
    double closestDis = INF;
    for (int i = 0; i < int(ip.size()); i++)
    {
        if (neighbours.find(ip[i]) == neighbours.end())
        {
            double dis = line_point_distance(ip[i], e);
            if (dis < closestDis)
            {
                closest = ip[i];
                closestDis = dis;
            }
        }
    }
    double neighbourDis1 = line_point_distance(
                               closest,
                               std::make_pair(
                                   e.first, neighbours[e.first].first == e.second ? neighbours[e.first].second : neighbours[e.first].first
                               )
                           );
    double neighbourDis2 = line_point_distance(
                               closest,
                               std::make_pair(
                                   e.second, neighbours[e.second].first == e.first ? neighbours[e.second].second : neighbours[e.second].first
                               )
                           );
    if (closestDis < neighbourDis1 && closestDis < neighbourDis2)
        return closest;
    return std::make_pair(INF, INF);
}

// May be useful
// bool is_close(double val1, double val2)
// {
//     if (int(val1 * 1000) == int(val2 * 1000))
//         return true;
//     return false;
// }
// double cal_angle_2(const Point& p1, const Point& p2, const Point& p3)
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
//     if (is_close(cos1, cosTheta))
//         res = angle2;
//     else
//         res = angle1;
//     return res > 0 ? res : 2 * M_PIl + res;
// }

double cal_angle(const Point& p1, const Point& p2, const Point& p3)
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

double cal_dis(const Point& p1, const Point& p2)
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
    double angle_ar = cal_angle_with_X_axis(a, refPoint);
    double angle_br = cal_angle_with_X_axis(b, refPoint);
    return angle_ar < angle_br;
}
