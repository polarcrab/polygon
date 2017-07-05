/**
    This file contains the utils functions.

    Most of the functions are geometry related, like finding the line point
    distance, calculating angle between 3 points etc.

    @author Harmandeep Singh
    @email harmandeep DOT singh1 AT delhivery DOT com
    @date 3 July 2017
*/


#include <cmath>
#include <iostream>
#include <list>
#include <map>

#include "common.h"
#include "constants.h"
#include "utils.h"

Line get_line(const Point& p1, const Point& p2)
{
    /**
        This function returns the equation of the line.
        Each line can be represented as
        y = mx+c
        So for a given line, this function will return the value of
        m and c as a pair.

        @param p1 First point of the two points for which the equation
                  of the line has to be calculated
        @param p1 Second point of the two points for which the equation
                  of the line has to be calculated
        @return m and c as `std::pair`
     */

    double slope = cal_slope(p1, p2);
    return std::make_pair(slope, p1.second - p1.first * slope);
}

bool get_sign(Line line, Point p)
{
    /**
        To calculate, if the point lies above or below the line.
        So, we evaluate the line equation at the given point and
        and return the sign

        @param line
        @param p The point for which we have to evaluate the line equation
        @return bool false if the value is negative and true otherwise
     */

    double val = p.second - line.first * p.first - line.second;
    if (val < 0)
        return false;
    return true;
}

bool check_signs(Point p1, Point p2, Point q1, Point q2)
{
    /**
        This function checks if the two line segments intersect or not.

        @param p1 One end of the first line segment
        @param p2 Other end of the first line segment
        @param q1 One end of the second line segment
        @param q2 Other end of the second line segment
        @return true if the line segments intersect, false otherwise
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
    /**
        To check if a single edge intersects with one or more edges from a
        list of edges.

        @param edges The list of edges
        @param e1 The edge for which we have to check if it intersects the
                  edges from the list
        @return true, if there exists atleast one edge that intersects the
                specified edge, false otherwise
     */

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
    /**
        This function is used when we are finding the convex hull,
        As we need to find the bottom most point so we use this comparator.
        The function assumes the point with bigger y value is bigger.
        If the y coordinate of the two points is same, then we consider the
        x coordinate.

        @param a The first point in the comparison
        @param b the second point in the comparison
        @return false if a > b else true
     */

    if (a.second > b.second)
        return false;
    if (a.second == b.second && a.first > b.first)
        return false;
    return true;
}

double cal_slope(const Point &a, const Point &b)
{
    /**
        Calculates the slope of the line passing between the two points
        with refernce to X-axis

        @param a The first point for slope calculation
        @param b The second point for slope calculation
        @return The slope of the line
     */

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
    /**
        Calculates the clockwise angle between the vector BA and
        the unit vector i.

        The function is used, as we have to sort the points on the basis
        of this angle when we calculate the convex hull.

        @param a The first point
        @param b The second point
        @return The angle in radians between the two points and x-axis
     */

    double x1 = a.first, y1 = a.second;
    double x2 = b.first, y2 = b.second;
    double angle = atan2(y1 - y2, x1 - x2);
    return angle >= 0 ? angle : angle + 2 * M_PIl;
}

double cal_length(Edge edge)
{
    /**
        Calculates the length of the line

        @param edge A line segment comprising of two points
        @return The length of the line segment
     */

    Point p1 = edge.first;
    Point p2 = edge.second;
    double x1_x2 = p1.first - p2.first;
    double y1_y2 = p1.second - p2.second;
    return sqrt(x1_x2 * x1_x2 + y1_y2 * y1_y2);
}

double cal_decision_length(Point &p, Edge &e)
{
    /**
        When we are enhancing the concave hull,
        we break an edge to form two edges, by using a new point,
        The decision that this point will be used
        depends on the decision length

        @param p The candidate point
        @param e The candidate edge which can be broken
        @return The decision length on the basis of which we will
                break the edge further
     */

    return std::min(
               cal_length(std::make_pair(p, e.first)),
               cal_length(std::make_pair(p, e.second))
           );
}

bool in_range(Point &p1, Point &p2, Point &p0)
{
    /**
        To check if the point lies between the perpendiculars,
        drawn at the ends of the line segment p1->p2

        .             .
        .             .
        |             |
        |             |
        |     p0      |
        |             |
        p1-----------p2
        |             |
        |             |
        |             |    p3
        |             |
        .             .
        .             .

        Like in this case p0 lies between the perpendiculars
        but p3 is not within that region.
     */

    double dx = p2.first - p1.first;
    double dy = p2.second - p1.second;

    double innerProduct = (p0.first - p1.first) * dx +
                          (p0.second - p1.second) * dy;
    return 0 <= innerProduct && innerProduct <= dx * dx + dy * dy;
}

double line_point_distance(Point &p, Edge e)
{
    /**
        Given a line and a point, the function will give
        perpendicular distance between the line and the point.

        If the point doesn't lie `in_range`, in that case, the function
        will return `INF`.
     */

    Point p1 = e.first;
    Point p2 = e.second;
    double x0 = p.first, y0 = p.second;
    double x1 = p1.first, y1 = p1.second;
    double x2 = p2.first, y2 = p2.second;
    if (in_range(p1, p2, p))
        return  abs((y2 - y1) * x0 - (x2 - x1) * y0 + x2 * y1 - y2 * x1) /
                sqrt((y2 - y1) * (y2 - y1) + (x2 - x1) * (x2 - x1));
    return INF;
}

Point closest_point(Edge &e, PointVector& ip,
                    std::map<Point, std::pair<Point, Point>> &neighbours)
{
    /**
        Given an edge, and a vector of points. The function returns
        the closest point from this edge.
        If the calculated closest point is closer to the neighbour edges of the
        given edge, in that case the function returns INF.

        @param e Edge from which we have to find the closest point
        @param ip PointVector containing all the candidate points
        @param neighbours Contains the neighbour edges of a given edge,
                    for checking if the closest point is not closer to the
                    neighbour edges.
        @return The closest point from the given edge
     */

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
                                   e.first,
                                   neighbours[e.first].first == e.second ?
                                   neighbours[e.first].second :
                                   neighbours[e.first].first
                               )
                           );
    double neighbourDis2 = line_point_distance(
                               closest,
                               std::make_pair(
                                   e.second,
                                   neighbours[e.second].first == e.first ?
                                   neighbours[e.second].second :
                                   neighbours[e.second].first
                               )
                           );
    if (closestDis < neighbourDis1 && closestDis < neighbourDis2)
        return closest;
    return std::make_pair(INF, INF);
}

double cal_angle(const Point& p1, const Point& p2, const Point& p3)
{
    /*
        Calculates to anti-clockwise angle between [P2-P1] and [P2-P3].

        @param p1 The first point for angle calculation
        @param p2 The second point for angle calculation
        @param p3 The third point for angle calculation
        @return Angle in radians between the three points.
    */

    double x1 = p1.first - p2.first, y1 = p1.second - p2.second;
    double x2 = p3.first - p2.first, y2 = p3.second - p2.second;
    double dot = x1 * x2 + y1 * y2;
    // dot product between [x1, y1] and [x2, y2]
    double det = x1 * y2 - y1 * x2;
    // determinant
    double angle = atan2(det, dot);
    return angle >= 0 ? angle : angle + 2 * M_PIl;
}

double cal_dis(const Point& p1, const Point& p2)
{
    /**
        Calculates the distance between two points

        @param p1 The first point
        @param p2 The second point
        @result Distance between the two points
     */

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
    /**
        This is used for sorting a list of points,
        based on the angle of the line segment between this point
        and a reference point, w.r.t. x-axis.
        @param a First point to compare
        @param b Second point to compare
        @result true if angle between a-ref-xaxis is
                less than angle between b-ref-xaxis else false
     */
    double angle_ar = cal_angle_with_X_axis(a, refPoint);
    double angle_br = cal_angle_with_X_axis(b, refPoint);
    return angle_ar < angle_br;
}
