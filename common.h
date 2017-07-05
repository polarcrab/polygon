/**
    Header file containing all the user defined types

    @author Harmandeep Singh
    @email harmandeep DOT singh1 AT delhivery DOT com
    @date 3 July 2017
*/

#ifndef COMMON_H
#define COMMON_H

#include <vector>

typedef std::pair<double, double> Point;
typedef std::pair<double, double> DirVector;
typedef std::pair<double, double> Line;
typedef std::vector<Point> PointVector;
typedef std::pair<Point, Point> Edge; 

#endif