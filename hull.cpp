#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <map>
#include <list>
#include <set>
#include <vector>

#include "common.h"
#include "constants.h"
#include "utils.h"


PointVector get_convex_hull(PointVector& mainip)
{
    PointVector ip = PointVector(mainip);
    PointVector::iterator lowerMostItr = std::min_element(
            ip.begin(), ip.end(), point_comparator);
    Point lowerMostPoint = *lowerMostItr;
    ip.erase(lowerMostItr);
    sort(ip.begin(), ip.end(), AngleSort(lowerMostPoint));
    PointVector hullStack;
    hullStack.push_back(lowerMostPoint);
    hullStack.push_back(ip[0]);
    hullStack.push_back(ip[1]);
    for (int i = 2; i < int(ip.size()); i++)
    {
        while (cal_angle(ip[i], hullStack.end()[-1],
                         hullStack.end()[-2]) >= M_PIl)
            hullStack.pop_back();
        hullStack.push_back(ip[i]);
    }
    return hullStack;
}

std::vector<Edge> get_concave_hull(PointVector& ip, double N = INF)
{
    PointVector ipCopy = PointVector(ip);
    PointVector convexHull = get_convex_hull(ipCopy);
    std::map<Point, std::pair<Point, Point>> neighbours;
    std::list<Edge> edges;
    for (size_t i = 0; i < convexHull.size(); i++)
    {
        int leftIndex = (int(i) - 1) % convexHull.size();
        // Using size_t, as size_t can never be negative, so converting it to int
        int rightIndex = (i + 1) % convexHull.size();
        neighbours[convexHull[i]] = std::make_pair(convexHull[leftIndex],
                                    convexHull[rightIndex]);
        edges.push_back(std::make_pair(convexHull[i], convexHull[rightIndex]));
    }
    for (std::list<Edge>::iterator itr = edges.begin(); itr != edges.end();)
    {
        Edge currentEdge = *itr;
        Point closest = closest_point(currentEdge, ip, neighbours);
        if (closest.first == INF)
        {
            itr++;
            continue;
        }
        double lengthOfEdge = cal_length(currentEdge);
        double decisionDistance = cal_decision_length(closest, currentEdge);
        if (lengthOfEdge / decisionDistance > N)
        {
            edges.push_back(std::make_pair(currentEdge.first, closest));
            edges.push_back(std::make_pair(currentEdge.second, closest));
            if (check_intersections(edges,
                                    std::make_pair(
                                        currentEdge.first,
                                        closest)) ||
                    check_intersections(edges,
                                        std::make_pair(
                                            currentEdge.second,
                                            closest)))
            {
                edges.pop_back();
                edges.pop_back();
                itr++;
                continue;
            }
            neighbours[closest] = std::make_pair(currentEdge.first, currentEdge.second);
            neighbours[currentEdge.first] = std::make_pair(
                                                neighbours[currentEdge.first].first == currentEdge.second ? closest : neighbours[currentEdge.first].first,
                                                neighbours[currentEdge.first].second == currentEdge.second ? closest : neighbours[currentEdge.first].second);
            neighbours[currentEdge.second] = std::make_pair(
                                                 neighbours[currentEdge.second].first == currentEdge.first ? closest : neighbours[currentEdge.second].first,
                                                 neighbours[currentEdge.second].second == currentEdge.first ? closest : neighbours[currentEdge.second].second);

            itr = edges.erase(itr);
        }
        else
            itr++;
    }
    std::vector<Edge> answer{
        std::make_move_iterator(std::begin(edges)),
        std::make_move_iterator(std::end(edges))
    };
    return answer;
}
