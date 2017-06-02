#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <map>
#include <list>
#include <set>
#include <vector>

#include "include/common.h"
#include "include/constants.h"
#include "include/utils.h"

void printStuff(const Point& p1, const Point& p2, const Point& p3)
{
    std::cout << p1.first << " " << p1.second << std::endl;
    std::cout << p2.first << " " << p2.second << std::endl;
    std::cout << p3.first << " " << p3.second << std::endl;
    std::cout << calAngle(p1, p2, p3) << std::endl;
    std::cout << std::endl;
}


PointVector getConvexHull(PointVector& mainip)
{
    PointVector ip = PointVector(mainip);
    PointVector::iterator lowerMostItr = std::min_element(
            ip.begin(), ip.end(), pointComparator);
    Point lowerMostPoint = *lowerMostItr;
    ip.erase(lowerMostItr);
    std::cout << M_PIl << std::endl;
    sort(ip.begin(), ip.end(), AngleSort(lowerMostPoint));
    PointVector hullStack;
    hullStack.push_back(lowerMostPoint);
    hullStack.push_back(ip[0]);
    hullStack.push_back(ip[1]);
    for (int i = 2; i < int(ip.size()); i++)
    {
        while (calAngle(ip[i], hullStack.end()[-1], hullStack.end()[-2]) >= M_PIl)
            hullStack.pop_back();
        hullStack.push_back(ip[i]);
    }
    return hullStack;
}

std::vector<Edge> getConcaveHull(PointVector& ip, double N = INF)
{
    PointVector ipCopy = PointVector(ip);
    PointVector convexHull = getConvexHull(ipCopy);
    std::vector<Edge> answer;
    std::map<Point, std::pair<Point, Point>> neighbours;
    std::list<Edge> edges;
    for (int i = 0; i < int(convexHull.size()); i++)
    {
        int leftIndex = (int(i) - 1) % convexHull.size();
        int rightIndex = (i + 1) % convexHull.size();
        neighbours[convexHull[i]] = std::make_pair(convexHull[leftIndex],
                                    convexHull[rightIndex]);
        edges.push_back(std::make_pair(convexHull[i], convexHull[rightIndex]));
    }
    for (std::list<Edge>::iterator itr = edges.begin(); itr != edges.end();)
    {
        Edge currentEdge = *itr;
        Point closest = closestPoint(currentEdge, ip, neighbours);
        if (closest.first == INF)
        {
            itr++;
            continue;
        }
        double lengthOfEdge = calLength(currentEdge);
        double decisionDistance = calDecisionLength(closest, currentEdge);
        if (lengthOfEdge / decisionDistance > N)
        {
            edges.push_back(std::make_pair(currentEdge.first, closest));
            edges.push_back(std::make_pair(currentEdge.second, closest));
            if (checkIntersections(edges, std::make_pair(currentEdge.first, closest), neighbours) || checkIntersections(edges, std::make_pair(currentEdge.second, closest), neighbours))
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
    for (std::list<Edge>::iterator itr = edges.begin(); itr != edges.end(); itr++)
    {
        answer.push_back(*itr);
    }
    return answer;
}
