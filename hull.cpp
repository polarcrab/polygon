/**
    This file contains the core functionality.

    The file contains 2 functions, one for creating the
    convex hull and the other for concave hull

    @author Harmandeep Singh
    @email harmandeep DOT singh1 AT delhivery DOT com
    @date 3 July 2017
*/

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
    /**
        For finding the convex hull of given vector of points

        @param mainip Vector of Points
        @return vector of Points which will form the convex hull
     */

    PointVector ip = PointVector(mainip);

    //Finding the lowermost point
    PointVector::iterator lowerMostItr = std::min_element(
            ip.begin(), ip.end(), point_comparator);
    Point lowerMostPoint = *lowerMostItr;
    ip.erase(lowerMostItr);

    //Sorting the rest of the points on the basis of the angle.
    sort(ip.begin(), ip.end(), AngleSort(lowerMostPoint));

    //Applying Graham Scan method of finding the convex hull
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
    /**
        Calculates the concave hull of a given vector of points.

        @param ip Vector of points
        @return vector of edges, where each edge is
                the part of the concave hull formed
     */

    PointVector ipCopy = PointVector(ip);

    // Calculating the convex hull as part of concave hull
    PointVector convexHull = get_convex_hull(ipCopy);

    /* For storing the adjacent point from each point
       or we can say for storing the neighbour point of each point */
    std::map<Point, std::pair<Point, Point>> adj;
    std::list<Edge> edges;

    for (size_t i = 0; i < convexHull.size(); i++)
    {
        // Creating edges and storing neighbour of each point
        int leftIndex = (int(i) - 1) % convexHull.size();
        // Using size_t, as size_t cannot be negative, so converting it to int
        int rightIndex = (i + 1) % convexHull.size();
        adj[convexHull[i]] = std::make_pair(convexHull[leftIndex],
                                            convexHull[rightIndex]);
        edges.push_back(std::make_pair(convexHull[i], convexHull[rightIndex]));
    }
    std::list<Edge>::iterator itr = edges.begin();
    while (itr != edges.end())
    {
        /* Iterating over each edge and checking
           if it can be broken into two edges */
        Edge cur = *itr;

        // Finding the closest valid point from the current edge
        Point closest = closest_point(cur, ip, adj);
        if (closest.first == INF)
        {
            /* If there is not valid point close to the cuurent edge,
            then  move onto the next edge */
            itr++;
            continue;
        }

        /* If the edge is valid, check
        if this can be broken into two edges or not */
        double lengthOfEdge = cal_length(cur);
        double decisionDistance = cal_decision_length(closest, cur);
        if (lengthOfEdge / decisionDistance > N)
        {
            /* If the edge can be partitioned,
              add the two new edges to the list of edges */
            edges.push_back(std::make_pair(cur.first, closest));
            edges.push_back(std::make_pair(cur.second, closest));

            // Check if the new edges intersect any old edge
            if (check_intersections(edges,
                                    std::make_pair(
                                        cur.first,
                                        closest)) ||
                    check_intersections(edges,
                                        std::make_pair(
                                            cur.second,
                                            closest)))
            {
                // If the edges intersect any old edge, remove the new edges
                edges.pop_back();
                edges.pop_back();
                itr++;
                continue;
            }

            /* Updating the neighbour points of the new point that
               has been added and the points which were the opposite
               ends of the edge which has been partitioned */
            adj[closest] = std::make_pair(cur.first, cur.second);
            adj[cur.first] = std::make_pair(
                                 adj[cur.first].first == cur.second ?
                                 closest : adj[cur.first].first,
                                 adj[cur.first].second == cur.second ?
                                 closest : adj[cur.first].second);
            adj[cur.second] = std::make_pair(
                                  adj[cur.second].first == cur.first ?
                                  closest : adj[cur.second].first,
                                  adj[cur.second].second == cur.first ?
                                  closest : adj[cur.second].second);
            itr = edges.erase(itr);
        }
        else
            itr++;
    }

    // Returning the edges as a vector
    std::vector<Edge> answer{
        std::make_move_iterator(std::begin(edges)),
        std::make_move_iterator(std::end(edges))
    };
    return answer;
}
