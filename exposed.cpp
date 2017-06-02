#include <iostream>
#include <vector>
#include <set>

#include <boost/python.hpp>
#include <boost/python/module.hpp>
#include <boost/python/def.hpp>

#include "include/common.h"
#include "include/constants.h"
#include "include/convexhull.h"
#include "include/utils.h"

boost::python::list concave_hull(boost::python::list ip, boost::python::object tweak)
{
    boost::python::ssize_t len = boost::python::len(ip);
    float cTweak = boost::python::extract<float>(tweak);
    PointVector inputPv;
    boost::python::list result;
    for (int i = 0; i < len; i++)
    {
        boost::python::tuple val = boost::python::extract<boost::python::tuple>(ip[i]);
        double x = boost::python::extract<double>(val[0]);
        double y = boost::python::extract<double>(val[1]);
        inputPv.push_back(std::make_pair(x * MUL_FACTOR, y * MUL_FACTOR));
    }
    std::set<std::pair<double, double>> cleanSet(inputPv.begin(), inputPv.end());
    inputPv.clear();
    std::copy(cleanSet.begin(), cleanSet.end(), std::back_inserter(inputPv));
    std::vector<Edge> opPv = getConcaveHull(inputPv, cTweak);
    for (int i = 0; i < int(opPv.size()); i++)
        result.append(boost::python::make_tuple(boost::python::make_tuple(opPv[i].first.first / MUL_FACTOR, opPv[i].first.second / MUL_FACTOR),
                                                boost::python::make_tuple(opPv[i].second.first / MUL_FACTOR, opPv[i].second.second / MUL_FACTOR)));
    return result;
}

boost::python::list convex_hull(boost::python::list ip)
{
    boost::python::ssize_t len = boost::python::len(ip);
    PointVector inputPv;
    boost::python::list result;
    for (int i = 0; i < len; i++)
    {
        boost::python::tuple val = boost::python::extract<boost::python::tuple>(ip[i]);
        double x = boost::python::extract<double>(val[0]);
        double y = boost::python::extract<double>(val[1]);
        inputPv.push_back(std::make_pair(x * MUL_FACTOR, y * MUL_FACTOR));
    }
    std::set<std::pair<double, double>> cleanSet(inputPv.begin(), inputPv.end());
    inputPv.clear();
    std::copy(cleanSet.begin(), cleanSet.end(), std::back_inserter(inputPv));
    PointVector opPv = getConvexHull(inputPv);
    for (int i = 0; i < int(opPv.size()); i++)
    {
        int idx = (i + 1) % int(opPv.size());
        result.append(boost::python::make_tuple(
                          boost::python::make_tuple(opPv[i].first / MUL_FACTOR, opPv[i].second / MUL_FACTOR),
                          boost::python::make_tuple(opPv[idx].first / MUL_FACTOR, opPv[idx].second / MUL_FACTOR)
                      ));
    }
    return result;
}

// main()
// {
//     // Point p1 = std::make_pair(1, 1);
//     // Point p2 = std::make_pair(0, 0);
//     // Point p3 = std::make_pair(2, 2);
//     // std::cout << calAngleWithXAxis(p2, p3) << std::endl;
//     std::cout << inRange(0, 0, 1, 0, 0.5, 1) << std::endl;
//     std::cout << inRange(0, 0, 2, 0, -1, 1) << std::endl;
//     std::cout << inRange(0, 0, -1, 0, 0.5, 1) << std::endl;
// }

BOOST_PYTHON_MODULE(libcavehull)
{
    boost::python::def("concave_hull", concave_hull);
    boost::python::def("convex_hull", convex_hull);
}