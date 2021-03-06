/*
 *
 * -----+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----
 *            . _..::__:  ,-"-"._        |7       ,     _,.__
 *    _.___ _ _<_>`!(._`.`-.    /         _._     `_ ,_/  '  '-._.---.-.__
 * >.{     " " `-==,',._\{  \  / {)      / _ ">_,-' `                mt-2_
 *   \_.:--.       `._ )`^-. "'       , [_/(                       __,/-'
 *  '"'     \         "    _L        oD_,--'                )     /. (|
 *           |           ,'          _)_.\\._<> 6              _,' /  '
 *           `.         /           [_/_'` `"(                <'}  )
 *            \\    .-. )           /   `-'"..' `:.#          _)  '
 *     `        \  (  `(           /         `:\  > \  ,-^.  /' '
 *               `._,   ""         |           \`'   \|   ?_)  {\
 *                  `=.---.        `._._       ,'     "`  |' ,- '.
 *                    |    `-._         |     /          `:`<_|h--._
 *                    (        >        .     | ,          `=.__.`-'\
 *                     `.     /         |     |{|              ,-.,\     .
 *                      |   ,'           \   / `'            ,"     \
 *                      |  /              |_'                |  __  /
 *                      | |                                  '-'  `-'   \.
 *                      |/                                         "    /
 *                      \.                                             '

 *                       ,/            ______._.--._ _..---.---------._
 *      ,-----"-..?----_/ )      __,-'"             "                  (
 * -.._(                  `-----'                                       `-
 * -----+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----
 * Map (C) 1998 Matthew Thomas. Freely usable if this line is included. <-
 */

/**
    This file contains the API functions, exposed as the library.

    Each point(x, y) in the following functions will be multiplied
    by a constant because difference between lat/longs is very less.
    So, when we need the square of the difference, it underflows.

    @author Harmandeep Singh
    @email harmandeep DOT singh1 AT delhivery DOT com
    @date 3 July 2017
*/

#include <iostream>
#include <vector>
#include <set>

#include <boost/python.hpp>
#include <boost/python/module.hpp>
#include <boost/python/def.hpp>

#include "common.h"
#include "constants.h"
#include "hull.h"
#include "utils.h"

namespace bp = boost::python;

bp::list concave_hull(bp::list ip, bp::object tweak)
{
    /**
    @param ip Python list of points(x, y) for whcih we have to
              find the concave hull
    @param tweak float value, the conncaveness parameter
    @return result Python list of tuples of tuple. Each outer tuple
                   contains two tuples and each inner tuple is of
                   length 2 having the x and y value.
    */
    bp::ssize_t len = bp::len(ip);
    float cTweak = bp::extract<float>(tweak);
    PointVector inputPv;
    bp::list result;
    for (int i = 0; i < len; i++)
    {
        bp::tuple val = bp::extract<bp::tuple>(ip[i]);
        double x = bp::extract<double>(val[0]);
        double y = bp::extract<double>(val[1]);
        inputPv.push_back(std::make_pair(x * MUL_FACTOR, y * MUL_FACTOR));
    }
    std::set<std::pair<double, double>> cleanSet(
                                         inputPv.begin(),
                                         inputPv.end()
                                     );
    inputPv.clear();
    std::copy(
        cleanSet.begin(), cleanSet.end(),
        std::back_inserter(inputPv)
    );
    std::vector<Edge> opPv = get_concave_hull(inputPv, cTweak);
    for (int i = 0; i < int(opPv.size()); i++)
        result.append(
            bp::make_tuple(
                bp::make_tuple(
                    opPv[i].first.first / MUL_FACTOR,
                    opPv[i].first.second / MUL_FACTOR
                ),
                bp::make_tuple(
                    opPv[i].second.first / MUL_FACTOR,
                    opPv[i].second.second / MUL_FACTOR
                )
            )
        );
    return result;
}

bp::list convex_hull(bp::list ip)
{
    /**
    @param ip Python list of points(x, y) for whcih we have to
              find the convex hull
    @return result Python list of tuples of tuple. Each outer tuple
                   contains two tuples and each inner tuple is of
                   length 2 having the x and y value.
    */
    bp::ssize_t len = bp::len(ip);
    PointVector inputPv;
    bp::list result;
    for (int i = 0; i < len; i++)
    {
        bp::tuple val = bp::extract<bp::tuple>(ip[i]);
        double x = bp::extract<double>(val[0]);
        double y = bp::extract<double>(val[1]);
        inputPv.push_back(std::make_pair(x * MUL_FACTOR, y * MUL_FACTOR));
    }
    std::set<std::pair<double, double>> cleanSet(
                                         inputPv.begin(),
                                         inputPv.end()
                                     );
    inputPv.clear();
    std::copy(cleanSet.begin(), cleanSet.end(), std::back_inserter(inputPv));
    PointVector opPv = get_convex_hull(inputPv);
    for (int i = 0; i < int(opPv.size()); i++)
    {
        int idx = (i + 1) % int(opPv.size());
        result.append(
            bp::make_tuple(
                bp::make_tuple(
                    opPv[i].first / MUL_FACTOR,
                    opPv[i].second / MUL_FACTOR
                ),
                bp::make_tuple(
                    opPv[idx].first / MUL_FACTOR,
                    opPv[idx].second / MUL_FACTOR
                )
            )
        );
    }
    return result;
}

BOOST_PYTHON_MODULE(libcavehull)
{
    bp::def("concave_hull", concave_hull);
    bp::def("convex_hull", convex_hull);
}