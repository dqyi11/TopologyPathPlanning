#ifndef WORLD_DATATYPE_H
#define WORLD_DATATYPE_H

#include <CGAL/Simple_cartesian.h>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Polygon_traits_2.h>
#include <CGAL/Polygon_2.h>
#include <CGAL/Polygon_with_holes_2.h>

typedef CGAL::Simple_cartesian<double> K;
typedef K::Point_2                     Point2D;
typedef K::Segment_2                   Segment2D;
typedef K::Ray_2                       Ray2D;
typedef K::Line_2                      Line2D;
typedef CGAL::Polygon_2<K>             Polygon2D;
typedef CGAL::Polygon_with_holes_2<K>  PolygonWithHoles2D;
typedef K::Direction_2                 Direction2D;

#endif // WORLD_DATATYPE_H
