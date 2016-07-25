#ifndef WORLD_DATATYPE_H
#define WORLD_DATATYPE_H

#include <CGAL/Cartesian.h>
#include <CGAL/Exact_predicates_exact_constructions_kernel.h>
#include <CGAL/Polygon_traits_2.h>
#include <CGAL/Polygon_2.h>
#include <CGAL/Polygon_set_2.h>
#include <CGAL/Polygon_with_holes_2.h>

//typedef CGAL::Cartesian<double> Kernel;
typedef CGAL::Exact_predicates_exact_constructions_kernel    Kernel;
typedef Kernel::Point_2                                      Point2D;
typedef Kernel::Segment_2                                    Segment2D;
typedef Kernel::Ray_2                                        Ray2D;
typedef Kernel::Line_2                                       Line2D;
typedef CGAL::Polygon_2<Kernel>                              Polygon2D;
typedef CGAL::Polygon_with_holes_2<Kernel>                   PolygonWithHoles2D;
typedef Kernel::Direction_2                                  Direction2D;
typedef Polygon2D::Vertex_circulator                         Vertex_circulator;
typedef CGAL::Polygon_set_2< Kernel, std::vector<Point2D> >  Polygon2D_set;

#endif // WORLD_DATATYPE_H
