#ifndef TOPOLOGYPATHPLANNING_OBSTACLE_HPP
#define TOPOLOGYPATHPLANNING_OBSTACLE_HPP

#include <vector>
#include <libxml/tree.h>
#include "topologyPathPlanning/homotopy/WorldDatatype.hpp"
#include "topologyPathPlanning/homotopy/LineSubsegment.hpp"

namespace topologyPathPlanning{

namespace homotopy {

  class WorldMap;

  class Obstacle {

  public:
    Obstacle(std::vector<Point2D> points, int index, WorldMap* world);
    virtual ~Obstacle();

    Point2D sample_position();
    double distance_to_bk( Point2D& point );
    bool contains( Point2D point );

    virtual void to_xml( const std::string& filename )const;
    virtual void to_xml( xmlDocPtr doc, xmlNodePtr root )const;

    virtual void from_xml( const std::string& filename );
    virtual void from_xml( xmlNodePtr root );

    Point2D get_centroid() { return _centroid; }

    WorldMap* get_world() { return _p_world; }
    int get_index () { return _index; }
    std::string get_name();
    
    std::vector<Point2D> m_points;
    Polygon2D m_pgn;
    Point2D m_bk;
    std::vector<Segment2D> m_border_segments;

    LineSubSegmentSet* mp_alpha_seg;
    LineSubSegmentSet* mp_beta_seg;

    std::vector<IntersectionPoint> m_alpha_intersection_points;
    std::vector<IntersectionPoint> m_beta_intersection_points;

    double m_dist_bk2cp;

  protected:
    int _index;
    int _min_x;
    int _min_y;
    int _max_x;
    int _max_y;
    Point2D _centroid;
    WorldMap* _p_world;

  };

  std::ostream& operator<<( std::ostream& out, const Obstacle& other );

} // homotopy

} // topologyPathPlanning

#endif // TOPOLOGYPATHPLANNING_OBSTACLE_HPP
