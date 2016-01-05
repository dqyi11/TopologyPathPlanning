#ifndef LINE_SUBSEGMENT_H
#define LINE_SUBSEGMENT_H

#include <utility>
#include <vector>
#include <libxml/tree.h>
#include "world_datatype.h"

namespace homotopy {

  class Obstacle;
  class LineSubSegmentSet;
  class SubRegionSet;
  class SubRegion;

  class IntersectionPoint {
  public:
    IntersectionPoint( Point2D point );
    virtual ~IntersectionPoint();

    Point2D   m_point;
    double    m_dist_to_bk;
    Obstacle* mp_obstacle;

    bool operator<(const  IntersectionPoint& other) const {
        return ( m_dist_to_bk < other.m_dist_to_bk );
    }
  };

  std::ostream& operator<<( std::ostream& out, const IntersectionPoint& other );

  class LineSubSegment {

  public:
    LineSubSegment( Point2D pos_a, Point2D pos_b, LineSubSegmentSet* p_subseg_set, unsigned int index, bool is_connected_to_central_point = false );
    virtual ~LineSubSegment();

    virtual void to_xml( const std::string& filename )const;
    virtual void to_xml( xmlDocPtr doc, xmlNodePtr root )const;

    virtual void from_xml( const std::string& filename );
    virtual void from_xml( xmlNodePtr root );

    bool contains( Point2D point );

    std::string get_name();
    Point2D sample_position();
    Segment2D m_subseg;
    bool m_is_connected_to_central_point;
    std::vector< SubRegion* > m_neighbors;

    bool m_connected_to_boundary;
    std::vector< Obstacle* > m_connected_obstacles;
     
  protected:
    LineSubSegmentSet* _p_subseg_set;
    unsigned int _index;

  };

  std::ostream& operator<<( std::ostream& out, const LineSubSegment& other );

  typedef enum {
    LINE_TYPE_UNKNOWN,
    LINE_TYPE_ALPHA,
    LINE_TYPE_BETA,
    NUM_LINE_TYPE
  } line_subsegment_set_type_t;

  class LineSubSegmentSet {

  public:
    LineSubSegmentSet( Point2D pos_a, Point2D pos_b, unsigned int type, Obstacle* p_obstacle );
    virtual ~LineSubSegmentSet();

    bool load( std::vector<IntersectionPoint>& intersections );

    /*
    bool operator<(const  LineSubSegmentSet& other) const {
        return ( m_direction < other.m_direction );
    }*/

    virtual void to_xml( const std::string& filename )const;
    virtual void to_xml( xmlDocPtr doc, xmlNodePtr root )const;

    virtual void from_xml( const std::string& filename );
    virtual void from_xml( xmlNodePtr root );

    static std::string type_to_std_string ( const unsigned int& type );
    static unsigned int type_from_std_string ( const std::string& type_str );

    Obstacle* const get_obstacle() { return _p_obstacle; }

    std::string get_name();

    Segment2D    m_seg;
    unsigned int m_type;
    std::vector< LineSubSegment* > m_subsegs;
    std::vector< SubRegionSet* >   m_neighbors;
  protected:
    Obstacle*    _p_obstacle;
  };

  std::ostream& operator<<( std::ostream& out, const LineSubSegmentSet& other );

}

#endif // LINE_SUBSEGMENT_H
