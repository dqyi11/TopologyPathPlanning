#ifndef TOPOLOGYPATHPLANNING_LINE_SUBSEGMENT_HPP
#define TOPOLOGYPATHPLANNING_LINE_SUBSEGMENT_HPP

#include <utility>
#include <vector>
#include <libxml/tree.h>
#include "topologyPathPlanning/homotopy/WorldDatatype.hpp"

namespace topologyPathPlanning {

namespace homotopy {

  class Obstacle;
  class LineSubSegmentSet;
  class SubRegionSet;
  class SubRegion;

  class IntersectionPoint {
  public:
    IntersectionPoint( Point2D point );
    virtual ~IntersectionPoint();

    Point2D   mPoint;
    double    mDistToBk;
    Obstacle* mpObstacle;

    bool operator<(const  IntersectionPoint& other) const {
        return ( mDistToBk < other.mDistToBk );
    }
  };

  std::ostream& operator<<( std::ostream& out, const IntersectionPoint& other );

  class LineSubSegment {

  public:
    LineSubSegment( Point2D pos_a, Point2D pos_b, LineSubSegmentSet* p_subseg_set, unsigned int index, bool is_connected_to_central_point = false );
    virtual ~LineSubSegment();

    virtual void toXml( const std::string& filename )const;
    virtual void toXml( xmlDocPtr doc, xmlNodePtr root )const;

    virtual void fromXml( const std::string& filename );
    virtual void fromXml( xmlNodePtr root );

    bool contains( Point2D point );
    bool isConnected( Obstacle* p_obstacle );
    
    std::string getName();
    Point2D samplePosition();
    Segment2D mSubseg;
    bool mIsConnectedToCentralPoint;
    std::vector< SubRegion* > mNeighbors;

    bool mConnectedToBoundary;
    std::vector< Obstacle* > mConnectedObstacles;
     
  protected:
    LineSubSegmentSet* mpSubsegSet;
    unsigned int mIndex;

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

    virtual void toXml( const std::string& filename )const;
    virtual void toXml( xmlDocPtr doc, xmlNodePtr root )const;

    virtual void fromXml( const std::string& filename );
    virtual void fromXml( xmlNodePtr root );

    static std::string typeToStdString ( const unsigned int& type );
    static unsigned int typeFromStdString ( const std::string& type_str );

    Obstacle* const getObstacle() { return mpObstacle; }

    std::string getName();

    Segment2D    mSeg;
    unsigned int mType;
    std::vector< LineSubSegment* > mSubsegs;
    std::vector< SubRegionSet* >   mNeighbors;
  protected:
    Obstacle*    mpObstacle;
  };

  std::ostream& operator<<( std::ostream& out, const LineSubSegmentSet& other );

} // homotopy

} // topologyPathPlanning

#endif // TOPOLOGYPATHPLANNING_LINE_SUBSEGMENT_HPP
