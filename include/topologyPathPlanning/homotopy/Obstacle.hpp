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

    Point2D samplePosition();
    double distanceToBk( Point2D& point );
    bool contains( Point2D point );

    virtual void toXml( const std::string& filename )const;
    virtual void toXml( xmlDocPtr doc, xmlNodePtr root )const;

    virtual void fromXml( const std::string& filename );
    virtual void fromXml( xmlNodePtr root );

    Point2D getCentroid() { return mCentroid; }

    WorldMap* getWorld() { return mpWorld; }
    int getIndex () { return mIndex; }
    std::string getName();
    
    std::vector<Point2D> mPoints;
    Polygon2D mPgn;
    Point2D mBk;
    std::vector<Segment2D> mBorderSegments;

    LineSubSegmentSet* mpAlphaSeg;
    LineSubSegmentSet* mpBetaSeg;

    std::vector<IntersectionPoint> mAlphaIntersectionPoints;
    std::vector<IntersectionPoint> mBetaIntersectionPoints;

    double mDistBk2cp;

  protected:
    int mIndex;
    int mMinX;
    int mMinY;
    int mMaxX;
    int mMaxY;
    Point2D mCentroid;
    WorldMap* mpWorld;

  };

  std::ostream& operator<<( std::ostream& out, const Obstacle& other );

} // homotopy

} // topologyPathPlanning

#endif // TOPOLOGYPATHPLANNING_OBSTACLE_HPP
