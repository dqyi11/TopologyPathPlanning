#ifndef TOPOLOGYPATHPLANNING_WORLDMAP_HPP
#define TOPOLOGYPATHPLANNING_WORLDMAP_HPP

#include <libxml/tree.h>
#include "topologyPathPlanning/homotopy/Obstacle.hpp"
#include "topologyPathPlanning/homotopy/Region.hpp"

namespace topologyPathPlanning {

namespace homotopy {

  class PointSequence {
  public:
    PointSequence();
    virtual ~PointSequence();
    void addPoint(double x, double y);

    std::vector<Point2D> mPoints;
  };

  class WorldMap {
  public:
    WorldMap();
    WorldMap( int width, int height );
    virtual ~WorldMap();

    bool resize( int width, int height );
    bool loadObstacleInfo( std::vector< std::vector<Point2D> > polygons);
    bool init( bool rand_init_points = true );

    virtual void toXml( const std::string& filename )const;
    virtual void toXml( xmlDocPtr doc, xmlNodePtr root )const;

    virtual void fromXml( const std::string& filename );
    virtual void fromXml( xmlNodePtr root );

    int getWidth() const { return mMapWidth; }
    int getHeight() const { return mMapHeight; }

    bool isInObstacle( Point2D point );
    bool isInObsBkLines( Point2D point );
    
    SubRegion* findSubregion( std::string name );
    LineSubSegment* findLinesubsegment( std::string name );

    SubRegion* findSubregion( Point2D point );
    LineSubSegment* findLinesubsegment( Point2D point );
    Obstacle* findObstacle( Point2D point );
 
    Point2D getCentralPoint() const { return mCentralPoint; }
    std::vector<Obstacle*> getObstacles() const { return mObstacles; }
    std::vector<SubRegionSet*> getSubregionSet() const { return mRegionSets; }
    std::vector<SubRegion*> getSubregions() const { return mSubregions; }
    std::vector<LineSubSegmentSet*> getLinesubsegmentSet() const { return mLineSegments; }

    double getDistanceToCentralPoint( Point2D point );

    SubRegion * inSubregion( Point2D point );
  protected:
    bool initPoints();
    bool initRays();
    bool initSegments();
    bool initRegions();

    std::vector< std::pair<Point2D, Obstacle*> > intersect( Segment2D seg, Obstacle* p_obstacle );
    Point2D* findIntersectionWithBoundary( Ray2D* p_ray );

    std::list<Point2D>       intersectWithBoundaries( LineSubSegmentSet* p_segment1, LineSubSegmentSet* p_segment2 );
    std::vector<SubRegion*>  getSubregions( SubRegionSet* p_region );
    bool                     isIntersected( Polygon2D poly, Segment2D seg, double delta );

  private:
    int mMapWidth;
    int mMapHeight;

    int mSampleWidthScale;
    int mSampleHeightScale;

    std::vector<Obstacle*>          mObstacles;
    std::vector<LineSubSegmentSet*> mLineSegments;
    std::vector<SubRegionSet*>      mRegionSets;
    std::vector<SubRegion*>         mSubregions;

    std::vector<Line2D>             mObsBkPairLines;
    std::vector<Segment2D>          mBoundaryLines;
    std::vector<Segment2D>          mCenterCornerLines;

    Point2D                mCentralPoint;
    Segment2D              mXMinLine;
    Segment2D              mXMaxLine;
    Segment2D              mYMinLine;
    Segment2D              mYMaxLine;
  };

  std::ostream& operator<<( std::ostream& out, const WorldMap& other );

} // homotopy

} // topologyPathPlanning

#endif // TOPOLOGYPATHPLANNING_WORLDMAP_HPP
