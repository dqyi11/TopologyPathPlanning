#ifndef TOPOLOGYPATHPLANNING_REGION_HPP
#define TOPOLOGYPATHPLANNING_REGION_HPP

#include <vector>
#include "topologyPathPlanning/homotopy/WorldDatatype.hpp"
#include "topologyPathPlanning/homotopy/LineSubsegment.hpp"

namespace topologyPathPlanning {

namespace homotopy {

  class SubRegionSet;

  class SubRegion {
  public:
    SubRegion( Polygon2D poly , SubRegionSet* p_parent );
    virtual ~SubRegion();
    
    bool contains( Point2D point );
    Point2D samplePosition();

 
    std::string getName();

    std::vector<Point2D>         mPoints;
    Polygon2D                    mPolygon;
    Point2D                      mCentroid;
    double                       mDistToCp;
    unsigned int                 mIndex;
    std::vector<LineSubSegment*> mNeighbors;
    bool                         mIsConnectedToCentralPoint;
  protected:
    SubRegionSet*                mpParent;
    int                          mMinX;
    int                          mMinY;
    int                          mMaxX;
    int                          mMaxY;
  };

  class SubRegionSet {
  public:
    SubRegionSet(std::list<Point2D> points, unsigned int idx);
    virtual ~SubRegionSet();

    std::string getName();

    Polygon2D    mPolygon;
    unsigned int mIndex;
    std::vector<Point2D>    mBoundaryPoints;
    std::vector<SubRegion*> mSubregions;
    Point2D                 mCentroid;

    LineSubSegmentSet*      mpLineSegmentsA;
    LineSubSegmentSet*      mpLineSegmentsB;
  };

} // homotopy

} // topologyPathPlanning

#endif // TOPOLOGYPATHPLANNING_REGION_HPP
