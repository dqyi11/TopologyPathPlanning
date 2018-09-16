#include <sstream>
#include "opencv2/core/core.hpp"
#include <CGAL/intersections.h>
#include <CGAL/Polygon_2_algorithms.h>
#include <CGAL/Boolean_set_operations_2.h>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include "topologyPathPlanning/homotopy/Worldmap.hpp"

#define DELTA_TRIAL 2.0

namespace topologyPathPlanning {

namespace homotopy {

bool Segment2DSort(const Segment2D& lhs, const Segment2D& rhs) {
  return lhs.direction() < rhs.direction();
}

bool LineSubSegmentSetSort(const LineSubSegmentSet* lhs, const LineSubSegmentSet* rhs) {
  return lhs->mSeg.direction() < rhs->mSeg.direction();
}

bool SubregionSort(const SubRegion* lhs, const SubRegion* rhs ) {
  return lhs->mDistToCp < rhs->mDistToCp;
}

PointSequence::PointSequence()
{
}

PointSequence::~PointSequence()
{
  mPoints.clear();
}

void PointSequence::addPoint(double x, double y)
{
    Point2D point(x, y);
    mPoints.push_back(point);
}

WorldMap::WorldMap() {
  mMapWidth = 0;
  mMapHeight = 0;

  mSampleWidthScale = 0;
  mSampleHeightScale = 0;

  mObstacles.clear();
  mBoundaryLines.clear();
  mObsBkPairLines.clear();
  mLineSegments.clear();
  mCenterCornerLines.clear();

  mCentralPoint = Point2D(0, 0);
}

WorldMap::WorldMap( int width, int height ) {
  WorldMap();
  resize(width, height);
}

WorldMap::~WorldMap() {

  for( unsigned int i=0; i < mObstacles.size(); i++ ) {
    Obstacle* po = mObstacles[i];
    delete po;
    po = NULL;
  }
  mObstacles.clear();
  mBoundaryLines.clear();
  mObsBkPairLines.clear();
  mLineSegments.clear();
  mCenterCornerLines.clear();
}

bool WorldMap::resize( int width, int height ) {
  if ( width < 0 || height < 0 ) {
    return false;
  }
  mMapWidth = width;
  mMapHeight = height;
  mSampleWidthScale = mMapWidth/5;
  mSampleHeightScale = mMapWidth/5;
  mCentralPoint = Point2D(width/2, height/2);

  return false;
}

bool WorldMap::loadObstacleInfo( std::vector< std::vector<Point2D> > polygons ) {
  mObstacles.clear();
  int obs_idx = 0;
  for( std::vector< std::vector<Point2D> >::iterator it=polygons.begin(); it!=polygons.end(); it++ ) {
    std::vector<Point2D> points = (*it);
    Obstacle* p_obs = new Obstacle(points, obs_idx, this);
    obs_idx ++;
    mObstacles.push_back(p_obs);
  }

  return true;
}

bool WorldMap::init( bool rand_init_points ) {
  if ( rand_init_points == true ) {
    initPoints();
  }
  initRays();
  initSegments();
  initRegions();
  return true;
}

bool WorldMap::initPoints() {
  // select random point for each obstacle
  for( std::vector<Obstacle*>::iterator it = mObstacles.begin(); it != mObstacles.end(); it++ ) {
    Obstacle * p_obstacle = (*it);
    p_obstacle->mBk = p_obstacle->samplePosition();
  }

  mObsBkPairLines.clear();
  for( unsigned int i=0; i < mObstacles.size(); i++ ) {
    for( unsigned int j=i+1; j < mObstacles.size(); j++ ) {
      Line2D pline(mObstacles[i]->mBk, mObstacles[j]->mBk);
      mObsBkPairLines.push_back(pline);
    }
  }

  // select central point c
  bool found_cp = false;
  while( found_cp == false ) {
    if ( (false == isInObstacle(mCentralPoint)) && (false == isInObsBkLines(mCentralPoint)) ) {
      found_cp = true;
    }
    else {
      float x_ratio = static_cast<float> (rand())/static_cast<float>(RAND_MAX);
      float y_ratio = static_cast<float> (rand())/static_cast<float>(RAND_MAX);
      int cp_x = static_cast<int>( x_ratio*static_cast<float>(mSampleWidthScale) ) + mMapWidth /2;
      int cp_y = static_cast<int>( y_ratio*static_cast<float>(mSampleHeightScale) ) + mMapHeight /2;
      mCentralPoint = Point2D(cp_x, cp_y);
    }
  }

  for( std::vector<Obstacle*>::iterator it = mObstacles.begin(); it != mObstacles.end(); it++ ) {
    Obstacle * p_obstacle = (*it);
    p_obstacle->mDistBk2cp = p_obstacle->distanceToBk(mCentralPoint);
  }

  return true;
}

bool WorldMap::initRays() {
  // init four boundary line
  mBoundaryLines.clear();
  mXMinLine = Segment2D(Point2D(0,0), Point2D(mMapWidth-1,0));
  mYMinLine = Segment2D(Point2D(0,0), Point2D(0,mMapHeight-1));
  mXMaxLine = Segment2D(Point2D(0,mMapHeight-1), Point2D(mMapWidth-1,mMapHeight-1));
  mYMaxLine = Segment2D(Point2D(mMapWidth-1,0), Point2D(mMapWidth-1,mMapHeight-1));
  mBoundaryLines.push_back(mXMinLine);
  mBoundaryLines.push_back(mYMaxLine);
  mBoundaryLines.push_back(mXMaxLine);
  mBoundaryLines.push_back(mYMinLine);

  // init lines from center point to four corners
  mCenterCornerLines.push_back(Segment2D(mCentralPoint, Point2D(0,0)));
  mCenterCornerLines.push_back(Segment2D(mCentralPoint, Point2D(mMapWidth, 0)));
  mCenterCornerLines.push_back(Segment2D(mCentralPoint, Point2D(mMapWidth, mMapHeight)));
  mCenterCornerLines.push_back(Segment2D(mCentralPoint, Point2D(0, mMapHeight)));
  std::sort(mCenterCornerLines.begin(), mCenterCornerLines.end(), Segment2DSort);

  // init alpha and beta segments
  for( std::vector<Obstacle*>::iterator it=mObstacles.begin(); it!=mObstacles.end(); it++) {
    Obstacle* p_obstacle = (*it);

    Ray2D alpha_ray( mCentralPoint, Point2D(2*mCentralPoint.x()-p_obstacle->mBk.x(), 2*mCentralPoint.y()-p_obstacle->mBk.y()) );
    Ray2D beta_ray( mCentralPoint, p_obstacle->mBk );

    Point2D * p_a_pt = findIntersectionWithBoundary( &alpha_ray );
    Point2D * p_b_pt = findIntersectionWithBoundary( &beta_ray );

    if ( p_a_pt ) {
      p_obstacle->mpAlphaSeg = new LineSubSegmentSet( p_obstacle->mBk, *p_a_pt, LINE_TYPE_ALPHA, p_obstacle );
      mLineSegments.push_back(p_obstacle->mpAlphaSeg);
    }
    if ( p_b_pt ) {
      p_obstacle->mpBetaSeg = new LineSubSegmentSet( p_obstacle->mBk, *p_b_pt, LINE_TYPE_BETA, p_obstacle );
      mLineSegments.push_back(p_obstacle->mpBetaSeg);
    }
  }

  std::sort(mLineSegments.begin(), mLineSegments.end(), LineSubSegmentSetSort);
  return true;
}

bool WorldMap::initSegments() {
  for( std::vector<Obstacle*>::iterator it=mObstacles.begin(); it!=mObstacles.end(); it++) {
    Obstacle* p_obstacle = (*it);
    p_obstacle->mAlphaIntersectionPoints.clear();
    p_obstacle->mBetaIntersectionPoints.clear();

    for( std::vector<Obstacle*>::iterator itr=mObstacles.begin(); itr!=mObstacles.end(); itr++) {
      Obstacle* p_ref_obstacle = (*itr);
      // check alpha_seg with obstacles
      std::vector< std::pair< Point2D, Obstacle* > > a_ints = intersect( p_obstacle->mpAlphaSeg->mSeg, p_ref_obstacle );
      std::vector< std::pair< Point2D, Obstacle* > > b_ints = intersect( p_obstacle->mpBetaSeg->mSeg, p_ref_obstacle );

      for( std::vector< std::pair< Point2D, Obstacle* > >::iterator itp = a_ints.begin(); itp != a_ints.end(); itp++ ) {
        std::pair< Point2D, Obstacle* > p = (*itp);
        IntersectionPoint ip( p.first );
        ip.mDistToBk = p_obstacle->distanceToBk( p.first );
        ip.mpObstacle = p.second;
        p_obstacle->mAlphaIntersectionPoints.push_back(ip);
      }
      for( std::vector< std::pair< Point2D, Obstacle* > >::iterator itp = b_ints.begin(); itp != b_ints.end(); itp++ ) {
        std::pair< Point2D, Obstacle* > p = (*itp);
        IntersectionPoint ip( p.first );
        ip.mDistToBk = p_obstacle->distanceToBk( p.first );
        ip.mpObstacle = p.second;
        p_obstacle->mBetaIntersectionPoints.push_back(ip);
      }
    }

    std::sort( p_obstacle->mAlphaIntersectionPoints.begin(), p_obstacle->mAlphaIntersectionPoints.end() );
    std::sort( p_obstacle->mBetaIntersectionPoints.begin(), p_obstacle->mBetaIntersectionPoints.end() );

    p_obstacle->mpAlphaSeg->load( p_obstacle->mAlphaIntersectionPoints );
    p_obstacle->mpBetaSeg->load( p_obstacle->mBetaIntersectionPoints );
  }
  return true;
}

bool WorldMap::initRegions() {
  mRegionSets.clear();
  mSubregions.clear();

  // generate regions
  unsigned int index = 0;
  for( unsigned int i=0; i < mLineSegments.size(); i++ ) {
    if ( i == mLineSegments.size()-1 ) {
      std::list<Point2D> points = intersectWithBoundaries( mLineSegments[i], mLineSegments[0] );
      SubRegionSet* p_subregion_set = new SubRegionSet( points, index );
      index ++;
      p_subregion_set->mpLineSegmentsA =  mLineSegments[i];
      p_subregion_set->mpLineSegmentsB =  mLineSegments[0];
      mLineSegments[i]->mNeighbors.push_back( p_subregion_set );
      mLineSegments[0]->mNeighbors.push_back( p_subregion_set );
      mRegionSets.push_back( p_subregion_set );
    }
    else {
      std::list<Point2D> points = intersectWithBoundaries( mLineSegments[i], mLineSegments[i+1] );
      SubRegionSet* p_subregion_set = new SubRegionSet( points, index );
      index ++;
      p_subregion_set->mpLineSegmentsA =  mLineSegments[i];
      p_subregion_set->mpLineSegmentsB =  mLineSegments[i+1];
      mLineSegments[i]->mNeighbors.push_back( p_subregion_set );
      mLineSegments[i+1]->mNeighbors.push_back( p_subregion_set );
      mRegionSets.push_back( p_subregion_set );
    }
  }

  for( unsigned int i=0; i < mRegionSets.size(); i++ ) {
    SubRegionSet* p_subregions_set = mRegionSets[i];
    p_subregions_set->mSubregions = getSubregions( p_subregions_set );
    std::cout << "GENERATE FOR REGION " << i << " NUM_OF_SUB (" << p_subregions_set->mSubregions.size() << ")" << std::endl;
    for( unsigned int j=0; j < p_subregions_set->mSubregions.size(); j++ ) {
      SubRegion* p_subreg = p_subregions_set->mSubregions[j];
      mSubregions.push_back( p_subreg );
    }
    std::sort( p_subregions_set->mSubregions.begin(), p_subregions_set->mSubregions.end(), SubregionSort );
    for ( unsigned int sub_idx = 0; sub_idx < p_subregions_set->mSubregions.size(); sub_idx ++ ) {
      p_subregions_set->mSubregions[sub_idx]->mIndex = sub_idx;
    }
    p_subregions_set->mSubregions[0]->mIsConnectedToCentralPoint = true;
  }

  // associate line segments with subregions
  for( std::vector<SubRegionSet*>::iterator it = mRegionSets.begin();
       it != mRegionSets.end(); it++ ) {
    SubRegionSet* p_subregion_set = (*it);

    Obstacle* p_obstacle_a = p_subregion_set->mpLineSegmentsA->getObstacle();
    for( std::vector<SubRegion*>::iterator itr = p_subregion_set->mSubregions.begin();
         itr != p_subregion_set->mSubregions.end(); itr ++ ) {
      SubRegion* p_subregion = (*itr);

      for( std::vector<LineSubSegment*>::iterator itl = p_obstacle_a->mpAlphaSeg->mSubsegs.begin();
           itl != p_obstacle_a->mpAlphaSeg->mSubsegs.end(); itl++ ) {
        LineSubSegment* p_line_subseg = (*itl);
        if ( isIntersected( p_subregion->mPolygon , p_line_subseg->mSubseg , DELTA_TRIAL ) ) {
          p_line_subseg->mNeighbors.push_back( p_subregion );
          p_subregion->mNeighbors.push_back( p_line_subseg );
        }
      }
      for( std::vector<LineSubSegment*>::iterator itl = p_obstacle_a->mpBetaSeg->mSubsegs.begin();
           itl != p_obstacle_a->mpBetaSeg->mSubsegs.end(); itl++ ) {
        LineSubSegment* p_line_subseg = (*itl);
        if ( isIntersected( p_subregion->mPolygon , p_line_subseg->mSubseg , DELTA_TRIAL ) ) {
          p_line_subseg->mNeighbors.push_back( p_subregion );
          p_subregion->mNeighbors.push_back( p_line_subseg );
        }
      }
    }

    Obstacle* p_obstacle_b = p_subregion_set->mpLineSegmentsB->getObstacle();
    for( std::vector<SubRegion*>::iterator itr = p_subregion_set->mSubregions.begin();
         itr != p_subregion_set->mSubregions.end(); itr ++ ) {
      SubRegion* p_subregion = (*itr);

      for( std::vector<LineSubSegment*>::iterator itl = p_obstacle_b->mpAlphaSeg->mSubsegs.begin();
           itl != p_obstacle_b->mpAlphaSeg->mSubsegs.end(); itl++ ) {
        LineSubSegment* p_line_subseg = (*itl);
        if ( isIntersected( p_subregion->mPolygon , p_line_subseg->mSubseg , DELTA_TRIAL ) ) {
          p_line_subseg->mNeighbors.push_back( p_subregion );
          p_subregion->mNeighbors.push_back( p_line_subseg );
        }
      }
      for( std::vector<LineSubSegment*>::iterator itl = p_obstacle_b->mpBetaSeg->mSubsegs.begin();
           itl != p_obstacle_b->mpBetaSeg->mSubsegs.end(); itl++ ) {
        LineSubSegment* p_line_subseg = (*itl);
        if ( isIntersected( p_subregion->mPolygon , p_line_subseg->mSubseg , DELTA_TRIAL ) ) {
          p_line_subseg->mNeighbors.push_back( p_subregion );
          p_subregion->mNeighbors.push_back( p_line_subseg );
        }
      }
    }
  }

  // check neighbor num of line subsegment
  for( std::vector<LineSubSegmentSet*>::iterator it = mLineSegments.begin(); it != mLineSegments.end(); it ++ ) {
    LineSubSegmentSet* p_line_subseg_set = (*it);
    for( std::vector<LineSubSegment*>::iterator its = p_line_subseg_set->mSubsegs.begin();
         its != p_line_subseg_set->mSubsegs.end(); its ++ ) {
      LineSubSegment* p_line_subseg = (*its);
      if( p_line_subseg->mNeighbors.size() != 2 ) {
        std::cout << "ERROR: " << p_line_subseg->getName() << std::endl;
      }
    }
  }
  return true;
}

bool WorldMap::isIntersected( Polygon2D poly, Segment2D seg, double delta ) {

  double mid_x = ( CGAL::to_double( seg.source().x() ) + CGAL::to_double( seg.target().x() ) ) / 2.0;
  double mid_y = ( CGAL::to_double( seg.source().y() ) + CGAL::to_double( seg.target().y() ) ) / 2.0;

  if ( poly.bounded_side( Point2D( mid_x, mid_y ) ) != CGAL::ON_UNBOUNDED_SIDE  ) {
    return true;
  }
  Line2D perp_line = seg.supporting_line().perpendicular( Point2D( mid_x, mid_y ) );

  if( perp_line.is_vertical() ) {
      std::cout << "VERTICAL " << std::endl;
  }
  if( perp_line.is_horizontal() ) {
      std::cout << "HORIZONTAL " << std::endl;
  }

  double dx = CGAL::to_double ( perp_line.direction().dx() );
  double dy = CGAL::to_double ( perp_line.direction().dy() );
  if( std::abs(dx) < std::abs(dy) ) {
    double l_mid_x = mid_x - delta;
    double l_mid_y = CGAL::to_double( perp_line.y_at_x( l_mid_x ) );
    /*if ( poly.bounded_side( Point2D( l_mid_x, l_mid_y ) ) != CGAL::ON_UNBOUNDED_SIDE  ) {
      return true;
    }*/
    Polygon2D_set ps;
    ps.insert( poly );
    Polygon2D l_pep_line;
    l_pep_line.push_back( Point2D( mid_x, mid_y ) );
    l_pep_line.push_back( Point2D( l_mid_x, l_mid_y ) );
    if( ps.do_intersect( l_pep_line ) ) {
        return true;
    }

    double r_mid_x = mid_x + delta;
    double r_mid_y = CGAL::to_double( perp_line.y_at_x( r_mid_x ) );
    /*if ( poly.bounded_side( Point2D( r_mid_x, r_mid_y ) ) != CGAL::ON_UNBOUNDED_SIDE  ) {
      return true;
    }*/
    Polygon2D r_pep_line;
    r_pep_line.push_back( Point2D( mid_x, mid_y ) );
    r_pep_line.push_back( Point2D( r_mid_x, r_mid_y ) );
    if( ps.do_intersect( r_pep_line ) ) {
        return true;
    }
  }
  else {
    double l_mid_y = mid_y - delta;
    double l_mid_x = CGAL::to_double( perp_line.x_at_y( l_mid_y ) );
    /*if ( poly.bounded_side( Point2D( l_mid_x, l_mid_y ) ) != CGAL::ON_UNBOUNDED_SIDE  ) {
      return true;
    }*/
    Polygon2D_set ps;
    ps.insert( poly );
    Polygon2D l_pep_line;
    l_pep_line.push_back( Point2D( mid_x, mid_y ) );
    l_pep_line.push_back( Point2D( l_mid_x, l_mid_y ) );
    if( ps.do_intersect( l_pep_line ) ) {
        return true;
    }

    double r_mid_y = mid_y + delta;
    double r_mid_x = CGAL::to_double( perp_line.x_at_y( r_mid_y ) );
    /*if ( poly.bounded_side( Point2D( r_mid_x, r_mid_y ) ) != CGAL::ON_UNBOUNDED_SIDE  ) {
      return true;
    }*/
    Polygon2D r_pep_line;
    r_pep_line.push_back( Point2D( mid_x, mid_y ) );
    r_pep_line.push_back( Point2D( r_mid_x, r_mid_y ) );
    if( ps.do_intersect( r_pep_line ) ) {
        return true;
    }
  }
  return false;

}

std::list<Point2D> WorldMap::intersectWithBoundaries( LineSubSegmentSet* p_segment1, LineSubSegmentSet* p_segment2 ) {
  std::list<Point2D> points;
  Direction2D d1 = Ray2D( mCentralPoint, p_segment1->mSeg.target() ).direction();
  Direction2D d2 = Ray2D( mCentralPoint, p_segment2->mSeg.target() ).direction();

  if( d1 < d2 ) {
    for( unsigned int j=0; j < mCenterCornerLines.size(); j++ ) {
      Direction2D corner_d = mCenterCornerLines[j].direction();
      if( true == corner_d.counterclockwise_in_between( d1, d2 ) ) {
        points.push_back( mCenterCornerLines[j].target() );
      }
    }
    points.push_front( p_segment1->mSeg.target() );
    points.push_front( mCentralPoint );
    points.push_back( p_segment2->mSeg.target() );
  }
  else {
    for( unsigned int j=0; j < mCenterCornerLines.size(); j++ ) {
      Direction2D corner_d = mCenterCornerLines[j].direction();
      if( true == corner_d.counterclockwise_in_between( d1, d2 ) ) {
        if ( corner_d < d2 ) {
          points.push_front( mCenterCornerLines[j].target() );
        }
        else if ( d1 < corner_d ) {
          points.push_back( mCenterCornerLines[j].target() );
        }
      }
    }
    points.push_front( p_segment2->mSeg.target() );
    points.push_back( p_segment1->mSeg.target() );
    points.push_back( mCentralPoint );
    points.reverse();
  }
  return points;
}

bool WorldMap::isInObsBkLines(Point2D point) {
  for( std::vector<Line2D>::iterator it = mObsBkPairLines.begin(); it != mObsBkPairLines.end(); it++ ) {
    Line2D bk_line = (*it);
    if ( bk_line.has_on( point )==true ) {
      return true;
    }
  }
  return false;
}

double WorldMap::getDistanceToCentralPoint( Point2D point ) {
  double dist = 0.0;
  double cp_x = CGAL::to_double( mCentralPoint.x() );
  double cp_y = CGAL::to_double( mCentralPoint.y() );
  double p_x = CGAL::to_double( point.x() );
  double p_y = CGAL::to_double( point.y() );
  dist = pow( cp_x - p_x, 2 ) + pow( cp_y - p_y, 2 );
  dist = sqrt( dist );
  return dist;
}

Point2D* WorldMap::findIntersectionWithBoundary(Ray2D* p_ray) {
  for(std::vector<Segment2D>::iterator it=mBoundaryLines.begin(); it!=mBoundaryLines.end(); it++) {
    Segment2D seg = (*it);
    CGAL::Object result = intersection(seg, (*p_ray));
    Point2D* p = new Point2D();
    if ( CGAL::assign(*p, result) ) {
      return p;
    }
    if(p) {
      delete p;
      p = NULL;
    }
  }
  return NULL;
}

bool WorldMap::isInObstacle( Point2D point ) {
  for( std::vector<Obstacle*>::iterator it = mObstacles.begin(); it != mObstacles.end(); it++ ) {
    Obstacle * p_obstacle = (*it);
    if ( CGAL::ON_UNBOUNDED_SIDE != p_obstacle->mPgn.bounded_side( point ) ) {
      return true;
    }
  }
  return false;
}

SubRegion* WorldMap::inSubregion( Point2D point ) {
  for( unsigned int i = 0; i < getSubregionSet().size(); i ++ ) {
    SubRegionSet* p_subregion_set = getSubregionSet()[i]; 
    if( p_subregion_set ) {
      //std::cout << p_subregion_set->get_name() << std::endl;
      for( std::vector<SubRegion*>::iterator itr = p_subregion_set->mSubregions.begin();
           itr != p_subregion_set->mSubregions.end(); itr ++) { 
        SubRegion* p_subregion = (*itr);
        if ( p_subregion ) {
          if ( p_subregion->contains( point ) ) {
            return p_subregion;
          }
        }
      }
    } 
  }
  return NULL;
}

std::vector< std::pair<Point2D, Obstacle*> > WorldMap::intersect( Segment2D seg, Obstacle* p_obstacle ) {
  std::vector< std::pair<Point2D, Obstacle*> > points;
  if( p_obstacle ) {
    for(std::vector<Segment2D>::iterator it=p_obstacle->mBorderSegments.begin(); it!=p_obstacle->mBorderSegments.end(); it++) {
      Segment2D bound = (*it);
      CGAL::Object result = intersection(seg, bound);
      Point2D p;
      if ( CGAL::assign(p, result) ) {
        points.push_back( std::make_pair( p , p_obstacle ) );
      }
    }
  }
  return points;
}

std::vector<SubRegion*>  WorldMap::getSubregions( SubRegionSet* p_region ) {
  std::vector<SubRegion*> sr_set;
  if ( p_region == NULL ) {
    return sr_set;
  }
  std::vector<Polygon2D> candidates;
  std::vector<Polygon2D> new_candidates;
  candidates.push_back(p_region->mPolygon);
  for( std::vector< Obstacle* >::iterator itO = mObstacles.begin(); itO != mObstacles.end(); itO ++ ) {
    Obstacle* p_obs = (*itO);
    new_candidates.clear();
    for( std::vector<Polygon2D>::iterator itP = candidates.begin(); itP != candidates.end(); itP++ ) {
      Polygon2D subpoly = (*itP);

      if( do_intersect( subpoly , p_obs->mPgn ) ) {
        std::vector<PolygonWithHoles2D> res;
        CGAL::difference( subpoly, p_obs->mPgn, std::back_inserter(res) );
        //std::cout << "REG " << p_region->m_index << " INTERSECT " << p_obs->get_index();
        //std::cout << " DIFF SIZE " << res.size() << std::endl;
        for( std::vector< PolygonWithHoles2D >::iterator itP = res.begin();
             itP != res.end(); itP ++ ) {
          PolygonWithHoles2D poly = (*itP);
          if ( poly.has_holes() == false ) {
            Polygon2D poly_out = poly.outer_boundary();
            new_candidates.push_back( poly_out );
          }
        }
      }
      else {
        new_candidates.push_back( subpoly );
      }
    }
    //if ( new_candidates.size() > 0 ) {
    candidates = new_candidates;
    //}
    //std::cout << "REG " << p_region->m_index << " CAN " << candidates.size() << " NEWCAN " << new_candidates.size() << std::endl;
  }

  for( std::vector<Polygon2D>::iterator itP = candidates.begin(); itP != candidates.end(); itP++ ) {
    Polygon2D poly = (*itP);
    SubRegion* p_subregion = new SubRegion( poly , p_region );
    p_subregion->mDistToCp = getDistanceToCentralPoint( p_subregion->mCentroid );
    sr_set.push_back( p_subregion );
  }
  return sr_set;
}

void WorldMap::toXml( const std::string& filename )const {
  xmlDocPtr doc = xmlNewDoc( ( xmlChar* )( "1.0" ) );
  xmlNodePtr root = xmlNewDocNode( doc, NULL, ( xmlChar* )( "world" ), NULL );
  xmlDocSetRootElement( doc, root );
  toXml( doc, root );
  xmlSaveFormatFileEnc( filename.c_str(), doc, "UTF-8", 1 );
  xmlFreeDoc( doc );
  return;
}

void WorldMap::toXml( xmlDocPtr doc, xmlNodePtr root )const {
  std::stringstream width_str, height_str;
  width_str << mMapWidth;
  height_str << mMapHeight;
  xmlNewProp( root, ( const xmlChar* )( "width" ), ( const xmlChar* )( width_str.str().c_str() ) );
  xmlNewProp( root, ( const xmlChar* )( "height" ), ( const xmlChar* )( height_str.str().c_str() ) );
  std::stringstream cpx_str, cpy_str;
  cpx_str << mCentralPoint.x();
  cpy_str << mCentralPoint.y();
  xmlNewProp( root, ( const xmlChar* )( "central_x" ), ( const xmlChar* )( cpx_str.str().c_str() ) );
  xmlNewProp( root, ( const xmlChar* )( "central_y" ), ( const xmlChar* )( cpy_str.str().c_str() ) );

  for( unsigned int i=0; i<mObstacles.size(); i++ ) {
    Obstacle* p_obs = mObstacles[i];
    xmlNodePtr obs_node = xmlNewDocNode( doc, NULL, ( xmlChar* )( "obstacle" ), NULL );
    std::stringstream index_str, bkx_str, bky_str;
    index_str << p_obs->getIndex();
    bkx_str << p_obs->mBk.x();
    bky_str << p_obs->mBk.y();
    xmlNewProp( obs_node, ( const xmlChar* )( "index" ), ( const xmlChar* )( index_str.str().c_str() ) );
    xmlNewProp( obs_node, ( const xmlChar* )( "bk_x" ), ( const xmlChar* )( bkx_str.str().c_str() ) );
    xmlNewProp( obs_node, ( const xmlChar* )( "bk_y" ), ( const xmlChar* )( bky_str.str().c_str() ) );
    for( unsigned int j=0; j < p_obs->mPoints.size(); j++ ) {
      xmlNodePtr point_node = xmlNewDocNode( doc, NULL, ( xmlChar* )( "point" ), NULL );
      std::stringstream px_str, py_str;
      px_str << p_obs->mPoints[j].x();
      py_str << p_obs->mPoints[j].y();
      xmlNewProp( point_node, ( const xmlChar* )( "x" ), ( const xmlChar* )( px_str.str().c_str() ) );
      xmlNewProp( point_node, ( const xmlChar* )( "y" ), ( const xmlChar* )( py_str.str().c_str() ) );
      xmlAddChild( obs_node, point_node );
    } 
    xmlAddChild( root, obs_node );
  }
  return;
}

void WorldMap::fromXml( const std::string& filename ) {
  xmlDoc * doc = NULL;
  xmlNodePtr root = NULL;
  doc = xmlReadFile( filename.c_str(), NULL, 0 );
  if( doc != NULL ) {
    root = xmlDocGetRootElement( doc );
    if( root->type == XML_ELEMENT_NODE ){
      if( xmlStrcmp( root->name, ( const xmlChar* )( "world" ) ) == 0 ){
        fromXml(root);
      }
    }
    xmlFreeDoc( doc );
  }
  return;
}

void WorldMap::fromXml( xmlNodePtr root ) {
  int width = 0, height = 0;
  xmlChar* tmpw = xmlGetProp( root, ( const xmlChar* )( "width" ) );
  if( tmpw != NULL ) {
    width =  std::atoi( ( char* )( tmpw ) );
    xmlFree( tmpw );
  }
  xmlChar* tmph = xmlGetProp( root, ( const xmlChar* )( "height" ) );
  if( tmph != NULL ) {
    height =  std::atoi( ( char* )( tmph ) );
    xmlFree( tmph );
  }
  resize( width, height );
  xmlChar* tmpx = xmlGetProp( root, ( const xmlChar* )( "central_x" ) );
  xmlChar* tmpy = xmlGetProp( root, ( const xmlChar* )( "central_y" ) );
  if( tmpx != NULL && tmpy != NULL ) {
    mCentralPoint = Point2D( std::atoi( (char*)(tmpx) ) , std::atoi( (char*)(tmpy) ) );
    xmlFree( tmpx );
    xmlFree( tmpy );
  }
  for( xmlNodePtr l1 = root->children; l1; l1 = l1->next ){
    if( l1->type == XML_ELEMENT_NODE ) {
      if( xmlStrcmp( l1->name, ( const xmlChar* )( "obstacle" ) ) == 0 ){
        int idx = 0;
        double bk_x = 0, bk_y = 0;
        xmlChar* tmpIdx = xmlGetProp( l1, ( const xmlChar* )( "index" ) );
        if( tmpIdx != NULL ) {
          idx = std::atoi( ( char* )( tmpIdx ) );
          xmlFree( tmpIdx );
        }
        xmlChar* tmpBKx = xmlGetProp( l1, ( const xmlChar* )( "bk_x" ) );
        if( tmpBKx != NULL ) {
          bk_x = std::atoi( ( char* )( tmpBKx ) );
          xmlFree( tmpIdx );
        }
        xmlChar* tmpBKy = xmlGetProp( l1, ( const xmlChar* )( "bk_y" ) );
        if( tmpIdx != NULL ) {
          bk_y = std::atoi( ( char* )( tmpBKy ) );
          xmlFree( tmpBKy );
        }
        std::vector<Point2D> points;
        for( xmlNodePtr l2 = l1->children; l2; l2 = l2->next ){
          if( l2->type == XML_ELEMENT_NODE ) {
            if( xmlStrcmp( l2->name, ( const xmlChar* )( "point" ) ) == 0 ){
              int p_x = 0, p_y = 0;
              xmlChar* tmpPx = xmlGetProp( l2, ( const xmlChar* )( "x" ) );
              if( tmpPx != NULL ) {
                p_x = std::atoi( ( char* )( tmpPx ) );
                xmlFree( tmpPx );
              }
              xmlChar* tmpPy = xmlGetProp( l2, ( const xmlChar* )( "y" ) );
              if( tmpPy != NULL ) {
                p_y = std::atoi( ( char* )( tmpPy ) );
                xmlFree( tmpPy );
              }
              points.push_back(Point2D(p_x, p_y) );
            }
          }
        }

        Obstacle* p_obs = new Obstacle(points, idx, this);
        p_obs->mBk = Point2D( bk_x, bk_y );
        mObstacles.push_back(p_obs);
      }
    }
  }

  mObsBkPairLines.clear();
  for( unsigned int i=0; i < mObstacles.size(); i++ ) {
    for( unsigned int j=i+1; j < mObstacles.size(); j++ ) {
      Line2D pline(mObstacles[i]->mBk, mObstacles[j]->mBk);
      mObsBkPairLines.push_back(pline);
    }
  }
}

SubRegion* WorldMap::findSubregion( std::string name ) {
  SubRegion* p_subregion = NULL;
  for( std::vector<SubRegion*>::iterator it = mSubregions.begin();
       it != mSubregions.end(); it ++ ) {
     SubRegion* p_current_subregion = (*it);
     if( p_current_subregion->getName() == name ) {
       return p_current_subregion;
     }
  }
  return p_subregion;
}

LineSubSegment* WorldMap::findLinesubsegment( std::string name ) {
  LineSubSegment* p_linesubsegment = NULL; 
  for( std::vector<LineSubSegmentSet*>::iterator it =  mLineSegments.begin();
       it != mLineSegments.end(); it ++ ) {
    LineSubSegmentSet* p_current_linesubsegment_set = (*it);
    if( p_current_linesubsegment_set ) {
      for( std::vector< LineSubSegment* >::iterator its = p_current_linesubsegment_set->mSubsegs.begin();
           its != p_current_linesubsegment_set->mSubsegs.end(); its ++ ) {
        LineSubSegment* p_current_linesubsegment = (*its);
        if( p_current_linesubsegment ) {
          if( p_current_linesubsegment->getName() == name ) {
            return p_current_linesubsegment;
          }
        }
      }
    }
  }
  return p_linesubsegment;
}
    
SubRegion* WorldMap::findSubregion( Point2D point ) {
  SubRegion* p_subregion = NULL;
  for( std::vector<SubRegion*>::iterator it = mSubregions.begin();
       it != mSubregions.end(); it ++ ) {
     SubRegion* p_current_subregion = (*it);
     if( p_current_subregion->contains( point ) ) {
       return p_current_subregion;
     }
  }
  return p_subregion;
}

LineSubSegment* WorldMap::findLinesubsegment( Point2D point ) {
  LineSubSegment* p_linesubsegment = NULL; 
  for( std::vector<LineSubSegmentSet*>::iterator it =  mLineSegments.begin();
       it != mLineSegments.end(); it ++ ) {
    LineSubSegmentSet* p_current_linesubsegment_set = (*it);
    if( p_current_linesubsegment_set ) {
      for( std::vector< LineSubSegment* >::iterator its = p_current_linesubsegment_set->mSubsegs.begin();
           its != p_current_linesubsegment_set->mSubsegs.end(); its ++ ) {
        LineSubSegment* p_current_linesubsegment = (*its);
        if( p_current_linesubsegment ) {
          if( p_current_linesubsegment->contains( point ) ) {
            return p_current_linesubsegment;
          }
        }
      }
    }
  }
  return p_linesubsegment;
}

Obstacle* WorldMap::findObstacle( Point2D point ) {
  Obstacle* p_obstacle = NULL;
  for( unsigned int i=0; i < mObstacles.size(); i++ ) {
    Obstacle* p_current_obstacle = mObstacles[i];
    if( p_current_obstacle ) {
       if( p_current_obstacle->contains( point ) ) {
         return p_current_obstacle;
       }
    }
  }
  return p_obstacle;
}

std::ostream& operator<<( std::ostream& out, const WorldMap& other ) {

  out << "Size[" << other.getWidth() << "*" << other.getHeight() << "]  " << std::endl;
  for( unsigned int i=0; i < other.getObstacles().size(); i++ ) {
    out << other.getObstacles()[i] << std::endl;
  }
  return out;
}

} // homotopy

} // topologyPathPlanning
