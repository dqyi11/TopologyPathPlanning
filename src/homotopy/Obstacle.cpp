#include <sstream>
#include <stdlib.h>
#include <CGAL/centroid.h>
#include <CGAL/squared_distance_2.h>
#include "topologyPathPlanning/homotopy/Obstacle.hpp"
#include "topologyPathPlanning/homotopy/Worldmap.hpp"

using namespace std;

namespace topologyPathPlanning {

namespace homotopy {

Obstacle::Obstacle(vector<Point2D> points, int index, WorldMap* world ){
  mIndex = index;
  mpWorld = world;

  mDistBk2cp = 0.0;

  mPoints.clear();
  mBorderSegments.clear();
  for( vector<Point2D>::iterator it = points.begin(); it != points.end(); it++ ) {
    Point2D pos = (*it);
    mPoints.push_back(pos);
    mPgn.push_back(pos);
  }

  for( unsigned int i=0; i< mPoints.size(); i++ ) {
    if( i < mPoints.size()-1 ) {
      Segment2D seg( mPoints[i], mPoints[i+1] );
      mBorderSegments.push_back(seg);
    }
    else {
      Segment2D seg( mPoints[i], mPoints[0] );
      mBorderSegments.push_back(seg);
    }
  }

  if ( mPgn.orientation() == CGAL::CLOCKWISE ) {
    mPgn.reverse_orientation();
  }
  mMinX = mPgn.bbox().xmin();
  mMinY = mPgn.bbox().ymin();
  mMaxX = mPgn.bbox().xmax();
  mMaxY = mPgn.bbox().ymax();

  mCentroid = Point2D( (mMinX + mMaxX)/2 , (mMinY + mMaxY)/2 );
}

Obstacle::~Obstacle() {

  if ( mpAlphaSeg ) {
    delete mpAlphaSeg;
    mpAlphaSeg = NULL;
  }
  if ( mpBetaSeg ) {
    delete mpBetaSeg;
    mpBetaSeg = NULL;
  }

  mPoints.clear();
  mBorderSegments.clear();
}

double Obstacle::distanceToBk( Point2D& point ) {
  double dist = 0.0;
  double bk_x = CGAL::to_double( mBk.x() );
  double bk_y = CGAL::to_double( mBk.y() );
  double p_x = CGAL::to_double( point.x() );
  double p_y = CGAL::to_double( point.y() );
  dist = pow( bk_x - p_x , 2 ) + pow( bk_y - p_y , 2 );
  //dist = CGAL::squared_distance( m_bk , point );
  dist = sqrt(dist);
  return dist;
}

bool Obstacle::contains( Point2D point ) {
  if ( CGAL::ON_UNBOUNDED_SIDE != mPgn.bounded_side( point ) ) { 
    return true;
  }
  return false;
}

Point2D Obstacle::samplePosition() {

  bool found = false;
  while (found == false) {
    float x_ratio = static_cast<float> (rand())/static_cast<float>(RAND_MAX);
    float y_ratio = static_cast<float> (rand())/static_cast<float>(RAND_MAX);
    int rnd_x = static_cast<int>(x_ratio*(mMaxX - mMinX)) + mMinX;
    int rnd_y = static_cast<int>(y_ratio*(mMaxY - mMinY)) + mMinY;

    Point2D rnd_point(rnd_x, rnd_y);
    if ( CGAL::ON_BOUNDED_SIDE == mPgn.bounded_side( rnd_point ) ) {
      return rnd_point;
    }
  }
  return mCentroid;
}

string Obstacle::getName() {
  stringstream ss;
  ss << "OBS" << mIndex;
  return ss.str(); 
}

void Obstacle::toXml( const string& filename )const {
  xmlDocPtr doc = xmlNewDoc( ( xmlChar* )( "1.0" ) );
  xmlNodePtr root = xmlNewDocNode( doc, NULL, ( xmlChar* )( "root" ), NULL );
  xmlDocSetRootElement( doc, root );
  toXml( doc, root );
  xmlSaveFormatFileEnc( filename.c_str(), doc, "UTF-8", 1 );
  xmlFreeDoc( doc );
  return;
}

void Obstacle::toXml( xmlDocPtr doc, xmlNodePtr root )const {

}

void Obstacle::fromXml( const string& filename ) {
  xmlDoc * doc = NULL;
  xmlNodePtr root = NULL;
  doc = xmlReadFile( filename.c_str(), NULL, 0 );
  if( doc != NULL ){
    root = xmlDocGetRootElement( doc );
    if( root->type == XML_ELEMENT_NODE ){
      xmlNodePtr l1 = NULL;
      for( l1 = root->children; l1; l1 = l1->next ){
        if( l1->type == XML_ELEMENT_NODE ){
          if( xmlStrcmp( l1->name, ( const xmlChar* )( "obstacle" ) ) == 0 ){
            fromXml( l1 );
          }
        }
      }
    }
    xmlFreeDoc( doc );
  }
  return;
}

void Obstacle::fromXml( xmlNodePtr root ) {

}


  std::ostream& operator<<( std::ostream& out, const Obstacle& other ) {
    out << "bk[" << other.mBk.x() << "," << other.mBk.y() << "]";
    out << "    (" << other.mDistBk2cp << ")";
    return out;
  }


} // homotopy

} // topologyPathPlanning
