#include <sstream>
#include <CGAL/squared_distance_2.h>
#include "topologyPathPlanning/homotopy/LineSubsegment.hpp"
#include "topologyPathPlanning/homotopy/Obstacle.hpp"
#include "topologyPathPlanning/homotopy/Worldmap.hpp"

using namespace std;

namespace topologyPathPlanning {

namespace homotopy {

IntersectionPoint::IntersectionPoint( Point2D point ) {
  mPoint = point;
  mDistToBk = 0.0;
  mpObstacle = NULL;
}

IntersectionPoint::~IntersectionPoint() {
  mpObstacle = NULL;
}

std::ostream& operator<<( std::ostream& out, const IntersectionPoint& other ) {
    out << "P[" << other.mPoint.x() << "," << other.mPoint.y() << "]";
    out << "  (" << other.mDistToBk << ")" << std::endl;
    return out;
}

LineSubSegment::LineSubSegment( Point2D pos_a, Point2D pos_b, LineSubSegmentSet* p_subseg_set, unsigned int index, bool is_connected_to_central_point ) {
  mIndex = index;
  mIsConnectedToCentralPoint = is_connected_to_central_point;
  mSubseg = Segment2D( pos_a, pos_b );
  mpSubsegSet = p_subseg_set;

  mConnectedToBoundary = false;
  mConnectedObstacles.clear();
}

LineSubSegment::~LineSubSegment() {
  mpSubsegSet = NULL;

  mConnectedToBoundary = false;
  mConnectedObstacles.clear();
}

string LineSubSegment::getName() {
  if( mpSubsegSet ) {
    stringstream ss;
    ss << mpSubsegSet->getName().c_str() << "-" << mIndex;
    return ss.str();
  }
  return "NA";
}
    
Point2D LineSubSegment::samplePosition() {

  float ratio = static_cast<float> (rand())/static_cast<float>(RAND_MAX);
  
  double s_x = CGAL::to_double( mSubseg.source().x() );
  double s_y = CGAL::to_double( mSubseg.source().y() );
  double t_x = CGAL::to_double( mSubseg.target().x() );
  double t_y = CGAL::to_double( mSubseg.target().y() );
  
  double n_x = ( ratio * s_x + (1.0 - ratio) * t_x ) ;
  double n_y = ( ratio * s_y + (1.0 - ratio) * t_y ) ;
  return Point2D( n_x, n_y );
}

bool LineSubSegment::contains( Point2D point ) {
  return mSubseg.has_on(point);
}

void LineSubSegment::toXml( const string& filename )const {
  xmlDocPtr doc = xmlNewDoc( ( xmlChar* )( "1.0" ) );
  xmlNodePtr root = xmlNewDocNode( doc, NULL, ( xmlChar* )( "root" ), NULL );
  xmlDocSetRootElement( doc, root );
  toXml( doc, root );
  xmlSaveFormatFileEnc( filename.c_str(), doc, "UTF-8", 1 );
  xmlFreeDoc( doc );
  return;
}

bool LineSubSegment::isConnected( Obstacle* p_obstacle ) {

  for( vector< Obstacle* >::iterator it = mConnectedObstacles.begin();
       it != mConnectedObstacles.end(); it++ ) {
    Obstacle* p_current_obstacle = (*it);
    if( p_current_obstacle ) {
      //cout << "COMPARE " << p_current_obstacle->get_name() << " " << p_obstacle->get_name() << endl;
      if( p_current_obstacle->getName() == p_obstacle->getName() ) {
        return true;
      }
    }
  }
  return false;
}

void LineSubSegment::toXml( xmlDocPtr doc, xmlNodePtr root )const {

}

void LineSubSegment::fromXml( const string& filename ) {
  xmlDoc * doc = NULL;
  xmlNodePtr root = NULL;
  doc = xmlReadFile( filename.c_str(), NULL, 0 );
  if( doc != NULL ){
    root = xmlDocGetRootElement( doc );
    if( root->type == XML_ELEMENT_NODE ){
      xmlNodePtr l1 = NULL;
      for( l1 = root->children; l1; l1 = l1->next ){
        if( l1->type == XML_ELEMENT_NODE ){
          if( xmlStrcmp( l1->name, ( const xmlChar* )( "line_subsegment" ) ) == 0 ){
            fromXml( l1 );
          }
        }
      }
    }
    xmlFreeDoc( doc );
  }
  return;
}

void LineSubSegment::fromXml( xmlNodePtr root ) {

}

std::ostream& operator<<( std::ostream& out, const LineSubSegment& other ) {
  out << "LineSubSegment [" << other.mSubseg.source().x() << "," << other.mSubseg.source().y() <<"] ==> ";
  out << "[" << other.mSubseg.target().x() <<"," << other.mSubseg.target().y() << "]" << std::endl;
  return out;
}

LineSubSegmentSet::LineSubSegmentSet( Point2D pos_a, Point2D pos_b, unsigned int type, Obstacle* p_obstacle ) {

  mSeg = Segment2D(pos_a, pos_b);
  mType = type;
  mpObstacle = p_obstacle;
  mSubsegs.clear();
}

LineSubSegmentSet::~LineSubSegmentSet() {
  for( vector< LineSubSegment* >::iterator it=mSubsegs.begin(); it != mSubsegs.end(); it++ ) {
    LineSubSegment* p_line_subseg = (*it);
    delete p_line_subseg;
    p_line_subseg = NULL;
  }
  mSubsegs.clear();
}

string LineSubSegmentSet::typeToStdString ( const unsigned int& type ) {
  switch( type ) {
  case( LINE_TYPE_ALPHA ):
    return "A";
  case( LINE_TYPE_BETA ):
    return "B";
  case( LINE_TYPE_UNKNOWN ):
  default:
    return "na";
  }
}

unsigned int LineSubSegmentSet::typeFromStdString ( const string& type_str ) {
  for( unsigned int i = 0; i < NUM_LINE_TYPE; i++ ){
    if( type_str == LineSubSegmentSet::typeToStdString( i ) ){
      return i;
    }
  }
  return LINE_TYPE_UNKNOWN;
}

bool LineSubSegmentSet::load( vector<IntersectionPoint>& intersections ) {

  if( mpObstacle == NULL ) {
    return false;
  }
  if( mpObstacle->getWorld() == NULL ) {
    return false;
  }
  if( intersections.size()==0 ) {
    return false;
  }

  mSubsegs.clear();
  if( mType == LINE_TYPE_ALPHA ) {

    unsigned int idx = 0;
    for( unsigned int i = 0; i < intersections.size()-1; i ++ ) {
      IntersectionPoint sec1 = intersections[i];
      IntersectionPoint sec2 = intersections[i+1];

      Point2D mid_point = Point2D( (sec1.mPoint.x()+sec2.mPoint.x())/2, (sec1.mPoint.y()+sec2.mPoint.y())/2);
      if (mpObstacle->getWorld()->isInObstacle(mid_point) == false ) {

        if ( sec1.mDistToBk <= mpObstacle->mDistBk2cp && sec2.mDistToBk >= mpObstacle->mDistBk2cp ) {
          LineSubSegment* p_subseg = new LineSubSegment( sec1.mPoint , mpObstacle->getWorld()->getCentralPoint(), this, idx, true );
          p_subseg->mConnectedObstacles.push_back( sec1.mpObstacle );
          p_subseg->mIsConnectedToCentralPoint = true;
          mSubsegs.push_back( p_subseg );
          idx += 1;

          LineSubSegment* p_subseg2 = new LineSubSegment( mpObstacle->getWorld()->getCentralPoint(), sec2.mPoint, this, idx, true );
          p_subseg2->mIsConnectedToCentralPoint = true;
          p_subseg2->mConnectedObstacles.push_back( sec2.mpObstacle );
          mSubsegs.push_back( p_subseg2 );
          idx += 1;
        }
        else {
          LineSubSegment* p_subseg = new LineSubSegment( sec1.mPoint, sec2.mPoint, this, idx );
          p_subseg->mConnectedObstacles.push_back( sec1.mpObstacle );
          p_subseg->mConnectedObstacles.push_back( sec2.mpObstacle );
          mSubsegs.push_back(p_subseg);
          idx += 1;
        }
      }
    }

    if( intersections[intersections.size()-1].mDistToBk > mpObstacle->mDistBk2cp ) {
      Point2D end_point = static_cast<Point2D>(mSeg.target());
      Point2D last_sec_point = intersections[intersections.size()-1].mPoint;

      Point2D mid_point = Point2D( (end_point.x()+last_sec_point.x())/2, (end_point.y()+last_sec_point.y())/2);
      if( mpObstacle->getWorld()->isInObstacle(mid_point) == false ) {
        LineSubSegment* p_subseg = new LineSubSegment( intersections[intersections.size()-1].mPoint, end_point, this, idx );
        p_subseg->mConnectedObstacles.push_back( intersections[intersections.size()-1].mpObstacle );
        p_subseg->mConnectedToBoundary = true;
        mSubsegs.push_back( p_subseg );
        idx += 1;
      }
    }
    else{
      Point2D end_point = static_cast<Point2D>(mSeg.target());
      Point2D last_sec_point = intersections[intersections.size()-1].mPoint;

      Point2D mid_point = Point2D( (end_point.x()+last_sec_point.x())/2, (end_point.y()+last_sec_point.y())/2);
      if( mpObstacle->getWorld()->isInObstacle(mid_point) == false ) {
        LineSubSegment* p_subseg = new LineSubSegment( intersections[intersections.size()-1].mPoint, mpObstacle->getWorld()->getCentralPoint(), this, idx, true );
        p_subseg->mConnectedObstacles.push_back( intersections[intersections.size()-1].mpObstacle );
        p_subseg->mIsConnectedToCentralPoint = true;
        mSubsegs.push_back( p_subseg );
        idx += 1;

        LineSubSegment* p_subseg2 = new LineSubSegment( mpObstacle->getWorld()->getCentralPoint(), end_point, this, idx, true );
        p_subseg2->mConnectedObstacles.push_back( intersections[intersections.size()-1].mpObstacle );
        p_subseg2->mIsConnectedToCentralPoint = true;
        p_subseg2->mConnectedToBoundary = true;
        mSubsegs.push_back( p_subseg2 );
        idx += 1;
      }
    }
  }
  else if( mType == LINE_TYPE_BETA ) {
    unsigned int idx = 0;
    for( unsigned int i = 0; i < intersections.size()-1; i++ ) {
      IntersectionPoint sec1 = intersections[i];
      IntersectionPoint sec2 = intersections[i+1];

      Point2D mid_point = Point2D( (sec1.mPoint.x()+sec2.mPoint.x())/2, (sec1.mPoint.y()+sec2.mPoint.y())/2);
      if (mpObstacle->getWorld()->isInObstacle(mid_point) == false ) {
        LineSubSegment* p_subseg = new LineSubSegment( sec1.mPoint, sec2.mPoint, this, idx );
        p_subseg->mConnectedObstacles.push_back( sec1.mpObstacle );
        p_subseg->mConnectedObstacles.push_back( sec2.mpObstacle );
        mSubsegs.push_back( p_subseg );
        idx += 1;
      }

    }
    Point2D end_point = static_cast<Point2D>(mSeg.target());
    Point2D last_sec_point = intersections[intersections.size()-1].mPoint;

    Point2D mid_point = Point2D( (end_point.x()+last_sec_point.x())/2, (end_point.y()+last_sec_point.y())/2);
    if (mpObstacle->getWorld()->isInObstacle(mid_point) == false ) {
      LineSubSegment* p_subseg = new LineSubSegment( last_sec_point, end_point, this, idx );
      p_subseg->mConnectedObstacles.push_back( intersections[intersections.size()-1].mpObstacle );
      p_subseg->mConnectedToBoundary = true;
      mSubsegs.push_back( p_subseg );
      idx += 1;
    }
  }else {
    return false;
  }
  return true;
}

string LineSubSegmentSet::getName() {
  stringstream ss;
  if( mType == LINE_TYPE_ALPHA ) {
    ss << "A";
  }
  else if( mType == LINE_TYPE_BETA ) {
    ss << "B";
  }
  ss << mpObstacle->getIndex();
  return ss.str();
}

void LineSubSegmentSet::toXml( const string& filename )const {
  xmlDocPtr doc = xmlNewDoc( ( xmlChar* )( "1.0" ) );
  xmlNodePtr root = xmlNewDocNode( doc, NULL, ( xmlChar* )( "root" ), NULL );
  xmlDocSetRootElement( doc, root );
  toXml( doc, root );

  xmlSaveFormatFileEnc( filename.c_str(), doc, "UTF-8", 1 );
  xmlFreeDoc( doc );
  return;
}

void LineSubSegmentSet::toXml( xmlDocPtr doc, xmlNodePtr root )const {
  xmlNodePtr node = xmlNewDocNode( doc, NULL, ( const xmlChar* )( "line_subsegment_set" ), NULL );
  for( unsigned int i=0; i < mSubsegs.size(); i++ ) {
    if( mSubsegs[i] != NULL ) {
      mSubsegs[i]->toXml( doc, node );
    }
  }
  xmlAddChild( root, node );
  return;
}

void LineSubSegmentSet::fromXml( const string& filename ) {
  xmlDoc * doc = NULL;
  xmlNodePtr root = NULL;
  doc = xmlReadFile( filename.c_str(), NULL, 0 );
  if( doc != NULL ){
    root = xmlDocGetRootElement( doc );
    if( root->type == XML_ELEMENT_NODE ){
      xmlNodePtr l1 = NULL;
      for( l1 = root->children; l1; l1 = l1->next ){
        if( l1->type == XML_ELEMENT_NODE ){
          if( xmlStrcmp( l1->name, ( const xmlChar* )( "line_subsegment_set" ) ) == 0 ){
            fromXml( l1 );
          }
        }
      }
    }
    xmlFreeDoc( doc );
  }
  return;
}

void LineSubSegmentSet::fromXml( xmlNodePtr root ) {

}

std::ostream& operator<<( std::ostream& out, const LineSubSegmentSet& other ) {
    for( unsigned int i =0; i < other.mSubsegs.size(); i++ ) {
      LineSubSegment* seg = other.mSubsegs[i];
      out << (*seg) << std::endl;
    }
    return out;
}

} // homotopy

} // topologyPathPlanning
