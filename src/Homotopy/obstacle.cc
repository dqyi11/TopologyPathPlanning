#include <stdlib.h>
#include <CGAL/centroid.h>
#include <CGAL/squared_distance_2.h>
#include "obstacle.h"
#include "worldmap.h"

using namespace homotopy;

Obstacle::Obstacle(std::vector<Point2D> points, int index, WorldMap* world ){
  _index = index;
  _p_world = world;

  m_dist_bk2cp = 0.0;

  m_points.clear();
  m_border_segments.clear();
  for( std::vector<Point2D>::iterator it = points.begin(); it != points.end(); it++ ) {
    Point2D pos = (*it);
    m_points.push_back(pos);
    m_pgn.push_back(pos);
  }

  for( unsigned int i=0; i< m_points.size(); i++ ) {
    if( i < m_points.size()-1 ) {
      Segment2D seg( m_points[i], m_points[i+1] );
      m_border_segments.push_back(seg);
    }
    else {
      Segment2D seg( m_points[i], m_points[0] );
      m_border_segments.push_back(seg);
    }
  }

  if ( m_pgn.orientation() == CGAL::CLOCKWISE ) {
    m_pgn.reverse_orientation();
  }
  _min_x = m_pgn.bbox().xmin();
  _min_y = m_pgn.bbox().ymin();
  _max_x = m_pgn.bbox().xmax();
  _max_y = m_pgn.bbox().ymax();

  _centroid = Point2D( (_min_x + _max_x)/2 , (_min_y + _max_y)/2 );
}

Obstacle::~Obstacle() {

  if ( mp_alpha_seg ) {
    delete mp_alpha_seg;
    mp_alpha_seg = NULL;
  }
  if ( mp_beta_seg ) {
    delete mp_beta_seg;
    mp_beta_seg = NULL;
  }

  m_points.clear();
  m_border_segments.clear();
}

double Obstacle::distance_to_bk( Point2D& point ) {
  double dist = 0.0;
  double bk_x = CGAL::to_double( m_bk.x() );
  double bk_y = CGAL::to_double( m_bk.y() );
  double p_x = CGAL::to_double( point.x() );
  double p_y = CGAL::to_double( point.y() );
  dist = pow( bk_x - p_x , 2 ) + pow( bk_y - p_y , 2 );
  //dist = CGAL::squared_distance( m_bk , point );
  dist = sqrt(dist);
  return dist;
}

bool Obstacle::contains( Point2D point ) {
  if ( CGAL::ON_UNBOUNDED_SIDE != m_pgn.bounded_side( point ) ) { 
    return true;
  }
  return false;
}

Point2D Obstacle::sample_position() {

  bool found = false;
  while (found == false) {
    float x_ratio = static_cast<float> (rand())/static_cast<float>(RAND_MAX);
    float y_ratio = static_cast<float> (rand())/static_cast<float>(RAND_MAX);
    int rnd_x = static_cast<int>(x_ratio*(_max_x - _min_x)) + _min_x;
    int rnd_y = static_cast<int>(y_ratio*(_max_y - _min_y)) + _min_y;

    Point2D rnd_point(rnd_x, rnd_y);
    if ( CGAL::ON_BOUNDED_SIDE == m_pgn.bounded_side( rnd_point ) ) {
      return rnd_point;
    }
  }
  return _centroid;
}

void Obstacle::to_xml( const std::string& filename )const {
  xmlDocPtr doc = xmlNewDoc( ( xmlChar* )( "1.0" ) );
  xmlNodePtr root = xmlNewDocNode( doc, NULL, ( xmlChar* )( "root" ), NULL );
  xmlDocSetRootElement( doc, root );
  to_xml( doc, root );
  xmlSaveFormatFileEnc( filename.c_str(), doc, "UTF-8", 1 );
  xmlFreeDoc( doc );
  return;
}

void Obstacle::to_xml( xmlDocPtr doc, xmlNodePtr root )const {

}

void Obstacle::from_xml( const std::string& filename ) {
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
            from_xml( l1 );
          }
        }
      }
    }
    xmlFreeDoc( doc );
  }
  return;
}

void Obstacle::from_xml( xmlNodePtr root ) {

}

namespace homotopy {

  std::ostream& operator<<( std::ostream& out, const Obstacle& other ) {
    out << "bk[" << other.m_bk.x() << "," << other.m_bk.y() << "]";
    out << "    (" << other.m_dist_bk2cp << ")";
    return out;
  }

}
