#include <sstream>
#include "opencv2/core/core.hpp"
#include <CGAL/intersections.h>
#include <CGAL/Polygon_2_algorithms.h>
#include <CGAL/Boolean_set_operations_2.h>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include "worldmap.h"

bool Segment2DSort(const Segment2D& lhs, const Segment2D& rhs) {
  return lhs.direction() < rhs.direction();
}

bool LineSubSegmentSetSort(const LineSubSegmentSet* lhs, const LineSubSegmentSet* rhs) {
    return lhs->m_seg.direction() < rhs->m_seg.direction();
}

WorldMap::WorldMap() {
    _map_width = 0;
    _map_height = 0;

    _sample_width_scale = 0;
    _sample_height_scale = 0;

    _obstacles.clear();
    _boundary_lines.clear();
    _obs_bk_pair_lines.clear();
    _line_segments.clear();
    _center_corner_lines.clear();

    _central_point = Point2D(0, 0);
}

WorldMap::WorldMap( int width, int height ) {
    WorldMap();
    resize(width, height);
}

WorldMap::~WorldMap() {

    for( unsigned int i=0; i < _obstacles.size(); i++ ) {
        Obstacle* po = _obstacles[i];
        delete po;
        po = NULL;
    }
    _obstacles.clear();
    _boundary_lines.clear();
    _obs_bk_pair_lines.clear();
    _line_segments.clear();
    _center_corner_lines.clear();
}

bool WorldMap::resize( int width, int height ) {
    if ( width < 0 || height < 0 ) {
        return false;
    }
    _map_width = width;
    _map_height = height;
    _sample_width_scale = _map_width/5;
    _sample_height_scale = _map_width/5;
    _central_point = Point2D(width/2, height/2);

    return false;
}

bool WorldMap::load_obstacle_info( std::vector< std::vector<Point2D> > polygons ) {
    _obstacles.clear();
    int obs_idx = 0;
    for( std::vector< std::vector<Point2D> >::iterator it=polygons.begin(); it!=polygons.end(); it++ ) {
        std::vector<Point2D> points = (*it);
        Obstacle* p_obs = new Obstacle(points, obs_idx, this);
        obs_idx ++;
        _obstacles.push_back(p_obs);
    }

    return true;
}

bool WorldMap::init( bool rand_init_points ) {
    if ( rand_init_points == true ) {
        _init_points();
    }
    _init_rays();
    _init_segments();
    _init_regions();
    return true;
}

bool WorldMap::_init_points() {
    // select random point for each obstacle
    for( std::vector<Obstacle*>::iterator it = _obstacles.begin(); it != _obstacles.end(); it++ ) {
        Obstacle * p_obstacle = (*it);
        p_obstacle->m_bk = p_obstacle->sample_position();
    }

    _obs_bk_pair_lines.clear();
    for( unsigned int i=0; i < _obstacles.size(); i++ ) {
        for( unsigned int j=i+1; j < _obstacles.size(); j++ ) {
            Line2D pline(_obstacles[i]->m_bk, _obstacles[j]->m_bk);
            _obs_bk_pair_lines.push_back(pline);
        }
    }

    // select central point c
    bool found_cp = false;
    while( found_cp == false ) {
        if ( (false == _is_in_obstacle(_central_point)) && (false == _is_in_obs_bk_lines(_central_point)) ) {
            found_cp = true;
        }
        else {
            float x_ratio = static_cast<float> (rand())/static_cast<float>(RAND_MAX);
            float y_ratio = static_cast<float> (rand())/static_cast<float>(RAND_MAX);
            int cp_x = static_cast<int>( x_ratio*static_cast<float>(_sample_width_scale) ) + _map_width /2;
            int cp_y = static_cast<int>( y_ratio*static_cast<float>(_sample_height_scale) ) + _map_height /2;
            _central_point = Point2D(cp_x, cp_y);
        }
    }

    for( std::vector<Obstacle*>::iterator it = _obstacles.begin(); it != _obstacles.end(); it++ ) {
        Obstacle * p_obstacle = (*it);
        p_obstacle->m_dist_bk2cp = p_obstacle->distance_to_bk(_central_point);
    }

    return true;
}

bool WorldMap::_init_rays() {
    // init four boundary line
    _boundary_lines.clear();
    _x_min_line = Segment2D(Point2D(0,0), Point2D(_map_width-1,0));
    _y_min_line = Segment2D(Point2D(0,0), Point2D(0,_map_height-1));
    _x_max_line = Segment2D(Point2D(0,_map_height-1), Point2D(_map_width-1,_map_height-1));
    _y_max_line = Segment2D(Point2D(_map_width-1,0), Point2D(_map_width-1,_map_height-1));
    _boundary_lines.push_back(_x_min_line);
    _boundary_lines.push_back(_y_max_line);
    _boundary_lines.push_back(_x_max_line);
    _boundary_lines.push_back(_y_min_line);

    // init lines from center point to four corners
    _center_corner_lines.push_back(Segment2D(_central_point, Point2D(0,0)));
    _center_corner_lines.push_back(Segment2D(_central_point, Point2D(_map_width, 0)));
    _center_corner_lines.push_back(Segment2D(_central_point, Point2D(_map_width, _map_height)));
    _center_corner_lines.push_back(Segment2D(_central_point, Point2D(0, _map_height)));
    std::sort(_center_corner_lines.begin(), _center_corner_lines.end(), Segment2DSort);

    // init alpha and beta segments
    for( std::vector<Obstacle*>::iterator it=_obstacles.begin(); it!=_obstacles.end(); it++) {
        Obstacle* p_obstacle = (*it);

        Ray2D alpha_ray( _central_point, Point2D(2*_central_point.x()-p_obstacle->m_bk.x(), 2*_central_point.y()-p_obstacle->m_bk.y()) );
        Ray2D beta_ray( _central_point, p_obstacle->m_bk );

        Point2D * p_a_pt = _find_intersection_with_boundary( &alpha_ray );
        Point2D * p_b_pt = _find_intersection_with_boundary( &beta_ray );

        if ( p_a_pt ) {
            p_obstacle->mp_alpha_seg = new LineSubSegmentSet( p_obstacle->m_bk, *p_a_pt, LINE_TYPE_ALPHA, p_obstacle );
            _line_segments.push_back(p_obstacle->mp_alpha_seg);
        }
        if ( p_b_pt ) {
            p_obstacle->mp_beta_seg = new LineSubSegmentSet( p_obstacle->m_bk, *p_b_pt, LINE_TYPE_BETA, p_obstacle );
            _line_segments.push_back(p_obstacle->mp_beta_seg);
        }
    }

    std::sort(_line_segments.begin(), _line_segments.end(), LineSubSegmentSetSort);

    return true;
}

bool WorldMap::_init_segments() {
    for( std::vector<Obstacle*>::iterator it=_obstacles.begin(); it!=_obstacles.end(); it++) {
        Obstacle* p_obstacle = (*it);
        p_obstacle->m_alpha_intersection_points.clear();
        p_obstacle->m_beta_intersection_points.clear();

        for( std::vector<Obstacle*>::iterator itr=_obstacles.begin(); itr!=_obstacles.end(); itr++) {
            Obstacle* p_ref_obstacle = (*itr);
            // check alpha_seg with obstacles
            std::vector< Point2D > a_ints = _intersect(p_obstacle->mp_alpha_seg->m_seg, p_ref_obstacle->m_border_segments);
            std::vector< Point2D > b_ints = _intersect(p_obstacle->mp_beta_seg->m_seg, p_ref_obstacle->m_border_segments);
            for( std::vector< Point2D >::iterator itp = a_ints.begin(); itp != a_ints.end(); itp++ ) {
                Point2D p = (*itp);
                IntersectionPoint ip;
                ip.m_point = p;
                ip.m_dist_to_bk = p_obstacle->distance_to_bk(p);
                p_obstacle->m_alpha_intersection_points.push_back(ip);
            }
            for( std::vector< Point2D >::iterator itp = b_ints.begin(); itp != b_ints.end(); itp++ ) {
                Point2D p = (*itp);
                IntersectionPoint ip;
                ip.m_point = p;
                ip.m_dist_to_bk = p_obstacle->distance_to_bk(p);
                p_obstacle->m_beta_intersection_points.push_back(ip);
            }
        }

        std::sort(p_obstacle->m_alpha_intersection_points.begin(), p_obstacle->m_alpha_intersection_points.end());
        std::sort(p_obstacle->m_beta_intersection_points.begin(), p_obstacle->m_beta_intersection_points.end());

        p_obstacle->mp_alpha_seg->load( p_obstacle->m_alpha_intersection_points );
        p_obstacle->mp_beta_seg->load( p_obstacle->m_beta_intersection_points );
    }
    return true;
}

bool WorldMap::_init_regions() {
    _regionSets.clear();
    unsigned int index = 0;
    for( unsigned int i=0; i < _line_segments.size(); i++ ) {
        if ( i == _line_segments.size()-1 ) {
            std::list<Point2D> points = _intersect_with_boundaries( _line_segments[i], _line_segments[0] );
            SubRegionSet* p_subregion_set = new SubRegionSet( points, index );
            index ++;
            _regionSets.push_back( p_subregion_set );
        }
        else {
            std::list<Point2D> points = _intersect_with_boundaries( _line_segments[i], _line_segments[i+1] );
            SubRegionSet* p_subregion_set = new SubRegionSet( points, index );
            index ++;
            _regionSets.push_back( p_subregion_set );
        }
    }

    for( unsigned int i=0; i < _regionSets.size(); i++ ) {
        SubRegionSet* p_subregions_set = _regionSets[i];
        p_subregions_set->m_subregions = _get_subregions(p_subregions_set);
        std::cout << "GENERATE FOR REGION " << i << " NUM_OF_SUB (" << p_subregions_set->m_subregions.size() << ")" << std::endl;
    }

    return true;
}

std::list<Point2D> WorldMap::_intersect_with_boundaries( LineSubSegmentSet* p_segment1, LineSubSegmentSet* p_segment2 ) {
    std::list<Point2D> points;
    Direction2D d1 = Ray2D( _central_point, p_segment1->m_seg.target() ).direction();
    Direction2D d2 = Ray2D( _central_point, p_segment2->m_seg.target() ).direction();

    if( d1 < d2 ) {

        for( unsigned int j=0; j < _center_corner_lines.size(); j++ ) {
            Direction2D corner_d = _center_corner_lines[j].direction();
            if( true == corner_d.counterclockwise_in_between( d1, d2 ) ) {
                points.push_back( _center_corner_lines[j].target() );
            }
        }
        points.push_front( p_segment1->m_seg.target() );
        points.push_front( _central_point );
        points.push_back( p_segment2->m_seg.target() );
    }
    else {
        for( unsigned int j=0; j < _center_corner_lines.size(); j++ ) {
            Direction2D corner_d = _center_corner_lines[j].direction();
            if( true == corner_d.counterclockwise_in_between( d1, d2 ) ) {
                if ( corner_d < d2 ) {
                    points.push_front( _center_corner_lines[j].target() );
                }
                else if ( d1 < corner_d ) {
                    points.push_back( _center_corner_lines[j].target() );
                }
            }
        }
        points.push_front( p_segment2->m_seg.target() );
        points.push_back( p_segment1->m_seg.target() );
        points.push_back( _central_point );
        points.reverse();
    }
    return points;
}

bool WorldMap::_is_in_obs_bk_lines(Point2D point) {
    for( std::vector<Line2D>::iterator it = _obs_bk_pair_lines.begin(); it != _obs_bk_pair_lines.end(); it++ ) {
        Line2D bk_line = (*it);
        if ( bk_line.has_on( point )==true ) {
            return true;
        }
    }
    return false;
}

Point2D* WorldMap::_find_intersection_with_boundary(Ray2D* p_ray) {
    for(std::vector<Segment2D>::iterator it=_boundary_lines.begin(); it!=_boundary_lines.end(); it++) {
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

bool WorldMap::_is_in_obstacle( Point2D point ) {
    for( std::vector<Obstacle*>::iterator it = _obstacles.begin(); it != _obstacles.end(); it++ ) {
        Obstacle * p_obstacle = (*it);
        if ( CGAL::ON_UNBOUNDED_SIDE != p_obstacle->m_pgn.bounded_side( point ) ) {
            return true;
        }
    }
    return false;
}

std::vector<Point2D> WorldMap::_intersect( Segment2D seg, std::vector<Segment2D> segments ) {
    std::vector<Point2D> points;
    for(std::vector<Segment2D>::iterator it=segments.begin(); it!=segments.end(); it++) {
        Segment2D bound = (*it);
        CGAL::Object result = intersection(seg, bound);
        Point2D p;
        if ( CGAL::assign(p, result) ) {
            points.push_back(p);

        }
    }
    return points;
}

std::vector<SubRegion*>  WorldMap::_get_subregions( SubRegionSet* p_region ) {
    std::vector<SubRegion*> sr_set;
    if ( p_region == NULL ) {
        return sr_set;
    }
    std::vector<Polygon2D> candidates;
    std::vector<Polygon2D> new_candidates;
    candidates.push_back(p_region->m_polygon);
    for( std::vector< Obstacle* >::iterator itO = _obstacles.begin(); itO != _obstacles.end(); itO ++ ) {
        Obstacle* p_obs = (*itO);
        new_candidates.clear();
        for( std::vector<Polygon2D>::iterator itP = candidates.begin(); itP != candidates.end(); itP++ ) {
            Polygon2D subpoly = (*itP);

            if( do_intersect( subpoly , p_obs->m_pgn ) ) {
                std::vector<PolygonWithHoles2D> res;
                CGAL::difference( subpoly, p_obs->m_pgn, std::back_inserter(res) );
                std::cout << "REG " << p_region->m_index << " INTERSECT " << p_obs->get_index();
                std::cout << " DIFF SIZE " << res.size() << std::endl;
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
        SubRegion* p_subregion = new SubRegion( poly );
        sr_set.push_back( p_subregion );
    }
    return sr_set;
}

void WorldMap::to_xml( const std::string& filename )const {
    xmlDocPtr doc = xmlNewDoc( ( xmlChar* )( "1.0" ) );
    xmlNodePtr root = xmlNewDocNode( doc, NULL, ( xmlChar* )( "world" ), NULL );
    xmlDocSetRootElement( doc, root );
    to_xml( doc, root );
    xmlSaveFormatFileEnc( filename.c_str(), doc, "UTF-8", 1 );
    xmlFreeDoc( doc );
    return;
}

void WorldMap::to_xml( xmlDocPtr doc, xmlNodePtr root )const {
    std::stringstream width_str, height_str;
    width_str << _map_width;
    height_str << _map_height;
    xmlNewProp( root, ( const xmlChar* )( "width" ), ( const xmlChar* )( width_str.str().c_str() ) );
    xmlNewProp( root, ( const xmlChar* )( "height" ), ( const xmlChar* )( height_str.str().c_str() ) );
    std::stringstream cpx_str, cpy_str;
    cpx_str << _central_point.x();
    cpy_str << _central_point.y();
    xmlNewProp( root, ( const xmlChar* )( "central_x" ), ( const xmlChar* )( cpx_str.str().c_str() ) );
    xmlNewProp( root, ( const xmlChar* )( "central_y" ), ( const xmlChar* )( cpy_str.str().c_str() ) );

    for( unsigned int i=0; i<_obstacles.size(); i++ ) {
        Obstacle* p_obs = _obstacles[i];
        xmlNodePtr obs_node = xmlNewDocNode( doc, NULL, ( xmlChar* )( "obstacle" ), NULL );
        std::stringstream index_str, bkx_str, bky_str;
        index_str << p_obs->get_index();
        bkx_str << p_obs->m_bk.x();
        bky_str << p_obs->m_bk.y();
        xmlNewProp( obs_node, ( const xmlChar* )( "index" ), ( const xmlChar* )( index_str.str().c_str() ) );
        xmlNewProp( obs_node, ( const xmlChar* )( "bk_x" ), ( const xmlChar* )( bkx_str.str().c_str() ) );
        xmlNewProp( obs_node, ( const xmlChar* )( "bk_y" ), ( const xmlChar* )( bky_str.str().c_str() ) );
        for( unsigned int j=0; j < p_obs->m_points.size(); j++ ) {
            xmlNodePtr point_node = xmlNewDocNode( doc, NULL, ( xmlChar* )( "point" ), NULL );
            std::stringstream px_str, py_str;
            px_str << p_obs->m_points[j].x();
            py_str << p_obs->m_points[j].y();
            xmlNewProp( point_node, ( const xmlChar* )( "x" ), ( const xmlChar* )( px_str.str().c_str() ) );
            xmlNewProp( point_node, ( const xmlChar* )( "y" ), ( const xmlChar* )( py_str.str().c_str() ) );
            xmlAddChild( obs_node, point_node );
        } 
        xmlAddChild( root, obs_node );
    }
    return;
}

void WorldMap::from_xml( const std::string& filename ) {
    xmlDoc * doc = NULL;
    xmlNodePtr root = NULL;
    doc = xmlReadFile( filename.c_str(), NULL, 0 );
    if( doc != NULL ) {
        root = xmlDocGetRootElement( doc );
        if( root->type == XML_ELEMENT_NODE ){
            if( xmlStrcmp( root->name, ( const xmlChar* )( "world" ) ) == 0 ){
                from_xml(root);
            }
        }
        xmlFreeDoc( doc );
    }
    return;
}

void WorldMap::from_xml( xmlNodePtr root ) {
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
        _central_point = Point2D( std::atoi( (char*)(tmpx) ) , std::atoi( (char*)(tmpy) ) );
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
                p_obs->m_bk = Point2D( bk_x, bk_y );
                _obstacles.push_back(p_obs);
            }
        }
    }

    _obs_bk_pair_lines.clear();
    for( unsigned int i=0; i < _obstacles.size(); i++ ) {
        for( unsigned int j=i+1; j < _obstacles.size(); j++ ) {
            Line2D pline(_obstacles[i]->m_bk, _obstacles[j]->m_bk);
            _obs_bk_pair_lines.push_back(pline);
        }
    }
}

std::ostream& operator<<( std::ostream& out, const WorldMap& other ) {

    out << "Size[" << other.get_width() << "*" << other.get_height() << "]  " << std::endl;
    for( unsigned int i=0; i < other.get_obstacles().size(); i++ ) {
        out << other.get_obstacles()[i] << std::endl;
    }
    return out;
}
