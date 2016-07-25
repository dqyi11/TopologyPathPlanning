#include <CGAL/intersections.h>
#include "tpp/homotopy/reference_frames.h"

#define MERGED_CENTER_SUBREGION "SC"

using namespace homotopy;

ReferenceFrame::ReferenceFrame( LineSubSegment* p_subseg ) {
  m_name = p_subseg->get_name();
  m_connect_to_cp = false;
  m_segment = Segment2D( p_subseg->m_subseg.source(), p_subseg->m_subseg.target() );
  mp_line_subsegment = p_subseg;
  m_mid_point = Point2D( (p_subseg->m_subseg.source().x()+p_subseg->m_subseg.target().x())/2, (p_subseg->m_subseg.source().y()+p_subseg->m_subseg.target().y())/2 ); 
}

ReferenceFrame::~ReferenceFrame() {
  mp_line_subsegment = NULL;
}

bool ReferenceFrame::is_line_crossed( Point2D pos_a, Point2D pos_b ) {
  Segment2D new_line( pos_a, pos_b );
  //std::cout << "CHECK " << new_line << " with " << m_segment << std::endl;
  CGAL::Object result = CGAL::intersection( new_line, m_segment );
  Point2D ipoint;
  Segment2D iseg; 
 
  if( CGAL::assign( ipoint, result ) ) {
    return true;
  }
  else if( CGAL::assign( iseg, result ) ) {
    return true;
  }
  return false;
}

ReferenceFrameSet::ReferenceFrameSet() {
  _p_world_map = NULL;
  _reference_frames.clear();
  _string_constraint.clear();
}

ReferenceFrameSet::~ReferenceFrameSet() {
  for( std::vector<ReferenceFrame*>::iterator it = _reference_frames.begin();
       it != _reference_frames.end(); it ++ ) {
    ReferenceFrame* p_rf = (*it);
    delete p_rf;
    p_rf = NULL;
  }
  _reference_frames.clear();
  _string_constraint.clear();
}

void ReferenceFrameSet::init(int width, int height, std::vector< std::vector<Point2D> >& obstacles) {
  if( _p_world_map ) {
    delete _p_world_map;
    _p_world_map = NULL;
  }
  _reference_frames.clear();
  
  _p_world_map = new WorldMap(width, height);
  _p_world_map->load_obstacle_info(obstacles); 
  _p_world_map->init();

  for( unsigned int obs_i = 0; obs_i < _p_world_map->get_obstacles().size(); obs_i++ ) {
    Obstacle* p_obstacle = _p_world_map->get_obstacles()[obs_i];
    if ( p_obstacle ) {
      if (p_obstacle->mp_alpha_seg) {
        for( unsigned int a_i = 0; a_i < p_obstacle->mp_alpha_seg->m_subsegs.size(); a_i ++ ) {
          LineSubSegment* p_subseg_a = p_obstacle->mp_alpha_seg->m_subsegs[a_i];
          if (p_subseg_a) {
            ReferenceFrame* p_rf = new ReferenceFrame( p_subseg_a );
            p_rf->m_connect_to_cp = p_subseg_a->m_is_connected_to_central_point; 
            //std::cout << "REF " << p_rf->m_segment << std::endl; 
            _reference_frames.push_back(p_rf);
          }
        }
      }

      if (p_obstacle->mp_beta_seg) {
        for( unsigned int b_i = 0; b_i < p_obstacle->mp_beta_seg->m_subsegs.size(); b_i ++ ) {
          LineSubSegment* p_subseg_b = p_obstacle->mp_beta_seg->m_subsegs[b_i];
          if (p_subseg_b) {
            ReferenceFrame* p_rf = new ReferenceFrame( p_subseg_b );
            p_rf->m_connect_to_cp = p_subseg_b->m_is_connected_to_central_point; 
            //std::cout << "REF " << p_rf->m_segment << std::endl; 
            _reference_frames.push_back(p_rf);
          }
        }
      }
    }
  }
}

ReferenceFrame* ReferenceFrameSet::get_reference_frame( std::string name ) {  
  for( std::vector<ReferenceFrame*>::iterator it = _reference_frames.begin();
       it != _reference_frames.end(); it ++ ) {
    ReferenceFrame* p_rf = (*it);
    if( p_rf->m_name == name ) {
      return p_rf;
    }
  }
  return NULL; 
}

StringGrammar* ReferenceFrameSet::get_string_grammar( int init_x, int init_y, int goal_x, int goal_y ) {
  Point2D init_point( init_x, init_y );
  Point2D goal_point( goal_x, goal_y );
  return get_string_grammar( init_point, goal_point );
}

HomotopicGrammar* ReferenceFrameSet::get_homotopic_grammar( int init_x, int init_y, int goal_x, int goal_y ) {
  Point2D init_point( init_x, init_y );
  Point2D goal_point( goal_x, goal_y );
  return get_homotopic_grammar( init_point, goal_point );
}

StringGrammar* ReferenceFrameSet::get_string_grammar( Point2D init, Point2D goal ) {
  SubRegion* p_init_subregion = NULL;
  SubRegion* p_goal_subregion = NULL;
 
  if( _p_world_map) {
      p_init_subregion = _p_world_map->in_subregion( init );
      p_goal_subregion = _p_world_map->in_subregion( goal ); 
  }
  if( p_init_subregion == NULL || p_goal_subregion == NULL ){
      return NULL;
  }
  return get_string_grammar( p_init_subregion, p_goal_subregion ); 
}

HomotopicGrammar* ReferenceFrameSet::get_homotopic_grammar( Point2D init, Point2D goal ) {
  SubRegion* p_init_subregion = NULL;
  SubRegion* p_goal_subregion = NULL;
 
  if( _p_world_map) {
      p_init_subregion = _p_world_map->in_subregion( init );
      p_goal_subregion = _p_world_map->in_subregion( goal ); 
  }
  if( p_init_subregion == NULL || p_goal_subregion == NULL ){
      return NULL;
  }
  return get_homotopic_grammar( p_init_subregion, p_goal_subregion ); 
}

StringGrammar* ReferenceFrameSet::get_string_grammar( SubRegion* p_init, SubRegion* p_goal ) {
  StringGrammar* p_grammar = NULL;
  if( _p_world_map && p_init && p_goal ) {
    p_grammar = new StringGrammar();  
    //std::cout << "sublinesegment set " << _p_world_map->get_linesubsegment_set().size() << std::endl;

    for( unsigned int i = 0; i < _p_world_map->get_linesubsegment_set().size(); i++ ) {
      LineSubSegmentSet* p_linesubsegment_set = _p_world_map->get_linesubsegment_set()[i];
      if(p_linesubsegment_set) {
        //std::cout << p_linesubsegment_set->m_subsegs.size() << std::endl;
        //std::cout << "LR " << p_linesubsegment_set->get_name() << std::endl;
        for( unsigned int j = 0; j < p_linesubsegment_set->m_subsegs.size(); j++) {
          LineSubSegment* p_linesubsegment = p_linesubsegment_set->m_subsegs[j];
          if (p_linesubsegment) {
            std::string trans_name = p_linesubsegment->get_name();             
            SubRegion* p_region_a = p_linesubsegment->m_neighbors[0];
            SubRegion* p_region_b = p_linesubsegment->m_neighbors[1];
            std::string region_a_name = p_region_a->get_name();
            std::string region_b_name = p_region_b->get_name();
            p_grammar->add_transition( region_a_name, region_b_name, trans_name);
          }
        }
      }
    }
    
    p_grammar->set_init( p_init->get_name() );
    p_grammar->set_goal( p_goal->get_name() );
  }
  return p_grammar;
}

HomotopicGrammar* ReferenceFrameSet::get_homotopic_grammar( SubRegion* p_init, SubRegion* p_goal ) {
  HomotopicGrammar* p_grammar = NULL;
  if( _p_world_map && p_init && p_goal ) {
    p_grammar = new HomotopicGrammar();
     
    for( unsigned int i = 0; i < _p_world_map->get_linesubsegment_set().size(); i++ ) {
      LineSubSegmentSet* p_linesubsegment_set = _p_world_map->get_linesubsegment_set()[i];
      if(p_linesubsegment_set) {
        for( unsigned int j = 0; j < p_linesubsegment_set->m_subsegs.size(); j++) {
          LineSubSegment* p_linesubsegment = p_linesubsegment_set->m_subsegs[j];
          if (p_linesubsegment && p_linesubsegment->m_is_connected_to_central_point ) {
            std::string trans_name = p_linesubsegment->get_name();             
            SubRegion* p_region_a = p_linesubsegment->m_neighbors[0];
            SubRegion* p_region_b = p_linesubsegment->m_neighbors[1];
            std::string region_a_name = "";
            std::string region_b_name = "";
            if ( p_region_a->m_is_connected_to_central_point ) {
              region_a_name = MERGED_CENTER_SUBREGION;
            } else {
              region_a_name = p_region_a->get_name();
            }
            if ( p_region_b->m_is_connected_to_central_point ) {
              region_b_name = MERGED_CENTER_SUBREGION;
            } else {
              region_b_name = p_region_b->get_name();
            }
            p_grammar->add_transition( region_a_name, region_b_name, trans_name);
          }
        }
      }
    }
    p_grammar->set_init( p_init->get_name() );
    p_grammar->set_goal( p_goal->get_name() );
  }
  return p_grammar;
}

std::vector< std::string > ReferenceFrameSet::get_string( Point2D start, Point2D end, grammar_type_t type ) {
  std::vector< std::string > id_string;
  Segment2D line(start, end);
  //std::cout << "LINE " << line << std::endl;
  if (type == STRING_GRAMMAR_TYPE) {
    for( std::vector<ReferenceFrame*>::iterator it = _reference_frames.begin();
         it != _reference_frames.end(); it ++ ) {
      ReferenceFrame* p_rf = (*it);
      // std::cout << "REF " << p_rf->m_segment << std::endl;
      if ( CGAL::do_intersect( p_rf->m_segment, line ) ) {
        id_string.push_back( p_rf->m_name );
      }
    }
  }
  else if (type == HOMOTOPIC_GRAMMAR_TYPE) {
    for( std::vector<ReferenceFrame*>::iterator it = _reference_frames.begin();
         it != _reference_frames.end(); it ++ ) {
      ReferenceFrame* p_rf = (*it);
      // std::cout << "REF " << p_rf->m_segment << std::endl;
      if ( CGAL::do_intersect( p_rf->m_segment, line ) ) {
        if( p_rf->m_connect_to_cp ) {
          id_string.push_back( CENTER_POINT_ID_CHARACTER );
        }
        else {
          id_string.push_back( p_rf->m_name );
        }
      }
    }

  }
  return id_string;
}

std::vector< std::string > ReferenceFrameSet::get_string ( std::vector<Point2D> points, grammar_type_t type ) {
  std::vector< std::string > ids;
  for( unsigned int i = 0; i < points.size()-1; i ++ ) {
    std::vector< std::string > sub_ids = get_string( points[i], points[i+1], type); 
    for( unsigned int j = 0; j < sub_ids.size(); j ++ ) {
      if ( type == STRING_GRAMMAR_TYPE ) {
        ids.push_back( sub_ids[j] ); 
      }
      else if( type == HOMOTOPIC_GRAMMAR_TYPE ) {
      }
    }
  }
  return ids;
}

void ReferenceFrameSet::import_string_constraint( std::vector<Point2D> points, grammar_type_t type ) {
  std::vector< std::string > constraint = get_string( points, type );
  _string_constraint.push_back( constraint );
}

bool ReferenceFrameSet::is_constained_substring( std::vector< std::string > sub_str, bool reverse ) {
  for( unsigned int i = 0; i < _string_constraint.size(); i ++ ) {
    std::vector< std::string > constraint = _string_constraint[i];
    if( true == is_eligible_substring( sub_str, constraint, reverse ) ) {
      return true;
    }
  }
  return false;
}

bool ReferenceFrameSet::is_eligible_substring( std::vector< std::string > sub_str, std::vector< std::string > ref_str, bool reverse ) {
  if ( ref_str.size() < sub_str.size() ) {
    return false;
  }
  if( reverse == false ) {
    for( std::vector< std::string >::iterator it = sub_str.begin(),
                                              itr  = ref_str.begin(); 
                                              it != sub_str.end();
                                              it++, itr++ ) {
      if( (*it) != (*itr) ) {
        return false;
      }
    }
  }
  else {
    std::vector< std::string >::iterator it = sub_str.begin();
    std::vector< std::string >::reverse_iterator itr  = ref_str.rbegin();
    while( it != sub_str.end() ) {
      if( (*it) != (*itr) ) {
        return false;
      }
      it++;
      itr++;
    }
  }
  
  return true;
}
