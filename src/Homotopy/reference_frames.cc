#include <CGAL/intersections.h>
#include "reference_frames.h"

ReferenceFrame::ReferenceFrame() {
  m_name = "";
  m_connect_to_cp = false;
}

ReferenceFrame::~ReferenceFrame() {

}

ReferenceFrameSet::ReferenceFrameSet() {
  _p_world_map = NULL;
  _reference_frames.clear();
}

ReferenceFrameSet::~ReferenceFrameSet() {
  for( std::vector<ReferenceFrame*>::iterator it = _reference_frames.begin();
       it != _reference_frames.end(); it ++ ) {
    ReferenceFrame* p_rf = (*it);
    delete p_rf;
    p_rf = NULL;
  }
  _reference_frames.clear();
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
            ReferenceFrame* p_rf = new ReferenceFrame();
            p_rf->m_name = p_subseg_a->get_name();
            p_rf->m_connect_to_cp = p_subseg_a->m_is_connected_to_central_point; 
            p_rf->m_segment = Segment2D( p_subseg_a->m_subseg.source(), p_subseg_a->m_subseg.target());
            _reference_frames.push_back(p_rf);
          }
        }
      }

      if (p_obstacle->mp_beta_seg) {
        for( unsigned int b_i = 0; b_i < p_obstacle->mp_beta_seg->m_subsegs.size(); b_i ++ ) {
          LineSubSegment* p_subseg_b = p_obstacle->mp_beta_seg->m_subsegs[b_i];
          if (p_subseg_b) {
            ReferenceFrame* p_rf = new ReferenceFrame();
            p_rf->m_name = p_subseg_b->get_name();
            p_rf->m_connect_to_cp = p_subseg_b->m_is_connected_to_central_point; 
            p_rf->m_segment = Segment2D( p_subseg_b->m_subseg.source(), p_subseg_b->m_subseg.target());
            _reference_frames.push_back(p_rf);
          }
        }
      }
    }
  }
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
    //std::cout << "sublinesegment set " << _p_world_map->get_sublinesegment_set().size() << std::endl;

    for( unsigned int i = 0; i < _p_world_map->get_sublinesegment_set().size(); i++ ) {
      LineSubSegmentSet* p_linesubsegment_set = _p_world_map->get_sublinesegment_set()[i];
      if(p_linesubsegment_set) {
        //std::cout << p_linesubsegment_set->m_subsegs.size() << std::endl;
        //std::cout << "LR " << p_linesubsegment_set->get_name() << std::endl;
        for( unsigned int j = 0; j < p_linesubsegment_set->m_subsegs.size(); j++) {
          LineSubSegment* p_linesubsegment = p_linesubsegment_set->m_subsegs[j];
          if (p_linesubsegment) {
            p_grammar->add_transition( p_linesubsegment->m_neighbors[0]->get_name(),
                                       p_linesubsegment->m_neighbors[1]->get_name(),
                                       p_linesubsegment->get_name() );
            p_grammar->set_init( p_init->get_name() );
            p_grammar->set_goal( p_goal->get_name() );
          }
        }
      }
    }
  }
  return p_grammar;
}

HomotopicGrammar* ReferenceFrameSet::get_homotopic_grammar( SubRegion* p_init, SubRegion* p_goal ) {
  HomotopicGrammar* p_grammar = NULL;
  if( _p_world_map && p_init && p_goal ) {
    p_grammar = new HomotopicGrammar();
     
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
