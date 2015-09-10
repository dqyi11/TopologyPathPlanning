#include "reference_frames.h"

ReferenceFrame::ReferenceFrame() {
  m_name = "";
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
  _p_world_map = new WorldMap(width, height);
  _p_world_map->load_obstacle_info(obstacles); 
  _p_world_map->init();
  
  _reference_frames.clear();

  for( unsigned int obs_i = 0; obs_i < _p_world_map->get_obstacles().size(); obs_i++ ) {
    Obstacle* p_obstacle = _p_world_map->get_obstacles()[obs_i];
    if ( p_obstacle ) {
      if (p_obstacle->mp_alpha_seg) {
        for( unsigned int a_i = 0; a_i < p_obstacle->mp_alpha_seg->m_subsegs.size(); a_i ++ ) {
          LineSubSegment* p_subseg_a = p_obstacle->mp_alpha_seg->m_subsegs[a_i];
          if (p_subseg_a) {
            ReferenceFrame* p_rf = new ReferenceFrame();
            p_rf->m_name = p_subseg_a->get_name();
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

  return get_string_grammar( p_init_subregion, p_goal_subregion ); 
}

HomotopicGrammar* ReferenceFrameSet::get_homotopic_grammar( Point2D init, Point2D goal ) {
  SubRegion* p_init_subregion = NULL;
  SubRegion* p_goal_subregion = NULL;

  return get_homotopic_grammar( p_init_subregion, p_goal_subregion ); 
}

StringGrammar* ReferenceFrameSet::get_string_grammar( SubRegion* p_init, SubRegion* p_goal ) {
  StringGrammar* p_grammar = NULL;
  if( _p_world_map || p_init || p_goal ) {
    p_grammar = new StringGrammar();  
    for( std::vector<LineSubSegmentSet*>::iterator it = _p_world_map->get_sublinesegment_set().begin();
        it != _p_world_map->get_sublinesegment_set().end(); it ++ ) {
      LineSubSegmentSet* p_linesubsegment_set = (*it);
      for( std::vector<LineSubSegment*>::iterator its = p_linesubsegment_set->m_subsegs.begin();
           its != p_linesubsegment_set->m_subsegs.end(); its ++ ) {
        LineSubSegment* p_linesubsegment = (*its);
        p_grammar->add_transition( p_linesubsegment->m_neighbors[0]->get_name(),
                                   p_linesubsegment->m_neighbors[1]->get_name(),
                                   p_linesubsegment->get_name() );
        p_grammar->set_init( p_init->get_name() );
        p_grammar->set_goal( p_goal->get_name() );   
      }
    }
  }
  return p_grammar;
}

HomotopicGrammar* ReferenceFrameSet::get_homotopic_grammar( SubRegion* p_init, SubRegion* p_goal ) {
  HomotopicGrammar* p_grammar = NULL;
  if( _p_world_map ) {
    p_grammar = new HomotopicGrammar();
     
  }
  return p_grammar;
}

std::string ReferenceFrameSet::get_character_id( Point2D start, Point2D end, grammar_type_t type ) {
  std::string character_id = "";
  Segment2D line(start, end);
  if (type == STRING_GRAMMAR_TYPE) {
    for( std::vector<ReferenceFrame*>::iterator it = _reference_frames.begin();
         it != _reference_frames.end(); it ++ ) {
      ReferenceFrame* p_rf = (*it);
      CGAL::Object result = intersection( p_rf->m_segment, line );
      Point2D po;
      if ( CGAL::assign( po, result ) ) {
        return p_rf->m_name;
      }
    }
  }
  else if (type == HOMOTOPIC_GRAMMAR_TYPE) {

  }        
  return character_id;
}

std::vector< std::string > ReferenceFrameSet::get_string( Point2D start, Point2D end, grammar_type_t type ) {
  std::vector< std::string > id_string;
 
  float x1 = CGAL::to_double( start.x() );
  float y1 = CGAL::to_double( start.y() );
  float x2 = CGAL::to_double( end.x() );
  float y2 = CGAL::to_double( end.y() );

  const bool steep = (fabs(y2 - y1) > fabs(x2 - x1));
  if (steep) {
    std::swap(x1, y1);
    std::swap(x2, y2);
  }

  if (x1 > x2) {
    std::swap(x1, x2);
    std::swap(y1, y2);
  }

  const float dx = x2 - x1;
  const float dy = fabs(y2 - y1);

  float error = dx / 2.0f;
  const int ystep = (y1 < y2) ? 1 : -1;
  int y = (int)y1;

  const int maxX = (int)x2;

  Point2D lastPos(x1, y1);
  Point2D newPos(x1, y1);
  std::string lastString = "";
  for(int x=(int)x1; x<maxX; x++) {
    if(steep) {
      newPos = Point2D( x, y );
      std::string ref_str = get_character_id(lastPos, newPos, type);
      if (ref_str!="") {
        if ( ref_str != id_string[id_string.size()-1] ) {
          id_string.push_back(ref_str);
        }
      }
      lastPos = newPos;
    }
    else {
      newPos = Point2D( x, y );
      std::string ref_str = get_character_id(lastPos, newPos, type);
      if (ref_str!="") {
        if ( ref_str != id_string[id_string.size()-1] ) {
          id_string.push_back(ref_str);
        }
      }
      lastPos = newPos;
    }

    error -= dy;
    if(error < 0) {
      y += ystep;
      error += dx;
    }
  }

  return id_string;
}


std::vector< std::string > ReferenceFrameSet::get_string ( std::vector<Point2D> points, grammar_type_t type ) {
  std::vector< std::string > ids;
  std::string last_string = "";
  for( unsigned int i = 0; i < points.size()-1; i ++ ) {
    if ( ids.size() > 0 ) {
      last_string = ids.back();
    }
    std::vector< std::string > sub_ids = get_string( points[i], points[i+1], type); 
    for( unsigned int j = 0; j < sub_ids.size(); j ++ ) {
      if( last_string != sub_ids[j] ) {
        ids.push_back( sub_ids[j] );
      }
    } 
  }
  return ids;
}
