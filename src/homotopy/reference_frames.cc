#include "reference_frames.h"

ReferenceFrame::ReferenceFrame() {
}

ReferenceFrame::~ReferenceFrame() {

}

ReferenceFrameSet::ReferenceFrameSet( WorldMap* p_world_map ) {
  _p_world_map = p_world_map;
  _reference_frames.clear();
  for( std::vector<LineSubSegmentSet*>::iterator it = _p_world_map->get_sublinesegment_set().begin();
       it != _p_world_map->get_sublinesegment_set().end(); it ++ ) {
    LineSubSegmentSet* p_linesubsegment_set = (*it);
    for( std::vector<LineSubSegment*>::iterator its = p_linesubsegment_set->m_subsegs.begin();
         its != p_linesubsegment_set->m_subsegs.end(); its ++ ) {
      LineSubSegment* p_linesubsegment = (*its);
      ReferenceFrame* p_rf = new ReferenceFrame();
      p_rf->m_name = p_linesubsegment->get_name();
      p_rf->m_segment = p_linesubsegment->m_subseg;
      _reference_frames.push_back(p_rf);
    }
  }  
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
