#include "reference_frames.h"

ReferenceFrames::ReferenceFrames( WorldMap* p_world_map ) {
    _p_world_map = p_world_map;
}

ReferenceFrames::~ReferenceFrames() {

}

StringGrammar* ReferenceFrames::get_string_grammar( SubRegion* p_init, SubRegion* p_goal ) {
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

HomotopicGrammar* ReferenceFrames::get_homotopic_grammar( SubRegion* p_init, SubRegion* p_goal ) {
  HomotopicGrammar* p_grammar = NULL;
  if( _p_world_map ) {
    p_grammar = new HomotopicGrammar();
    
  }
  return p_grammar;
}

std::string ReferenceFrames::get_character_id( Point2D start, Point2D end, grammar_type_t type ) {
   std::string character_id = "";
   Segment2D line(start, end);
   if (type == STRING_GRAMMAR_TYPE) {
     for( std::vector<LineSubSegmentSet*>::iterator it = _p_world_map->get_sublinesegment_set().begin();
          it != _p_world_map->get_sublinesegment_set().end(); it ++ ) {
       LineSubSegmentSet* p_linesubsegment_set = (*it);
       for( std::vector<LineSubSegment*>::iterator itl = p_linesubsegment_set->m_subsegs.begin();
            itl != p_linesubsegment_set->m_subsegs.end(); itl ++ ) {
          LineSubSegment* p_linesubsegment = (*itl);
          CGAL::Object result = intersection( p_linesubsegment->m_subseg, line );
          Point2D po;
          if ( CGAL::assign( po, result ) ) {
            return p_linesubsegment->get_name();
          }
       }
     }
   }
   else if (type == HOMOTOPIC_GRAMMAR_TYPE) {

   }        
   return character_id;
}
