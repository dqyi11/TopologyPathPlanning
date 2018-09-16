#include <CGAL/intersections.h>
#include "topologyPathPlanning/homotopy/ReferenceFrames.hpp"

#define MERGED_CENTER_SUBREGION "SC"

namespace topologyPathPlanning {

namespace homotopy {

ReferenceFrame::ReferenceFrame( LineSubSegment* p_subseg ) {
  mName = p_subseg->getName();
  mConnectToCp = false;
  mSegment = Segment2D( p_subseg->mSubseg.source(), p_subseg->mSubseg.target() );
  mpLineSubsegment = p_subseg;
  mMidPoint = Point2D( (p_subseg->mSubseg.source().x()+p_subseg->mSubseg.target().x())/2, (p_subseg->mSubseg.source().y()+p_subseg->mSubseg.target().y())/2 ); 
}

ReferenceFrame::~ReferenceFrame() {
  mpLineSubsegment = NULL;
}

bool ReferenceFrame::isLineCrossed( Point2D pos_a, Point2D pos_b ) {
  Segment2D new_line( pos_a, pos_b );
  //std::cout << "CHECK " << new_line << " with " << m_segment << std::endl;
  CGAL::Object result = CGAL::intersection( new_line, mSegment );
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
  mpWorldMap = NULL;
  mReferenceFrames.clear();
  mStringConstraint.clear();
}

ReferenceFrameSet::~ReferenceFrameSet() {
  for( std::vector<ReferenceFrame*>::iterator it = mReferenceFrames.begin();
       it != mReferenceFrames.end(); it ++ ) {
    ReferenceFrame* p_rf = (*it);
    delete p_rf;
    p_rf = NULL;
  }
  mReferenceFrames.clear();
  mStringConstraint.clear();
}

void ReferenceFrameSet::init(int width, int height, std::vector< std::vector<Point2D> >& obstacles) {
  if( mpWorldMap ) {
    delete mpWorldMap;
    mpWorldMap = NULL;
  }
  mReferenceFrames.clear();
  
  mpWorldMap = new WorldMap(width, height);
  mpWorldMap->loadObstacleInfo(obstacles); 
  mpWorldMap->init();

  for( unsigned int obs_i = 0; obs_i < mpWorldMap->getObstacles().size(); obs_i++ ) {
    Obstacle* p_obstacle = mpWorldMap->getObstacles()[obs_i];
    if ( p_obstacle ) {
      if (p_obstacle->mpAlphaSeg) {
        for( unsigned int a_i = 0; a_i < p_obstacle->mpAlphaSeg->mSubsegs.size(); a_i ++ ) {
          LineSubSegment* p_subseg_a = p_obstacle->mpAlphaSeg->mSubsegs[a_i];
          if (p_subseg_a) {
            ReferenceFrame* p_rf = new ReferenceFrame( p_subseg_a );
            p_rf->mConnectToCp = p_subseg_a->mIsConnectedToCentralPoint; 
            //std::cout << "REF " << p_rf->m_segment << std::endl; 
            mReferenceFrames.push_back(p_rf);
          }
        }
      }

      if (p_obstacle->mpBetaSeg) {
        for( unsigned int b_i = 0; b_i < p_obstacle->mpBetaSeg->mSubsegs.size(); b_i ++ ) {
          LineSubSegment* p_subseg_b = p_obstacle->mpBetaSeg->mSubsegs[b_i];
          if (p_subseg_b) {
            ReferenceFrame* p_rf = new ReferenceFrame( p_subseg_b );
            p_rf->mConnectToCp = p_subseg_b->mIsConnectedToCentralPoint; 
            //std::cout << "REF " << p_rf->m_segment << std::endl; 
            mReferenceFrames.push_back(p_rf);
          }
        }
      }
    }
  }
}

ReferenceFrame* ReferenceFrameSet::getReferenceFrame( std::string name ) {  
  for( std::vector<ReferenceFrame*>::iterator it = mReferenceFrames.begin();
       it != mReferenceFrames.end(); it ++ ) {
    ReferenceFrame* p_rf = (*it);
    if( p_rf->mName == name ) {
      return p_rf;
    }
  }
  return NULL; 
}

StringGrammar* ReferenceFrameSet::getStringGrammar( int init_x, int init_y, int goal_x, int goal_y ) {
  Point2D init_point( init_x, init_y );
  Point2D goal_point( goal_x, goal_y );
  return getStringGrammar( init_point, goal_point );
}

HomotopicGrammar* ReferenceFrameSet::getHomotopicGrammar( int init_x, int init_y, int goal_x, int goal_y ) {
  Point2D init_point( init_x, init_y );
  Point2D goal_point( goal_x, goal_y );
  return getHomotopicGrammar( init_point, goal_point );
}

StringGrammar* ReferenceFrameSet::getStringGrammar( Point2D init, Point2D goal ) {
  SubRegion* p_init_subregion = NULL;
  SubRegion* p_goal_subregion = NULL;
 
  if( mpWorldMap) {
      p_init_subregion = mpWorldMap->inSubregion( init );
      p_goal_subregion = mpWorldMap->inSubregion( goal ); 
  }
  if( p_init_subregion == NULL || p_goal_subregion == NULL ){
      return NULL;
  }
  return getStringGrammar( p_init_subregion, p_goal_subregion ); 
}

HomotopicGrammar* ReferenceFrameSet::getHomotopicGrammar( Point2D init, Point2D goal ) {
  SubRegion* p_init_subregion = NULL;
  SubRegion* p_goal_subregion = NULL;
 
  if( mpWorldMap) {
      p_init_subregion = mpWorldMap->inSubregion( init );
      p_goal_subregion = mpWorldMap->inSubregion( goal ); 
  }
  if( p_init_subregion == NULL || p_goal_subregion == NULL ){
      return NULL;
  }
  return getHomotopicGrammar( p_init_subregion, p_goal_subregion ); 
}

StringGrammar* ReferenceFrameSet::getStringGrammar( SubRegion* p_init, SubRegion* p_goal ) {
  StringGrammar* p_grammar = NULL;
  if( mpWorldMap && p_init && p_goal ) {
    p_grammar = new StringGrammar();  
    //std::cout << "sublinesegment set " << _p_world_map->get_linesubsegment_set().size() << std::endl;

    for( unsigned int i = 0; i < mpWorldMap->getLinesubsegmentSet().size(); i++ ) {
      LineSubSegmentSet* p_linesubsegment_set = mpWorldMap->getLinesubsegmentSet()[i];
      if(p_linesubsegment_set) {
        //std::cout << p_linesubsegment_set->m_subsegs.size() << std::endl;
        //std::cout << "LR " << p_linesubsegment_set->get_name() << std::endl;
        for( unsigned int j = 0; j < p_linesubsegment_set->mSubsegs.size(); j++) {
          LineSubSegment* p_linesubsegment = p_linesubsegment_set->mSubsegs[j];
          if (p_linesubsegment) {
            std::string trans_name = p_linesubsegment->getName();             
            SubRegion* p_region_a = p_linesubsegment->mNeighbors[0];
            SubRegion* p_region_b = p_linesubsegment->mNeighbors[1];
            std::string region_a_name = p_region_a->getName();
            std::string region_b_name = p_region_b->getName();
            p_grammar->addTransition( region_a_name, region_b_name, trans_name);
          }
        }
      }
    }
    
    p_grammar->setInit( p_init->getName() );
    p_grammar->setGoal( p_goal->getName() );
  }
  return p_grammar;
}

HomotopicGrammar* ReferenceFrameSet::getHomotopicGrammar( SubRegion* p_init, SubRegion* p_goal ) {
  HomotopicGrammar* p_grammar = NULL;
  if( mpWorldMap && p_init && p_goal ) {
    p_grammar = new HomotopicGrammar();
     
    for( unsigned int i = 0; i < mpWorldMap->getLinesubsegmentSet().size(); i++ ) {
      LineSubSegmentSet* p_linesubsegment_set = mpWorldMap->getLinesubsegmentSet()[i];
      if(p_linesubsegment_set) {
        for( unsigned int j = 0; j < p_linesubsegment_set->mSubsegs.size(); j++) {
          LineSubSegment* p_linesubsegment = p_linesubsegment_set->mSubsegs[j];
          if (p_linesubsegment && p_linesubsegment->mIsConnectedToCentralPoint ) {
            std::string trans_name = p_linesubsegment->getName();             
            SubRegion* p_region_a = p_linesubsegment->mNeighbors[0];
            SubRegion* p_region_b = p_linesubsegment->mNeighbors[1];
            std::string region_a_name = "";
            std::string region_b_name = "";
            if ( p_region_a->mIsConnectedToCentralPoint ) {
              region_a_name = MERGED_CENTER_SUBREGION;
            } else {
              region_a_name = p_region_a->getName();
            }
            if ( p_region_b->mIsConnectedToCentralPoint ) {
              region_b_name = MERGED_CENTER_SUBREGION;
            } else {
              region_b_name = p_region_b->getName();
            }
            p_grammar->addTransition( region_a_name, region_b_name, trans_name);
          }
        }
      }
    }
    p_grammar->setInit( p_init->getName() );
    p_grammar->setGoal( p_goal->getName() );
  }
  return p_grammar;
}

std::vector< std::string > ReferenceFrameSet::getString( Point2D start, Point2D end, grammar_type_t type ) {
  std::vector< std::string > id_string;
  Segment2D line(start, end);
  //std::cout << "LINE " << line << std::endl;
  if (type == STRING_GRAMMAR_TYPE) {
    for( std::vector<ReferenceFrame*>::iterator it = mReferenceFrames.begin();
         it != mReferenceFrames.end(); it ++ ) {
      ReferenceFrame* p_rf = (*it);
      // std::cout << "REF " << p_rf->m_segment << std::endl;
      if ( CGAL::do_intersect( p_rf->mSegment, line ) ) {
        id_string.push_back( p_rf->mName );
      }
    }
  }
  else if (type == HOMOTOPIC_GRAMMAR_TYPE) {
    for( std::vector<ReferenceFrame*>::iterator it = mReferenceFrames.begin();
         it != mReferenceFrames.end(); it ++ ) {
      ReferenceFrame* p_rf = (*it);
      // std::cout << "REF " << p_rf->m_segment << std::endl;
      if ( CGAL::do_intersect( p_rf->mSegment, line ) ) {
        if( p_rf->mConnectToCp ) {
          id_string.push_back( CENTER_POINT_ID_CHARACTER );
        }
        else {
          id_string.push_back( p_rf->mName );
        }
      }
    }

  }
  return id_string;
}

std::vector< std::string > ReferenceFrameSet::getString ( std::vector<Point2D> points, grammar_type_t type ) {
  std::vector< std::string > ids;
  for( unsigned int i = 0; i < points.size()-1; i ++ ) {
    std::vector< std::string > sub_ids = getString( points[i], points[i+1], type); 
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

std::vector< std::string > ReferenceFrameSet::getString ( PointSequence& path,  grammar_type_t type )
{
  return getString(path.mPoints, type);
}

void ReferenceFrameSet::importStringConstraint( std::vector<Point2D> points, grammar_type_t type ) {
  std::vector< std::string > constraint = getString( points, type );
  mStringConstraint.push_back( constraint );
}

bool ReferenceFrameSet::isConstainedSubstring( std::vector< std::string > sub_str, bool reverse ) {
  for( unsigned int i = 0; i < mStringConstraint.size(); i ++ ) {
    std::vector< std::string > constraint = mStringConstraint[i];
    if( true == isEligibleSubstring( sub_str, constraint, reverse ) ) {
      return true;
    }
  }
  return false;
}

bool ReferenceFrameSet::isEligibleSubstring( std::vector< std::string > sub_str, std::vector< std::string > ref_str, bool reverse ) {
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

} // homotopy

} // topologyPathPlanning
