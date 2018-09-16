#include "topologyPathPlanning/spatialinfer/SpatialRelationMgr.hpp"

using namespace std;
using namespace topologyPathPlanning::homotopy;

namespace topologyPathPlanning {

namespace topologyinference {

StringClass::StringClass( vector< std::string > string ) {
  mString = string;
}

StringClass::~StringClass() {
  mpReferenceFrames.clear();
}
  
string StringClass::getName() {
  string name = "";
  for( unsigned int i=0; i<mString.size(); i++ ) {
    if( i>0 ) {
      name += " ";
    }
    name += mString[i];
  }
  return name;
}

void StringClass::init( ReferenceFrameSet* p_rfs ) {
  if( p_rfs ) {
    for( unsigned int i=0; i < mString.size(); i++ ) {
      string id = mString[i];
      ReferenceFrame* p_rf = p_rfs->getReferenceFrame( id );
      if( p_rf ) {
        mpReferenceFrames.push_back( p_rf );
      }
    }
  } 
}

bool contains( vector< string > string_set, string string_item ) {
  for( vector< string >::iterator it = string_set.begin();
       it != string_set.end(); it ++ ) {
    string current_string_item = (*it);
    if( current_string_item == string_item ) {
      return true;
    }
  }

  return false;
}

SpatialRelationMgr::SpatialRelationMgr( WorldMap* p_worldmap ) {
  mpWorldmap = p_worldmap;
  mpFunctions.clear();
  mpStringClasses.clear();
  mRules.clear();
  mStartX = -1;
  mStartY = -1;
  mGoalX = -1;
  mGoalY = -1;
}

SpatialRelationMgr::~SpatialRelationMgr() {
  mpWorldmap = NULL;
  mpFunctions.clear();
  for( vector<StringClass*>::iterator it = mpStringClasses.begin();
       it != mpStringClasses.end(); it++ ) {
    StringClass* p_str_cls = (*it);
    delete p_str_cls;
    p_str_cls = NULL;
  }
  mpStringClasses.clear();
  mRules.clear();
}

vector< pair<ReferenceFrame*, bool> > SpatialRelationMgr::getRules( ReferenceFrameSet* p_reference_frame_set ) {
  vector< pair<ReferenceFrame*, bool> > rules;
  for( unsigned int i=0; i < mpFunctions.size(); i++ ) {
    cout << "FUNC: " << mpFunctions[i]->getName() << endl;
    vector< pair<ReferenceFrame*, bool> > rfs = mpFunctions[i]->getRules( p_reference_frame_set );
    printRules( rfs );
    for( unsigned int j=0; j < rfs.size(); j ++ ) {
      rules.push_back( rfs[j] );
    } 
  }
  return rules;
}

void SpatialRelationMgr::printRules( vector< pair< ReferenceFrame*, bool > > rules ) {

  cout << "RULES" << endl;
  for( unsigned int i=0; i<mRules.size(); i++ ) {
    pair<ReferenceFrame*, bool> rule = mRules[i];
    cout << rule.first->getName() << " " << rule.second << endl;
  }
}

vector< string > SpatialRelationMgr::getSpatialRelationFunctionNames() {
  vector< string > names;
  for(unsigned int i=0; i < mpFunctions.size(); i++) {
    SpatialRelationFunction* p_func = mpFunctions[i];
    if( p_func ) {
      names.push_back( p_func->getName() );
    }
  }
  return names;
}

bool SpatialRelationMgr::hasSpatialRelationFunction( string name ) {
  for(unsigned int i=0; i < mpFunctions.size(); i++) {
    SpatialRelationFunction* p_func = mpFunctions[i];
    if( p_func ) {
      if( p_func->getName() == name ) {
        return true;
      }
    }
  }
  return false;
}

void SpatialRelationMgr::removeSpatialRelationFunction( string name ) {

  for( vector<SpatialRelationFunction*>::iterator it = mpFunctions.begin();
       it != mpFunctions.end(); /* it ++ */ ) { 
    SpatialRelationFunction* p_func = (*it);
    if( p_func && ( p_func->getName() == name ) ) {
      mpFunctions.erase( it );
    }
    else {
      ++ it;
    }
  }
}

void SpatialRelationMgr::getStringClasses( ReferenceFrameSet* p_rfs  ) {
  mpStringClasses.clear();
  mRules.clear();
  vector< vector< string > > string_set;
  
  if( p_rfs ) {
    StringGrammar* p_grammar = p_rfs->getStringGrammar( mStartX, mStartY, mGoalX, mGoalY );
    if( p_grammar ){
      vector< vector< string > > all_simple_strings = p_grammar->findSimpleStrings(); 
      mRules = getRules( p_rfs );
      printRules( mRules );
      string_set = filter( all_simple_strings, mRules );
        
      delete p_grammar;
      p_grammar=NULL;
    }   
  }
  for( vector< vector< string > >::iterator it = string_set.begin();
       it != string_set.end(); it ++ ) {
    vector< string > item = (*it);
    StringClass* p_class = new StringClass( item );
    p_class->init( p_rfs );
    mpStringClasses.push_back( p_class );
  }

}


vector< vector< string > > SpatialRelationMgr::filter( vector< vector< string > > string_set, vector< pair< homotopy::ReferenceFrame*, bool > > rules ) {
  vector< vector < string > > output_set;
  for( vector< vector< string > >::iterator it = string_set.begin();
       it != string_set.end(); it++ ) {
    vector< string > item = (*it);
    if( isEligible( item, rules ) ) {
      output_set.push_back( item );
    }
  }
  return output_set;
}


bool SpatialRelationMgr::isEligible( vector< string > string_item, vector< pair< ReferenceFrame*, bool > > rules ) {
  for( vector< pair< ReferenceFrame*, bool > >::iterator it = rules.begin();
       it != rules.end(); it ++ ) {
    pair< ReferenceFrame*, bool> rule = (*it);
    if( false == isEligible( string_item, rule ) ) {
      return false;
    }
  }
  return true;
}

bool SpatialRelationMgr::isEligible( vector< string > string_item, pair< ReferenceFrame*, bool > rule ) {
  bool positive = rule.second;
  ReferenceFrame* p_rf = rule.first;
  if( contains( string_item, p_rf->getName() ) == positive ) { 
    return true; 
  }
  return false;
}

} // topologyinference

} // topologyPathPlanning
