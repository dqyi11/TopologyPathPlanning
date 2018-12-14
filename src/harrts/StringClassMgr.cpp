#include <limits>
#include "topologyPathPlanning/harrts/BiRRTstar.hpp"
#include "topologyPathPlanning/harrts/StringClassMgr.hpp"

using namespace topologyPathPlanning::homotopy;

namespace topologyPathPlanning {

namespace harrts {

StringClass::StringClass( std::vector< std::string > string, unsigned int created_iteration_num ) {
  mString = string;
  mCost = std::numeric_limits<float>::max();
  mpPath = NULL;
  mCreatedIterationNum = created_iteration_num;
}

StringClass::~StringClass() {
  mString.clear();
  mHistoricalData.clear();
  mpPath = NULL;
}

std::string StringClass::getName() {
  std::string name = "";
  for( unsigned int i = 0; i < mString.size(); i ++ ) {
    if (i > 0) { 
      name += " ";
    }
    name += mString[i];
  }
  return name;
}

void StringClass::dump_historical_data( std::string filename ) {
  std::ofstream hist_data_file;
  hist_data_file.open(filename.c_str());
  hist_data_file << getName() << std::endl;
  write_historical_data( hist_data_file );
  hist_data_file.close();
}

void StringClass::write_historical_data( std::ostream& out ) {
  /*
  for(unsigned int i=0;i<m_created_iteration_num;i++) {
    out << std::numeric_limits<float>::max() << " ";
  }*/
  //out << std::endl;
  for(std::vector<double>::iterator it = mHistoricalData.begin();
      it != mHistoricalData.end(); it++ ) {
    double data = (*it);
    out << data << " ";
  }
  out << std::endl;
  for(int i=mCreatedIterationNum;
      i<(signed)(mHistoricalData.size()+mCreatedIterationNum);i++) {
    out << i << " ";
  }
  out << std::endl;
}

void StringClass::record() {

  mHistoricalData.push_back(mCost);
}

StringClassMgr::StringClassMgr( StringGrammar* p_grammar ) {
  mpGrammar = p_grammar;
}

StringClassMgr::~StringClassMgr() {
  mpGrammar = NULL;
  mClasses.clear();
}

void StringClassMgr::importPath( Path* p_path, unsigned int iteration_num ) { 
  std::vector< std::string > non_repeating_id_string = mpGrammar->getNonRepeatingForm( p_path->mString );
  if ( mpGrammar->isValidString( non_repeating_id_string ) == false ) {
    std::cout << "INVALID STRING " << std::endl;
  }
  StringClass* p_string_class = findStringClass( non_repeating_id_string );
  if( p_string_class ) {
    if( p_string_class->mCost > p_path->mCost ) {
      p_string_class->mCost = p_path->mCost;
      p_string_class->mpPath = p_path;      
    }
  }
  else {
    p_string_class = new StringClass( non_repeating_id_string, iteration_num ); 
    p_string_class->mCost = p_path->mCost;
    p_string_class->mpPath = p_path;
    mClasses.push_back(p_string_class);
  }
}

std::vector<Path*> StringClassMgr::exportPaths() {

  std::vector<Path*> paths;
  for( unsigned int i = 0; i < mClasses.size(); i++) {
    paths.push_back( mClasses[i]->mpPath );
  }
  return paths;
}

StringClass* StringClassMgr::findStringClass( std::vector< std::string > str ) {
  
  StringClass* p_string_class = NULL;
  for( unsigned int i = 0; i < mClasses.size(); i ++ ) {
    if( mClasses[i]->mString.size() == str.size()) {
      bool identical = true;
      for( unsigned int j = 0; j < mClasses[i]->mString.size(); j ++) {
        if( mClasses[i]->mString[j] != str[j] ) {
          identical = false;
          break;
        }
      }
      if( identical ) {
        return mClasses[i];
      }
    }
  }
  return p_string_class;
}

void StringClassMgr::merge() {
  std::vector< StringClass* > merged_classes;
  //std::cout << "NUM OF CLASSES " << _classes.size() << std::endl;
  for( unsigned int i = 0; i < mClasses.size(); i ++ ) {
    StringClass* str_class = mClasses[i];
    if( merged_classes.size() == 0 ) {
      merged_classes.push_back( str_class );
    }
    else {
      //std::cout << "MERGE CLASS SIZE " << merged_classes.size() << std::endl;
      bool found_equivalence = false;
      for( unsigned int j = 0; j < merged_classes.size(); j++) {
        StringClass* str_class_in_mer = merged_classes[j];
        //std::cout << "COMPARE [" << str_class->get_name() << "] and [" << str_class_in_mer->get_name() << "]" << std::endl;
        if( mpGrammar->isEquivalent( str_class_in_mer->mString, str_class->mString ) ) {
          if( str_class_in_mer->mString.size() < str_class->mString.size() ) {
            str_class_in_mer->mString = str_class->mString;
          }
          if ( str_class_in_mer->mCost > str_class->mCost ) {
            str_class_in_mer->mCost = str_class->mCost;
            str_class_in_mer->mpPath = str_class->mpPath;
          }
          found_equivalence = true;
          break;
        }
      }
      if( found_equivalence == false) {
          merged_classes.push_back( str_class );
      }
    }
  }

  mClasses = merged_classes;
}
  
void StringClassMgr::exportGrammar( std::string filename ) {
  if( mpGrammar ) {
    mpGrammar->output( filename );
  }
}

void StringClassMgr::dumpHistoricalData( std::string filename ) {
  std::ofstream hist_data_file;
  hist_data_file.open(filename.c_str());
  for(std::vector< StringClass* >::iterator it = mClasses.begin();
      it != mClasses.end(); it++ ) {
    StringClass* p_str_cls = (*it);
    if(p_str_cls) {
      hist_data_file << p_str_cls->getName() << " : ";
      p_str_cls->write_historical_data( hist_data_file );
    }
  }
  hist_data_file.close();
}

void StringClassMgr::record() {

  for(std::vector< StringClass* >::iterator it = mClasses.begin();
      it != mClasses.end(); it++ ) {
    StringClass* p_str_cls = (*it);
    if(p_str_cls) {
      p_str_cls->record();
    }
  }
}

} // harrts

} // topologyPathPlanning
