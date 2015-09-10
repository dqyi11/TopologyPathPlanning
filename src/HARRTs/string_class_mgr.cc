#include "string_class_mgr.h"

StringClassMgr::StringClassMgr( StringGrammar* p_grammar ) {
  _p_grammar = p_grammar;
}

StringClassMgr::~StringClassMgr() {
  _p_grammar = NULL;
  _classes.clear();
}

void StringClassMgr::import_path( Path* p_path ) {

}


