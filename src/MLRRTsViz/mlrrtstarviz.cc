#include <QtGui>

#include "mlrrtstarviz.h"


using namespace mlrrts;

MLRRTstarViz::MLRRTstarViz( QWidget * parent ) : QLabel(parent) {

  mp_tree = NULL;
}

void MLRRTstarViz::set_tree( MLRRTstar* p_tree ) {
  mp_tree = p_tree;
}



