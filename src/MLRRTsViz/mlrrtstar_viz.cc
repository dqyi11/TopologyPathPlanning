#include <QtGui>

#include "mlrrtstar_viz.h"


using namespace mlrrts;

MLRRTstarViz::MLRRTstarViz( QWidget * parent ) : QLabel(parent) {

  mp_tree = NULL;
}

void MLRRTstarViz::set_tree( MLRRTstar* p_tree ) {
  mp_tree = p_tree;
}


void MLRRTstarViz::paint( QPaintDevice* device ) {

}

void MLRRTstarViz::paintEvent( QPaintEvent* e ) {
  QLabel::paintEvent(e);
  paint( this );
}
