#include <QtGui>

#include "topologyPathPlanning/harrtsviz/BiRRTstarViz.hpp"

#define START_TREE_COLOR        QColor(160,160,0)
#define START_TREE_COLOR_ALPHA  QColor(160,160,0,100)
#define GOAL_TREE_COLOR         QColor(0,160,160)
#define GOAL_TREE_COLOR_ALPHA   QColor(0,160,160,100)
#define START_COLOR             QColor(255,0,0)
#define GOAL_COLOR              QColor(0,0,255)
#define REFERENCE_FRAME_COLOR   QColor(0,255,0)
#define PATH_COLOR              QColor(255,153,21)
#define DRAWING_LINE_COLOR      QColor(153,76,0)
#define LINE_WIDTH              2

using namespace topologyPathPlanning::homotopy;

namespace topologyPathPlanning {

namespace harrts {

BIRRTstarViz::BIRRTstarViz( QWidget *parent ) :
    QLabel(parent) {
  mpTree = NULL;
  mShowReferenceFrames = false;
  mShowRegions = false;
  mFinishedPlanning = false;
  mReferenceFrameIndex = -1;
  mFoundPathIndex = -1;
  mRegionIndex = -1;
  mSubregionIndex = -1;
  mpReferenceFrames = NULL;
  mTreeShowType = BOTH_TREES_SHOW;
  mShowPoints = false;
  mColors.clear();
}

void BIRRTstarViz::setTree( BIRRTstar* p_tree ) {
  mpTree = p_tree;
}

void BIRRTstarViz::setReferenceFrameSet(ReferenceFrameSet* p_rf) {
  mpReferenceFrames = p_rf;
  for( unsigned int i = 0; i < mpReferenceFrames->getWorldMap()->getSubregionSet().size(); i++) {
    mColors.push_back( QColor( rand()%255, rand()%255, rand()%255 ) );
  }
}

void BIRRTstarViz::paintEvent( QPaintEvent * e ) {
  QLabel::paintEvent(e);

  paint(this);
}


void BIRRTstarViz::paint(QPaintDevice * device) {
  if(mpReferenceFrames==NULL) {
    return;
  }
  if(mpReferenceFrames->getWorldMap()==NULL) {
    return;
  }

  if(mShowRegions) {

    if( mRegionIndex < 0 ) {
      for( unsigned int i = 0; i < mpReferenceFrames->getWorldMap()->getSubregionSet().size(); i++) {
        SubRegionSet* p_subregion_set = mpReferenceFrames->getWorldMap()->getSubregionSet()[i];
        if(p_subregion_set) {
          QPainter rg_painter(device);
          rg_painter.setRenderHint(QPainter::Antialiasing);
          QBrush rg_brush( mColors[i] );
          rg_painter.setPen(Qt::NoPen);
          for( unsigned int j = 0; j < p_subregion_set->mSubregions.size(); j ++ ) {
            SubRegion* p_subreg = p_subregion_set->mSubregions[j];
            if(p_subreg) {
              QPolygon poly;
              for( unsigned int k = 0; k < p_subreg->mPoints.size(); k++) {
                double x = CGAL::to_double( p_subreg->mPoints[k].x() );
                double y = CGAL::to_double( p_subreg->mPoints[k].y() );
                poly << QPoint(x , y);
              }
              QPainterPath tmpPath;
              tmpPath.addPolygon(poly);
              rg_painter.fillPath(tmpPath, rg_brush);
            }
          }
          rg_painter.end();
        }
      }
    }
    else {
      SubRegionSet* p_subregion_set = mpReferenceFrames->getWorldMap()->getSubregionSet()[ mRegionIndex ];
      if(p_subregion_set) {
        QPainter rg_painter(device);
        rg_painter.setRenderHint(QPainter::Antialiasing);
        QBrush rg_brush( mColors[mRegionIndex] );
        rg_painter.setPen(Qt::NoPen);

        if( mSubregionIndex < 0 ) {
          for( unsigned int j = 0; j < p_subregion_set->mSubregions.size(); j ++ ) {
            SubRegion* p_subreg = p_subregion_set->mSubregions[j];
            if(p_subreg) {
              QPolygon poly;
              for( unsigned int k = 0; k < p_subreg->mPoints.size(); k++) {
                double x = CGAL::to_double( p_subreg->mPoints[k].x() );
                double y = CGAL::to_double( p_subreg->mPoints[k].y() );
                poly << QPoint(x , y);
              }
              QPainterPath tmpPath;
              tmpPath.addPolygon(poly);
              rg_painter.fillPath(tmpPath, rg_brush);
            }
          }
        }
        else {
          SubRegion* p_subreg = p_subregion_set->mSubregions[mSubregionIndex];
          if(p_subreg) {
            QPolygon poly;
            for( unsigned int k = 0; k < p_subreg->mPoints.size(); k++) {
              double x = CGAL::to_double( p_subreg->mPoints[k].x() );
              double y = CGAL::to_double( p_subreg->mPoints[k].y() );
              poly << QPoint(x , y);
            }
            QPainterPath tmpPath;
            tmpPath.addPolygon(poly);
            rg_painter.fillPath(tmpPath, rg_brush);
          }
        }
        rg_painter.end();
      } 
    }
  }

  if(mpTree) {
    if(mTreeShowType == START_TREE_SHOW || mTreeShowType == BOTH_TREES_SHOW ) {
      QPainter st_tree_painter(device);
      QPen st_tree_paintpen;
      if(mFinishedPlanning) {
        st_tree_paintpen.setColor(START_TREE_COLOR_ALPHA);
      }
      else{
        st_tree_paintpen.setColor(START_TREE_COLOR);
      }
      st_tree_paintpen.setWidth(1);
      if(mFinishedPlanning) {
        st_tree_painter.setOpacity(0.4);
      }
      st_tree_painter.setPen(st_tree_paintpen);
      for( std::list<BIRRTNode*>::iterator it= mpTree->getSTNodes().begin(); it!=mpTree->getSTNodes().end();it++ ) {
        BIRRTNode* p_node = (*it);
        if(p_node) {
          if(p_node->mpParent) {
            st_tree_painter.drawLine(QPoint(p_node->mPos[0], p_node->mPos[1]), QPoint(p_node->mpParent->mPos[0], p_node->mpParent->mPos[1]));
          }
        }
      }
      st_tree_painter.end();
    }
    if(mTreeShowType == GOAL_TREE_SHOW || mTreeShowType == BOTH_TREES_SHOW) {
      QPainter gt_tree_painter(device);
      QPen gt_tree_paintpen;
      if(mFinishedPlanning) {
        gt_tree_paintpen.setColor(GOAL_TREE_COLOR_ALPHA);
      }
      else{
        gt_tree_paintpen.setColor(GOAL_TREE_COLOR);
      }
      gt_tree_paintpen.setWidth(1);
      if(mFinishedPlanning) {
        gt_tree_painter.setOpacity(0.4);
      }
      gt_tree_painter.setPen(gt_tree_paintpen);
      for( std::list<BIRRTNode*>::iterator it= mpTree->getGTNodes().begin(); it!=mpTree->getGTNodes().end();it++ ) {
        BIRRTNode* p_node = (*it);
        if(p_node) {
          if(p_node->mpParent) {
            gt_tree_painter.drawLine(QPoint(p_node->mPos[0], p_node->mPos[1]), QPoint(p_node->mpParent->mPos[0], p_node->mpParent->mPos[1]));
          }
        }
      }
      gt_tree_painter.end();
    }
  } 
  if(m_PPInfo.mpFoundPaths.size() > 0 && mFoundPathIndex >= 0  ) {
    Path* p = m_PPInfo.mpFoundPaths[mFoundPathIndex];
    QPainter fpt_painter(device);
    QPen fpt_paintpen(QColor(255,140,0));
    fpt_paintpen.setWidth(4);
    fpt_painter.setPen(fpt_paintpen);

    int point_num = p->mWaypoints.size();
    if(point_num > 0) {
      for(int i=0;i<point_num-1;i++) {
        fpt_painter.drawLine(QPoint(p->mWaypoints[i][0], p->mWaypoints[i][1]), QPoint(p->mWaypoints[i+1][0], p->mWaypoints[i+1][1]));
      }
    }
    fpt_painter.end();
  }
    
  if(m_PPInfo.m_start.x() >= 0 && m_PPInfo.m_start.y() >= 0) {
    QPainter st_painter(device);
    QPen st_paintpen( START_COLOR );
    st_paintpen.setWidth(8);
    st_painter.setPen(st_paintpen);
    st_painter.drawPoint(m_PPInfo.m_start);
    st_painter.end();
  }

  if(m_PPInfo.m_goal.x() >= 0 && m_PPInfo.m_goal.y() >= 0) {
    QPainter gt_painter(device);
    QPen gt_paintpen( GOAL_COLOR );
    gt_paintpen.setWidth(8);
    gt_painter.setPen(gt_paintpen);
    gt_painter.drawPoint(m_PPInfo.m_goal);
    gt_painter.end();
  }

  if( mShowReferenceFrames ) {
    if( mpReferenceFrames ) {
      QPainter rf_painter(device);
      QPen rf_paintpen( REFERENCE_FRAME_COLOR );
      rf_paintpen.setWidth(2);
      rf_painter.setPen(rf_paintpen);

      if ( mReferenceFrameIndex >= mpReferenceFrames->getReferenceFrames().size() ) {
        for( unsigned int rf_i = 0; rf_i < mpReferenceFrames->getReferenceFrames().size(); rf_i ++ ) {
          ReferenceFrame* rf = mpReferenceFrames->getReferenceFrames()[rf_i];
          rf_painter.drawLine( QPoint( CGAL::to_double(rf->mSegment.source().x()), CGAL::to_double(rf->mSegment.source().y()) ),
                               QPoint( CGAL::to_double(rf->mSegment.target().x()), CGAL::to_double(rf->mSegment.target().y()) ) );
        }
      }
      else{
        ReferenceFrame* rf = mpReferenceFrames->getReferenceFrames()[mReferenceFrameIndex];
        rf_painter.drawLine( QPoint( CGAL::to_double(rf->mSegment.source().x()), CGAL::to_double(rf->mSegment.source().y()) ),
                             QPoint( CGAL::to_double(rf->mSegment.target().x()), CGAL::to_double(rf->mSegment.target().y()) ) );
      }
      rf_painter.end();
    }
  }

  if( mpTree!=NULL && mFinishedPlanning==false ) {
    if( mpTree->getStringClassMgr() ) {
      std::vector< StringClass* > classes = mpTree->getStringClassMgr()->getStringClasses();
      QPainter path_painter(device);
      QPen path_paintpen( PATH_COLOR );
      path_paintpen.setWidth(3);
      path_paintpen.setStyle( Qt::DashDotLine );
      path_painter.setPen(path_paintpen);
      for( unsigned int i = 0; i < classes.size(); i ++ ) {
        Path* p_path = classes[i]->mpPath;
        if( p_path ) {
          if( p_path->mWaypoints.size() > 0 ) {
            for( unsigned int j = 0; j < p_path->mWaypoints.size()-1; j ++ ){
              path_painter.drawLine( QPoint( p_path->mWaypoints[j][0],
                                             p_path->mWaypoints[j][1] ), 
                                     QPoint( p_path->mWaypoints[j+1][0],
                                             p_path->mWaypoints[j+1][1] ) );
            }
          }
        } 
      }
      path_painter.end();
    }
  }

  if( mShowPoints ) {
    QPainter draw_line_painter(device);
    QPen draw_line_pen( DRAWING_LINE_COLOR );
    draw_line_pen.setWidth( LINE_WIDTH );
    draw_line_painter.setPen(draw_line_pen);
    if( mDrawedPoints.size() > 1 ) {
      for( unsigned int pi = 0; pi < mDrawedPoints.size()-1 ; pi ++ ) {
        draw_line_painter.drawLine( mDrawedPoints[pi], mDrawedPoints[pi+1] );
      }
    }
    draw_line_painter.end();
  }
}

void BIRRTstarViz::setShowReferenceFrames(bool show) {
  mShowReferenceFrames = show;
  mReferenceFrameIndex = 0;
}

void BIRRTstarViz::setShowRegions(bool show) {
  mShowRegions = show;
}

bool BIRRTstarViz::drawPath(QString filename) {

  QPixmap pix(m_PPInfo.mObjectiveFile);

  std::cout << "DUMP PATH IMG " << pix.width() << " " << pix.height() << std::endl;

  QFile file(filename);
  if(file.open(QIODevice::WriteOnly)) {
    if(m_PPInfo.mpFoundPaths[ mFoundPathIndex ]) {
      drawPathOnMap(pix);
    }
    pix.save(&file, "PNG");
    return true;
  }
  return false;
}

bool BIRRTstarViz::saveCurrentViz(QString filename) {
  QPixmap pix(m_PPInfo.mMapFullpath);
  QFile file(filename);
  if(file.open(QIODevice::WriteOnly)) {
    paint( dynamic_cast<QPaintDevice*>(&pix) );
    pix.save(&file, "PNG");
    return true;
  }
  return false;
}

void BIRRTstarViz::drawPathOnMap(QPixmap& map) {

  Path * p = m_PPInfo.mpFoundPaths[ mFoundPathIndex ];
  QPainter painter(&map);
  QPen paintpen(QColor(255,140,0));
  paintpen.setWidth(2);
  painter.setPen(paintpen);

  int point_num = p->mWaypoints.size();

  if(point_num > 0) {
    for(int i=0;i<point_num-1;i++) {
      painter.drawLine( QPoint(p->mWaypoints[i][0], p->mWaypoints[i][1]), QPoint(p->mWaypoints[i+1][0], p->mWaypoints[i+1][1]) );
    }
  }

  painter.end();

  QPainter startPainter(&map);
  QPen paintpen1(QColor(255,0,0));
  paintpen.setWidth(10);
  startPainter.setPen(paintpen1);
  startPainter.end();

  startPainter.drawPoint( QPoint(p->mWaypoints[0][0], p->mWaypoints[0][1]) );
  int lastIdx = p->mWaypoints.size() - 1;
  QPainter endPainter(&map);
  QPen paintpen2(QColor(0,0,255));
  paintpen.setWidth(10);
  endPainter.setPen(paintpen2);
  endPainter.drawPoint( QPoint(p->mWaypoints[lastIdx][0], p->mWaypoints[lastIdx][1]) );
  endPainter.end();
        
}

void BIRRTstarViz::switchTreeShowType() {

  switch(mTreeShowType) {
  case NONE_TREE_SHOW:
    mTreeShowType = START_TREE_SHOW;
    break;
  case START_TREE_SHOW:
    mTreeShowType = GOAL_TREE_SHOW;
    break;
  case GOAL_TREE_SHOW:
    mTreeShowType = BOTH_TREES_SHOW;
    break;
  case BOTH_TREES_SHOW:
    mTreeShowType = NONE_TREE_SHOW;
    break;
  }
}

void BIRRTstarViz::prevReferenceFrame() {
  if(mpReferenceFrames) {
    if (mShowReferenceFrames) {
      if ( mReferenceFrameIndex <= 0) {
        mReferenceFrameIndex = mpReferenceFrames->getReferenceFrames().size();
      }else{
        mReferenceFrameIndex -- ;
      }
    }
  }
}

void BIRRTstarViz::nextReferenceFrame() {
  if(mpReferenceFrames) {
    if (mShowReferenceFrames) {
      if ( mReferenceFrameIndex >= mpReferenceFrames->getReferenceFrames().size() ) {
        mReferenceFrameIndex = 0;
      }else{
        mReferenceFrameIndex ++;
      }
    }
  }
}

std::string BIRRTstarViz::getReferenceFrameName() {

  if (mpReferenceFrames) {
    if ( mReferenceFrameIndex < mpReferenceFrames->getReferenceFrames().size() ) {
      return mpReferenceFrames->getReferenceFrames()[mReferenceFrameIndex]->mName;
    }
  }
  return "NO REF FRAME";
}

std::string BIRRTstarViz::getRegionName() {
  SubRegion* p_subregion = getSelectedSubregion();
  if( p_subregion ) {
    return p_subregion->getName();
  }
  SubRegionSet* p_subregion_set = getSelectedSubregionSet();
  if( p_subregion_set ) {
    return p_subregion_set->getName();
  }
  return "NO REGION";
}

void BIRRTstarViz::prevFoundPath() {
  if ( m_PPInfo.mpFoundPaths.size() == 0 ) {
    return;
  }
  if ( mFoundPathIndex < 0 ) {
    mFoundPathIndex = m_PPInfo.mpFoundPaths.size() - 1;
  } else {
    mFoundPathIndex --;
  }
}

void BIRRTstarViz::nextFoundPath() {
  if ( m_PPInfo.mpFoundPaths.size() == 0 ) {
    return;
  }
  if ( mFoundPathIndex >= m_PPInfo.mpFoundPaths.size()-1 ) {
    mFoundPathIndex = -1;
  } else {
    mFoundPathIndex ++;
  }
}

void BIRRTstarViz::importStringConstraint( std::vector< QPoint > points, grammar_type_t type ) {
  std::vector< Point2D > ref_points;
  for( unsigned int i = 0; i < points.size(); i ++ ) {
    ref_points.push_back( Point2D( points[i].x(), points[i].y() ) );
  }
  if( mpReferenceFrames ) {
    mpReferenceFrames->importStringConstraint( ref_points, type );
  }
}

void BIRRTstarViz::mousePressEvent( QMouseEvent * event ) {
  // std::cout << "mousePressEvent" << std::endl;
  if ( event->button() == Qt::LeftButton ) {
    mDragging = true;
    mShowPoints = true;
    mDrawedPoints.clear();
  }
}

void BIRRTstarViz::mouseMoveEvent( QMouseEvent * event ) {
  // std::cout << "mouseMoveEvent " << mPoints.size() << std::endl;
  if ( mDragging == true ) {
    //std::cout << event->x() << " " << event->y() << std::endl;
    QPoint new_point( event->x(), event->y() );
    if( mDrawedPoints.size() > 0 ) {
      QPoint last_point = mDrawedPoints.back();
      if( std::abs( new_point.x() - last_point.x() ) > 1 &&
          std::abs( new_point.y() - last_point.y() ) > 1 ) {
        mDrawedPoints.push_back( new_point );
      }
    }
    else {
      mDrawedPoints.push_back( new_point );
    }
    repaint();
  }
}

void BIRRTstarViz::mouseReleaseEvent( QMouseEvent * event ){
  // std::cout << "mouseReleaseEvent" << std::endl;
  if ( event->button() == Qt::LeftButton ) {
    mDragging = false;
  }
}

ReferenceFrame* BIRRTstarViz::getSelectedReferenceFrame() {

  if(mpReferenceFrames == NULL) {
      return NULL;
  }
  if ( mReferenceFrameIndex >= mpReferenceFrames->getReferenceFrames().size() ) {
    return NULL;
  }
  if ( mReferenceFrameIndex < 0 ) {
    return NULL;
  }
  return mpReferenceFrames->getReferenceFrames()[ mReferenceFrameIndex ];
}
    
SubRegionSet* BIRRTstarViz::getSelectedSubregionSet() {

  if ( mpReferenceFrames == NULL ) {
      return NULL;
  }
  if ( mRegionIndex >= mpReferenceFrames->getWorldMap()->getSubregionSet().size() ) {
    return NULL;
  }
  if ( mRegionIndex < 0 ) {
    return NULL;
  }
  return mpReferenceFrames->getWorldMap()->getSubregionSet()[ mRegionIndex ];
}

SubRegion* BIRRTstarViz::getSelectedSubregion() {

  SubRegionSet* p_subregion_set = getSelectedSubregionSet();
  if (p_subregion_set) {
    if( mSubregionIndex >= 0 && mSubregionIndex < p_subregion_set->mSubregions.size() ) {
      return p_subregion_set->mSubregions[ mSubregionIndex ];
    } 
    return NULL;
  } 
  return NULL;
}

void BIRRTstarViz::prevRegion() {
  if(mpReferenceFrames) {
    if (mShowRegions) {
      if ( mRegionIndex < 0) {
        mRegionIndex = mpReferenceFrames->getWorldMap()->getSubregionSet().size() - 1;
        mSubregionIndex = -1;
      }else{
        mRegionIndex -- ;
        mSubregionIndex = -1;
      }
    }
  }
}

void BIRRTstarViz::nextRegion() {
  if(mpReferenceFrames) {
    if (mShowRegions) {
      if ( mRegionIndex >= mpReferenceFrames->getWorldMap()->getSubregionSet().size()-1 ) {
        mRegionIndex = -1;
        mSubregionIndex = -1;
      }else{
        mRegionIndex ++;
        mSubregionIndex = -1;
      }
    }
  }
}
 
void BIRRTstarViz::prevSubregion() {
  if(mpReferenceFrames) {
    if (mShowRegions) {
      if( mRegionIndex >= 0 && mRegionIndex < mpReferenceFrames->getWorldMap()->getSubregionSet().size()-1 ) {
        SubRegionSet* p_subregions = mpReferenceFrames->getWorldMap()->getSubregionSet()[ mRegionIndex ];
        if( mSubregionIndex > 0 ) {
          mSubregionIndex --;
        }
        else {
          mSubregionIndex = p_subregions->mSubregions.size()-1;
        }
      }
    }
  }
}

void BIRRTstarViz::nextSubregion() {
  if(mpReferenceFrames) {
    if (mShowRegions) {
      if( mRegionIndex >= 0 && mRegionIndex < mpReferenceFrames->getWorldMap()->getSubregionSet().size()-1 ) {
        SubRegionSet* p_subregions = mpReferenceFrames->getWorldMap()->getSubregionSet()[ mRegionIndex ];
        if( mSubregionIndex < p_subregions->mSubregions.size()-1 ) {
          mSubregionIndex ++;
        }
        else {
          mSubregionIndex = 0;
        }
      }
    }
  }
}

void BIRRTstarViz::reset() {
  mpTree = NULL;
  mFinishedPlanning = false;
  mReferenceFrameIndex = -1;
  mFoundPathIndex = -1;
  mRegionIndex = -1;
  mSubregionIndex = -1;
  mTreeShowType = BOTH_TREES_SHOW;
  mShowPoints = false;
}

} // harrts

} // topologyPathPlanning
