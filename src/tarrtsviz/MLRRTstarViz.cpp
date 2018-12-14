#include <QtGui>

#include "topologyPathPlanning/tarrtsviz/MLRRTstarViz.hpp"
#include "topologyPathPlanning/tarrtsviz/MLVizUtil.hpp"

#define TEXT_COLOR        QColor(0, 0, 0)
#define TREE_COLOR        QColor(160,160,0)
#define TREE_COLOR_ALPHA  QColor(160,160,0,100)
#define START_COLOR             QColor(255,0,0)
#define GOAL_COLOR              QColor(0,0,255)
#define REFERENCE_FRAME_COLOR   QColor(0,255,0)
#define SUBREGION_COLOR         QColor(204,229,255)
#define PATH_COLOR              QColor(255,153,21)
#define DRAWING_LINE_COLOR      QColor(153,76,0)
#define TREE_WIDTH              1
#define LINE_WIDTH              2
#define PATH_WIDTH              4
#define POINT_WIDTH             8
#define TREE_OPACITY            0.4

using namespace std;
using namespace topologyPathPlanning::homotopy;

namespace topologyPathPlanning {

namespace tarrts {

MLRRTstarViz::MLRRTstarViz( QWidget * parent ) : QLabel(parent) {

  mpTree = NULL;
  mShowReferenceFrames = false;
  mShowSubregions = false;
  mShowPaths = true;
  mShowTree = true;
  mFinishedPlanning = false;
  mReferenceFrameIndex = -1;
  mFoundPathIndex = -1;
  mSubregionIndex = -1;
  mStringClassIndex = -1;
  mExpNodeIndex = -1;
  mExpNodeNum = 0;
  mpReferenceFrames = NULL;
  mShowPoints = false;
  mMode = NORMAL;
  mItemSelectedName = "";
}

void MLRRTstarViz::setTree( MLRRTstar* p_tree ) {
  mpTree = p_tree;
}

void MLRRTstarViz::setReferenceFrameSet( ReferenceFrameSet* p_rf ) {
  mpReferenceFrames = p_rf;
  updateVizSubregions();
  updateVizReferenceFrames();
}

void MLRRTstarViz::updateVizSubregions() {

  mVizSubregions.clear();
  if( mStringClassIndex < 0 ) {
    for( unsigned int i = 0; i < mpReferenceFrames->getWorldMap()->getSubregionSet().size(); i++) {
      SubRegionSet* p_subregion_set = mpReferenceFrames->getWorldMap()->getSubregionSet()[i];
      if(p_subregion_set) {
        for( unsigned int j = 0; j < p_subregion_set->mSubregions.size(); j ++ ) {
          SubRegion* p_subreg = p_subregion_set->mSubregions[j];
          mVizSubregions.push_back( p_subreg );
        }
      }
    }
  }
  else {
    StringClass* p_str_cls = mpTree->getExpandingTreeMgr()->getStringClasses()[ mStringClassIndex ];
    if( p_str_cls ) {
      mExpNodeNum = p_str_cls->mp_exp_nodes.size();

      for( vector<ExpandingNode*>::iterator it_exp = p_str_cls->mp_exp_nodes.begin();
           it_exp != p_str_cls->mp_exp_nodes.end(); it_exp++ ) {
        ExpandingNode* p_exp_node = (*it_exp);
        if ( p_exp_node ) {
          SubRegion* p_subreg = p_exp_node->mpSubregion;
          if ( p_subreg ) {
            mVizSubregions.push_back( p_subreg );
          }
        }
      }
    }
  }
  mSubregionIndex = -1;
}

void MLRRTstarViz::updateVizReferenceFrames() {

  mVizReferenceFrames.clear();
  if( mStringClassIndex < 0 ) {
    for( int i = 0; i < (signed)mpReferenceFrames->getReferenceFrames().size(); i ++ ) {
      ReferenceFrame* p_rf = mpReferenceFrames->getReferenceFrames()[i];
      mVizReferenceFrames.push_back( p_rf );
    }
  }
  else {
    StringClass* p_str_cls = mpTree->getExpandingTreeMgr()->getStringClasses()[ mStringClassIndex ];
    if( p_str_cls ) {
      for( vector<ReferenceFrame*>::iterator it_rf = p_str_cls->mp_reference_frames.begin();
           it_rf != p_str_cls->mp_reference_frames.end(); it_rf++ ) {
        ReferenceFrame* p_rf = (*it_rf);
        mVizReferenceFrames.push_back( p_rf );
      }
    }
  }
  mReferenceFrameIndex = -1;
}

void MLRRTstarViz::paint( QPaintDevice* device ) {

  /* DRAW SUB REGION */
  if(mShowSubregions) {

    QPainter rg_painter(device);
    rg_painter.setRenderHint(QPainter::Antialiasing);
    QBrush rg_brush( SUBREGION_COLOR );
    rg_painter.setPen(Qt::NoPen);
    if( mSubregionIndex < 0 ) {
      for( unsigned int i = 0; i < mVizSubregions.size(); i ++ ) {
        SubRegion* p_subreg = mVizSubregions[i];
        if(p_subreg) {
          QPolygon poly;
          for( unsigned int k = 0; k < p_subreg->mPoints.size(); k++) {
            poly << toQPoint( p_subreg->mPoints[k] );
          }
          QPainterPath tmpPath;
          tmpPath.addPolygon(poly);
          rg_painter.fillPath(tmpPath, rg_brush);
        }
      }
    }
    else {
      SubRegion* p_subreg = mVizSubregions[mSubregionIndex];
      if(p_subreg) {
        QPolygon poly;
        for( unsigned int k = 0; k < p_subreg->mPoints.size(); k++) {
          poly << toQPoint( p_subreg->mPoints[k] );
        }
        QPainterPath tmpPath;
        tmpPath.addPolygon(poly);
        rg_painter.fillPath(tmpPath, rg_brush);
      }
    }
    rg_painter.end();
  }

  /* DRAW TREE */
  if( mpTree ) {
    if( mShowTree ) {
      QPainter tree_painter(device);
      QPen tree_paintpen;
      if(mFinishedPlanning) {
        tree_paintpen.setColor(TREE_COLOR_ALPHA);
      }
      else{
        tree_paintpen.setColor(TREE_COLOR);
      }
      tree_paintpen.setWidth( TREE_WIDTH );
      if(mFinishedPlanning) {
        tree_painter.setOpacity( TREE_OPACITY );
      }
      tree_painter.setPen(tree_paintpen);
      if( mStringClassIndex < 0 ) {
        for( list<MLRRTNode*>::iterator it= mpTree->getNodes().begin();
             it!=mpTree->getNodes().end();it++ ) {
          MLRRTNode* p_node = (*it);
          if(p_node) {
            if(p_node->mpParent) {
              tree_painter.drawLine( toQPoint( p_node->mpParent->mPos ),
                                     toQPoint( p_node->mPos ) );
            }
          }
        }
      }
      else {
        StringClass* p_str_cls = mpTree->getExpandingTreeMgr()->getStringClasses()[ mStringClassIndex ];
        if( p_str_cls ) {

          if( mExpNodeIndex < 0 ) {
            for( vector<ExpandingNode*>::iterator it_exp = p_str_cls->mp_exp_nodes.begin();
                 it_exp != p_str_cls->mp_exp_nodes.end(); it_exp++ ) {
              ExpandingNode* p_exp_node = (*it_exp);
              if ( p_exp_node ) {
                for( list<MLRRTNode*>::iterator it = p_exp_node->mpNodes.begin();
                     it != p_exp_node->mpNodes.end(); it++ ) {
                  MLRRTNode* p_node = (*it);
                  if(p_node) {
                    if(p_node->mpParent) {
                      tree_painter.drawLine( toQPoint( p_node->mpParent->mPos ),
                                           toQPoint( p_node->mPos ) );
                    }
                  }
                }
              }
            }
          }
          else {
            ExpandingNode* p_exp_node = p_str_cls->mp_exp_nodes[ mExpNodeIndex ];
            if( p_exp_node ) {
              for( list<MLRRTNode*>::iterator it = p_exp_node->mpNodes.begin();
                   it != p_exp_node->mpNodes.end(); it++ ) {
                MLRRTNode* p_node = (*it);
                if(p_node) {
                  if(p_node->mpParent) {
                    tree_painter.drawLine( toQPoint( p_node->mpParent->mPos ),
                                           toQPoint( p_node->mPos ) );
                  }
                }
              }
            }

          }
        }
      }
      tree_painter.end();
    }
  }

  /* DRAW PATHS */
  if( mShowPaths ) {
    QPainter fpt_painter(device);
    QPen fpt_paintpen( PATH_COLOR );
    fpt_paintpen.setWidth( PATH_WIDTH );
    fpt_painter.setPen(fpt_paintpen);

    Path* p_viz_path = getVizPath();
    if( p_viz_path ) {
      int point_num = p_viz_path->mWaypoints.size();
      if(point_num > 0) {
        for(int i=0;i<point_num-1;i++) {
          fpt_painter.drawLine( toQPoint(p_viz_path->mWaypoints[i]),
                                toQPoint(p_viz_path->mWaypoints[i+1]) );
        }
      }
    }
    fpt_painter.end();
  }

  /* DRAW DRAWED POINTS */
  if( mShowPoints ) {
    if( mDrawedPoints.size() > 1 ) {
      QPainter dp_painter(device);
      QPen dp_paintpen( DRAWING_LINE_COLOR );
      dp_paintpen.setWidth( LINE_WIDTH );
      dp_painter.setPen(dp_paintpen);
      for( unsigned int i = 0; i < mDrawedPoints.size()-1; i++ ) {
        dp_painter.drawLine( mDrawedPoints[ i ] ,
                             mDrawedPoints[ i+1 ] );
      }
    }
  }

  /* DRAW REFERENCE FRAMES */
  if( mShowReferenceFrames ) {
    QPainter rf_painter(device);
    QPen rf_paintpen( REFERENCE_FRAME_COLOR );
    rf_paintpen.setWidth( LINE_WIDTH );
    rf_painter.setPen(rf_paintpen);
    if( mReferenceFrameIndex < 0 ) {
     for( unsigned int rf_i = 0; rf_i < mVizReferenceFrames.size(); rf_i ++ ) {
       ReferenceFrame* rf = mVizReferenceFrames[rf_i];
       rf_painter.drawLine( toQPoint( rf->mSegment.source() ),
                            toQPoint( rf->mSegment.target() ) );
      }
    }
    else{
      ReferenceFrame* rf = mVizReferenceFrames[mReferenceFrameIndex];
      rf_painter.drawLine( toQPoint( rf->mSegment.source() ),
                           toQPoint( rf->mSegment.target() ) );
    }
    rf_painter.end();
  }

  /* DRAW START AND GOAL */
  if(m_PPInfo.mStart.x() >= 0 && m_PPInfo.mStart.y() >= 0) {
    QPainter st_painter(device);
    QPen st_paintpen( START_COLOR );
    st_paintpen.setWidth( POINT_WIDTH );
    st_painter.setPen(st_paintpen);
    st_painter.drawPoint( m_PPInfo.mStart );
    st_painter.end();
  }

  if(m_PPInfo.mGoal.x() >= 0 && m_PPInfo.mGoal.y() >= 0) {
    QPainter gt_painter(device);
    QPen gt_paintpen( GOAL_COLOR );
    gt_paintpen.setWidth( POINT_WIDTH );
    gt_painter.setPen(gt_paintpen);
    gt_painter.drawPoint( m_PPInfo.mGoal );
    gt_painter.end();
  }

  if( NORMAL == mMode ) {
    if ( mItemSelectedName != "" ) {
      QPainter text_painter(device);
      QPen text_pen( TEXT_COLOR );
      text_painter.setPen(text_pen);
      QRect rect = QRect( 5, 5, 100, 10 );
      text_painter.drawText( rect, Qt::AlignCenter, mItemSelectedName );
    }
  }
}

void MLRRTstarViz::paintEvent( QPaintEvent* e ) {
  QLabel::paintEvent(e);
  paint( this );
}

void MLRRTstarViz::setShowReferenceFrames(bool show) {
  mShowReferenceFrames = show;
  mReferenceFrameIndex = -1;
}

void MLRRTstarViz::setShowSubregions(bool show) {
  mShowSubregions = show;
  mSubregionIndex = -1;
}

void MLRRTstarViz::setShowTree(bool show) {
  mShowTree = show;
}

void MLRRTstarViz::setShowPaths(bool show) {
  mShowPaths = show;
  mFoundPathIndex = -1;
}

bool MLRRTstarViz::drawPath(QString filename) {

  QPixmap pix(m_PPInfo.mObjectiveFile);
  cout << "DUMP PATH IMG " << pix.width() << " " << pix.height() << endl;

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

bool MLRRTstarViz::saveCurrentViz(QString filename) {
  QPixmap pix(m_PPInfo.mMapFullpath);
  if( m_PPInfo.mMinDistEnabled == false ) {
    pix = QPixmap( m_PPInfo.mObjectiveFile );
  }
  QFile file(filename);
  if(file.open(QIODevice::WriteOnly)) {
    paint( dynamic_cast<QPaintDevice*>(&pix) );
    pix.save(&file, "PNG");
    return true;
  }
  return false;
}

void MLRRTstarViz::nextExpNode() {

  if ( mExpNodeIndex < mExpNodeNum - 1 ) {
    mExpNodeIndex ++;
  }
  else {
    mExpNodeIndex = -1;
  }
}

void MLRRTstarViz::prevExpNode() {

  if ( mExpNodeIndex >= 0 ) {
     mExpNodeIndex --;
  }
  else {
     mExpNodeIndex = mExpNodeNum-1;
  }
}

void MLRRTstarViz::nextStringClass() {
  if ( mpTree ) {
    if ( mpTree->getExpandingTreeMgr() ) {
      ExpandingTreeMgr* p_mgr = mpTree->getExpandingTreeMgr();
      if ( mStringClassIndex < (signed)p_mgr->getStringClasses().size()-1 ) {
        mStringClassIndex ++;
        updateVizSubregions();
        updateVizReferenceFrames();
        mFoundPathIndex = -1;
        mExpNodeIndex = -1;
      }
      else {
        mStringClassIndex = -1;
        updateVizSubregions();
        updateVizReferenceFrames();
        mFoundPathIndex = -1;
        mExpNodeIndex = -1;
      }
    }
  }
}

void MLRRTstarViz::prevStringClass() {
  if ( mpTree ) {
    if ( mpTree->getExpandingTreeMgr() ) {
      ExpandingTreeMgr* p_mgr = mpTree->getExpandingTreeMgr();
      if ( mStringClassIndex >= 0 ) {
        mStringClassIndex --;
        updateVizSubregions();
        updateVizReferenceFrames();
        mFoundPathIndex = -1;
      }
      else {
        mStringClassIndex = p_mgr->getStringClasses().size()-1;
        updateVizSubregions();
        updateVizReferenceFrames();
        mFoundPathIndex = -1;
      }
    }
  }
}

void MLRRTstarViz::prevReferenceFrame() {
  if (mShowReferenceFrames) {
    if ( mReferenceFrameIndex < 0) {
      mReferenceFrameIndex = mVizReferenceFrames.size()-1;
    }else{
      mReferenceFrameIndex -- ;
    }
  }
}

void MLRRTstarViz::nextReferenceFrame() {
  if (mShowReferenceFrames) {
    if ( mReferenceFrameIndex >= (signed)mVizReferenceFrames.size()-1 ) {
      mReferenceFrameIndex = -1;
    }else{
      mReferenceFrameIndex ++;
    }
  }
}

string MLRRTstarViz::getReferenceFrameName() {

  if ( mReferenceFrameIndex < (signed)mpReferenceFrames->getReferenceFrames().size() ) {
    return mpReferenceFrames->getReferenceFrames()[mReferenceFrameIndex]->mName;
  }
  return "NO REF FRAME";
}

string MLRRTstarViz::getSubregionName() {
  SubRegion* p_subregion = getSelectedSubregion();
  if( p_subregion ) {
    return p_subregion->getName();
  }
  return "NO REGION";
}

void MLRRTstarViz::prevFoundPath() {
  if ( m_PPInfo.mpFoundPaths.size() == 0 ) {
    return;
  }
  if ( mFoundPathIndex < 0 ) {
    mFoundPathIndex = m_PPInfo.mpFoundPaths.size() - 1;
  } else {
    mFoundPathIndex --;
  }
}

void MLRRTstarViz::nextFoundPath() {
  if ( m_PPInfo.mpFoundPaths.size() == 0 ) {
    return;
  }
  if ( mFoundPathIndex >= (signed)m_PPInfo.mpFoundPaths.size()-1 ) {
    mFoundPathIndex = -1;
  } else {
    mFoundPathIndex ++;
  }
}

void MLRRTstarViz::importStringConstraint( vector< QPoint > points, grammar_type_t type ) {
  vector< Point2D > ref_points;
  for( unsigned int i = 0; i < points.size(); i ++ ) {
    ref_points.push_back( Point2D( points[i].x(), points[i].y() ) );
  }
  if( mpReferenceFrames ) {
    mpReferenceFrames->importStringConstraint( ref_points, type );
  }
}

void MLRRTstarViz::mousePressEvent( QMouseEvent * event ) {
  // std::cout << "mousePressEvent" << std::endl;
  if ( event->button() == Qt::LeftButton ) {
    if( DRAWING == mMode ) {
      mDragging = true;
      mShowPoints = true;
      mDrawedPoints.clear();
    }
    else if( NORMAL == mMode ) {
      QPoint new_point( event->x(), event->y() );
      mItemSelectedName = itemSelected( new_point );
      cout << "POS (" << event->x() << ", " << event->y() << ") " << mItemSelectedName.toStdString() << endl;
    }
  }
}

void MLRRTstarViz::mouseMoveEvent( QMouseEvent * event ) {
  // std::cout << "mouseMoveEvent " << mPoints.size() << std::endl;
  if( DRAWING == mMode ) {
    if ( mDragging == true ) {
      //std::cout << event->x() << " " << event->y() << std::endl;
      QPoint new_point( event->x(), event->y() );
      if( mDrawedPoints.size() > 0 ) {
        QPoint last_point = mDrawedPoints.back();
        if( abs( new_point.x() - last_point.x() ) > 1 &&
            abs( new_point.y() - last_point.y() ) > 1 ) {
          mDrawedPoints.push_back( new_point );
        }
      }
      else {
        mDrawedPoints.push_back( new_point );
      }
      repaint();
    }
  }
}

void MLRRTstarViz::mouseReleaseEvent( QMouseEvent * event ){
  // std::cout << "mouseReleaseEvent" << std::endl;
  if ( event->button() == Qt::LeftButton ) {
    if( DRAWING == mMode ) {
      mDragging = false;
    }
    else if( NORMAL == mMode ) {
      mItemSelectedName = "";
    }
  }
}

ReferenceFrame* MLRRTstarViz::getSelectedReferenceFrame() {

  if ( mReferenceFrameIndex >= (signed)mpReferenceFrames->getReferenceFrames().size() ) {
    return NULL;
  }
  if ( mReferenceFrameIndex < 0 ) {
    return NULL;
  }
  return mpReferenceFrames->getReferenceFrames()[ mReferenceFrameIndex ];
}

SubRegion* MLRRTstarViz::getSelectedSubregion() {
  if( mSubregionIndex >= 0 && mSubregionIndex < (signed)mVizSubregions.size() ) {
    return mVizSubregions[ mSubregionIndex ];
  }
  return NULL;
}


void MLRRTstarViz::prevSubregion() {
  if (mShowSubregions) {
    if( mSubregionIndex > 0 ) {
      mSubregionIndex --;
    }
    else {
      mSubregionIndex = mVizSubregions.size()-1;
    }
  }
}

void MLRRTstarViz::nextSubregion() {
  if (mShowSubregions) {
    if( mSubregionIndex < (signed)mVizSubregions.size()-1 ) {
      mSubregionIndex ++;
    }
    else {
      mSubregionIndex = 0;
    }
  }
}

void MLRRTstarViz::drawPathOnMap(QPixmap& map) {

  Path * p = m_PPInfo.mpFoundPaths[ mFoundPathIndex ];
  QPainter painter(&map);
  QPen paintpen(QColor(255,140,0));
  paintpen.setWidth(2);
  painter.setPen(paintpen);

  int point_num = p->mWaypoints.size();

  if(point_num > 0) {
    for(int i=0;i<point_num-1;i++) {
      painter.drawLine( toQPoint( p->mWaypoints[i] ), toQPoint( p->mWaypoints[i+1] ) );
    }
  }
  painter.end();

  QPainter startPainter(&map);
  QPen paintpen1(QColor(255,0,0));
  paintpen.setWidth(10);
  startPainter.setPen(paintpen1);
  startPainter.end();

  startPainter.drawPoint( toQPoint( p->mWaypoints[0] ) );
  int lastIdx = p->mWaypoints.size() - 1;
  QPainter endPainter(&map);
  QPen paintpen2(QColor(0,0,255));
  paintpen.setWidth(10);
  endPainter.setPen(paintpen2);
  endPainter.drawPoint( toQPoint( p->mWaypoints[lastIdx] ) );
  endPainter.end();
}

QString MLRRTstarViz::getStringClassInfo() {
  QString info;
  if( mpTree ) {
    if( mpTree->getExpandingTreeMgr() ) {
      if( mStringClassIndex < 0 ) {
        info = "ALL";
      }
      else {
        info = QString::fromStdString( mpTree->getExpandingTreeMgr()->getStringClasses()[ mStringClassIndex ]->getName() );
      }
    }
  }
  return info;
}

QString MLRRTstarViz::itemSelected( QPoint pos ) {
  QString name = "";
  Point2D point = toPoint2D( pos );

  if ( mpReferenceFrames ) {
    LineSubSegment* p_line_sub_segment = mpReferenceFrames->getWorldMap()->findLinesubsegment( point );
    if( p_line_sub_segment ) {
      cout << "FIND " << p_line_sub_segment->getName() << endl;
      return QString::fromStdString( p_line_sub_segment->getName() );
    }
    SubRegion* p_subregion = mpReferenceFrames->getWorldMap()->findSubregion( point );
    if( p_subregion ) {
      cout << "FIND " << p_subregion->getName() << endl;
      return QString::fromStdString( p_subregion->getName() );
    }
  }
  return name;
}

Path* MLRRTstarViz::getVizPath() {

  /* DRAW PATHS */
  if( mShowPaths ) {

    if( mStringClassIndex < 0 ) {
      if(m_PPInfo.mpFoundPaths.size() > 0 && mFoundPathIndex >= 0  ) {
        Path* p = m_PPInfo.mpFoundPaths[mFoundPathIndex];
        return p;
      }
    }
    else {
      StringClass* p_str_cls = mpTree->getExpandingTreeMgr()->getStringClasses()[ mStringClassIndex ];
      if( p_str_cls ) {
        if ( p_str_cls->mp_path ) {
          return p_str_cls->mp_path;
        }
      }
    }
  }
  return NULL;
}

} // tarrts

} // topologyPathPlanning
