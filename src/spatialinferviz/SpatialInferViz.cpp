#include <cstdlib>
#include <QPainter>
#include <QMouseEvent>

#include "topologyPathPlanning/homotopy/ImgLoadUtil.hpp"
#include "topologyPathPlanning/spatialinferviz/SpatialInferViz.hpp"
#include "SIVizUtil.hpp"

#define LINE_WIDTH              1
#define SELECTED_LINE_WIDTH     2
#define LINE_WIDTH_HIGHLIGHTED  5
#define POINT_SIZE              4
#define ALPHA_COLOR             QColor(0,0,255)
#define BETA_COLOR              QColor(0,255,0)
#define CENTER_POINT_COLOR      QColor(255,0,0)
#define BK_COLOR                QColor(255,140,0)
#define INTERSECTION_COLOR      QColor(160,160,160)
#define TEXT_COLOR              QColor(0,0,0)
#define OBSTACLE_COLOR          QColor(125,125,125)
#define ASSOCIATED_OBSTACLE_COLOR QColor(255,0,0)
#define SELECTED_OBSTACLE_COLOR QColor(0,255,0)
#define LINE_HIGHLIGHTED_COLOR  QColor(204,204,0)
#define DRAWING_LINE_COLOR      QColor(153,76,0)
#define SUBREGION_COLOR         QColor(204,229,255)
#define START_COLOR             QColor(255,0,0)
#define GOAL_COLOR              QColor(0,0,255)

#define RULE_LINE_WIDTH         4
#define RULE_POS_COLOR          QColor(0,255,0)
#define RULE_NEG_COLOR          QColor(255,0,0)
#define STRING_CLASS_POINT_SIZE 4
#define STRING_CLASS_POINT_COLOR QColor(0,255,255)

using namespace std;
using namespace topologyPathPlanning::homotopy;

namespace topologyPathPlanning {

namespace topologyinference {

SpatialInferViz::SpatialInferViz(QWidget *parent) :
    QLabel(parent) {

  mpMgr = NULL;
  mpReferenceFrameSet = NULL;
  mWorldWidth = 0;
  mWorldHeight = 0;
  mRegionIdx = -1;
  mSubRegionIdx = -1;
  mSubsegmentSetIdx = -1;
  mSubsegmentIdx = -1;
  mStringClassIdx = -1;
  mShowSubsegment = true;
  mVizSubregions.clear();
  mVizSubsegments.clear();
  mpVizStringClass = NULL;
  mMode = SUBREGION;
}

bool SpatialInferViz::loadMap( QString filename ) {

  QPixmap pix(filename);
  if( pix.isNull() == true ) {
    return false;
  }
  QPixmap emptyPix(pix.width(), pix.height());
  emptyPix.fill(QColor("white"));
  std::cout << " EMPTY PIX " << emptyPix.width() << " * " << emptyPix.height() << std::endl;
  //setPixmap(pix);
  setPixmap(emptyPix);
  initWorld(filename);
  return true;
}

bool SpatialInferViz::initWorld(QString filename) {

  std::vector< std::vector<Point2D> > conts;
  int map_width = 0, map_height = 0;
  if (mpReferenceFrameSet) {
    delete mpReferenceFrameSet;
    mpReferenceFrameSet = NULL;
  }

  loadMapInfo( filename.toStdString(), map_width, map_height, conts );   
  //std::cout << "CREATE WORLD " << map_width << " * " << map_height << std::endl;
  mpReferenceFrameSet = new ReferenceFrameSet();
  mpReferenceFrameSet->init( map_width, map_height, conts );
  //std::cout << "NUM OF OBS " << conts.size() << std::endl;
  mpMgr = new SpatialRelationMgr( mpReferenceFrameSet->getWorldMap() );
  return true;
}

void SpatialInferViz::paintEvent(QPaintEvent * e) {
    QLabel::paintEvent(e);
  if (mpMgr) {
    if (mpMgr->mpWorldmap) {

      QPainter region_painter(this);
      region_painter.setRenderHint(QPainter::Antialiasing);
      QBrush region_brush( SUBREGION_COLOR );
      region_painter.setPen(Qt::NoPen);
      for( std::vector<SubRegion*>::iterator itr = mVizSubregions.begin();
           itr != mVizSubregions.end(); itr++ ) {  
  
        SubRegion* p_subreg = (*itr);
        if (p_subreg) {
          QPolygon poly;
          for( unsigned int j=0; j < p_subreg->mPoints.size(); j++ ) {
            poly << toQPoint( p_subreg->mPoints[j] );
          }
          QPainterPath tmpPath;
          tmpPath.addPolygon(poly);
          region_painter.fillPath(tmpPath, region_brush);
        }
      }
      region_painter.end();

      QPainter line_hl_painter(this);
      QPen line_hl_pen( LINE_HIGHLIGHTED_COLOR );
      line_hl_pen.setWidth( LINE_WIDTH_HIGHLIGHTED );
      line_hl_painter.setPen( line_hl_pen );
 
      for( std::vector< LineSubSegment* >::iterator itLSS = mVizSubsegments.begin();
           itLSS != mVizSubsegments.end(); itLSS++ ) {
        LineSubSegment* p_line_subsegment = (*itLSS);
        if( p_line_subsegment ) {
          line_hl_painter.drawLine( toQPoint( p_line_subsegment->mSubseg.source() ),
                                    toQPoint( p_line_subsegment->mSubseg.target() ) );
        }
      }
      line_hl_painter.end();

      std::vector<Obstacle*> obstacles =  mpMgr->mpWorldmap->getObstacles();
  
      QPainter obstacle_painter(this);
      obstacle_painter.setRenderHint(QPainter::Antialiasing);
      QPen obstacle_pen( OBSTACLE_COLOR );
      obstacle_painter.setPen(obstacle_pen);
      for( std::vector<Obstacle*>::iterator it = obstacles.begin();
           it != obstacles.end(); it++ ) {
        Obstacle* p_obstacle = (*it);
        if (p_obstacle) {
          QPolygon poly;
          for( Polygon2D::Vertex_iterator itP=p_obstacle->mPgn.vertices_begin();
               itP != p_obstacle->mPgn.vertices_end(); itP++ ) {
            Point2D p = (*itP);
            poly << toQPoint( p );
          }
          obstacle_painter.drawPolygon(poly);
        }
      }
      obstacle_painter.end();

      QPainter hl_obs_painter(this);
      hl_obs_painter.setRenderHint(QPainter::Antialiasing);
      QPen hl_obs_pen( ASSOCIATED_OBSTACLE_COLOR );
      hl_obs_pen.setWidth( SELECTED_LINE_WIDTH );
      hl_obs_painter.setPen(hl_obs_pen);
      LineSubSegment* p_subseg = getSelectedLineSubsegment();
      if( p_subseg ) {
        for( std::vector<Obstacle*>::iterator it = p_subseg->mConnectedObstacles.begin();
             it != p_subseg->mConnectedObstacles.end(); it++ ) {
          Obstacle* p_obstacle = (*it);
          if (p_obstacle) {
            QPolygon poly;
            for( Polygon2D::Vertex_iterator itP=p_obstacle->mPgn.vertices_begin();
                 itP != p_obstacle->mPgn.vertices_end(); itP++ ) {
              Point2D p = (*itP);
              poly << toQPoint( p );
            }
            hl_obs_painter.drawPolygon(poly);
          }
        }
      }
      hl_obs_painter.end();

      QPainter sl_obs_painter(this);
      sl_obs_painter.setRenderHint(QPainter::Antialiasing);
      QPen sl_obs_pen( SELECTED_OBSTACLE_COLOR );
      sl_obs_pen.setWidth( SELECTED_LINE_WIDTH );
      sl_obs_painter.setPen(sl_obs_pen);
      for( std::vector<Obstacle*>::iterator it = mSelectedObstacles.begin();
           it != mSelectedObstacles.end(); it++ ) {
        Obstacle* p_obstacle = (*it);
        if (p_obstacle) {
          QPolygon poly;
          for( Polygon2D::Vertex_iterator itP=p_obstacle->mPgn.vertices_begin();
               itP != p_obstacle->mPgn.vertices_end(); itP++ ) {
            Point2D p = (*itP);
            poly << toQPoint( p );
          }
          sl_obs_painter.drawPolygon(poly);
        }
      }
      sl_obs_painter.end();

      if ( mShowSubsegment == false ) {
        /*
        QPainter alpha_painter(this);
        QPen alpha_pen( ALPHA_COLOR );
        alpha_pen.setWidth( LINE_WIDTH );
        alpha_painter.setPen( alpha_pen );

        for( std::vector<Obstacle*>::iterator it= obstacles.begin();
             it != obstacles.end(); it++ ) {
          Obstacle* p_obstacle = (*it);
          if ( p_obstacle ) {
            Point2D a_src = p_obstacle->mp_alpha_seg->m_seg.source();
            Point2D a_end = p_obstacle->mp_alpha_seg->m_seg.target();
            double a_src_x = CGAL::to_double( a_src.x() );
            double a_src_y = CGAL::to_double( a_src.y() );
            double a_end_x = CGAL::to_double( a_end.x() );
            double a_end_y = CGAL::to_double( a_end.y() );
            alpha_painter.drawLine( QPoint( a_src_x, a_src_y ), QPoint( a_end_x, a_end_y ) );
          }
        }
        alpha_painter.end();

        QPainter beta_painter(this);
        QPen beta_pen( BETA_COLOR );
        beta_pen.setWidth( LINE_WIDTH );
        beta_painter.setPen( beta_pen );

        for( std::vector<Obstacle*>::iterator it = obstacles.begin();
             it != obstacles.end(); it++ ) {
          Obstacle* p_obstacle = (*it);
          if ( p_obstacle ) {
            Point2D b_src = p_obstacle->mp_beta_seg->m_seg.source();
            Point2D b_end = p_obstacle->mp_beta_seg->m_seg.target();
            double b_src_x = CGAL::to_double( b_src.x() );
            double b_src_y = CGAL::to_double( b_src.y() );
            double b_end_x = CGAL::to_double( b_end.x() );
            double b_end_y = CGAL::to_double( b_end.y() );
            beta_painter.drawLine( QPoint( b_src_x, b_src_y ), QPoint( b_end_x, b_end_y ) );
          }
        }
        beta_painter.end();
        */
      }
      else {
        QPainter a_subseg_painter(this);
        QPen a_subseg_pen( ALPHA_COLOR );
        a_subseg_pen.setWidth( LINE_WIDTH );
        a_subseg_painter.setPen( a_subseg_pen );
        for( std::vector<Obstacle*>::iterator it = obstacles.begin();
             it != obstacles.end(); it++ ) {
          Obstacle* p_obstacle = (*it);
          if ( p_obstacle ) {
            //std::cout << "OBS " << p_obstacle->get_index() << " ALPHA:" << p_obstacle->mp_alpha_seg->m_subsegs.size() << std::endl;
            for( std::vector< LineSubSegment* >::iterator itap = p_obstacle->mpAlphaSeg->mSubsegs.begin();
                 itap != p_obstacle->mpAlphaSeg->mSubsegs.end(); itap++ ) {
              LineSubSegment* p_subseg_a = (*itap);
              a_subseg_painter.drawLine( toQPoint( p_subseg_a->mSubseg.source() ), 
                                         toQPoint( p_subseg_a->mSubseg.target() ));
            }
          }
        }
        a_subseg_painter.end();

        QPainter b_subseg_painter(this);
        QPen b_subseg_pen( BETA_COLOR );
        b_subseg_pen.setWidth( LINE_WIDTH );
        b_subseg_painter.setPen( b_subseg_pen );
        for( std::vector<Obstacle*>::iterator it = obstacles.begin();
             it != obstacles.end(); it++ ) {
          Obstacle* p_obstacle = (*it);
          if ( p_obstacle ) {
            for( std::vector< LineSubSegment* >::iterator itbp = p_obstacle->mpBetaSeg->mSubsegs.begin();
                 itbp != p_obstacle->mpBetaSeg->mSubsegs.end(); itbp++ ) {
              LineSubSegment* p_subseg_b = (*itbp);
              b_subseg_painter.drawLine( toQPoint( p_subseg_b->mSubseg.source() ), 
                                         toQPoint( p_subseg_b->mSubseg.target() ));
            }
          }
        }
        b_subseg_painter.end();
      }
  
      QPainter cp_painter(this);
      QPen cp_pen( CENTER_POINT_COLOR );
      cp_pen.setWidth( POINT_SIZE );
      cp_painter.setPen( cp_pen );
      cp_painter.drawPoint( toQPoint( mpMgr->mpWorldmap->getCentralPoint() ) );
      cp_painter.end();

      QPainter bk_painter(this);
      QPen bk_pen( BK_COLOR );
      bk_pen.setWidth( POINT_SIZE );
      bk_painter.setPen( bk_pen );
      for( std::vector<Obstacle*>::iterator it = obstacles.begin();
           it != obstacles.end(); it++ ) {
        Obstacle* p_obstacle = (*it);
        if ( p_obstacle ) {
          bk_painter.drawPoint( toQPoint( p_obstacle->mBk ) );
        }
      }
      bk_painter.end();
  
      QPainter intsec_painter(this);
      QPen intsec_pen( INTERSECTION_COLOR );
      intsec_pen.setWidth( POINT_SIZE );
      intsec_painter.setPen( intsec_pen );
      for( std::vector<Obstacle*>::iterator it = obstacles.begin();
           it != obstacles.end(); it++ ) {
        Obstacle* p_obstacle = (*it);
        if ( p_obstacle ) {
          for( std::vector< IntersectionPoint >::iterator itap = p_obstacle->mAlphaIntersectionPoints.begin();
               itap != p_obstacle->mAlphaIntersectionPoints.end(); itap++ ) {
            IntersectionPoint alpha_intsec = (*itap);
            intsec_painter.drawPoint( toQPoint( alpha_intsec.mPoint ) );
          }
          for( std::vector< IntersectionPoint >::iterator itbp = p_obstacle->mBetaIntersectionPoints.begin();
               itbp != p_obstacle->mBetaIntersectionPoints.end(); itbp++ ) {
            IntersectionPoint beta_intsec = (*itbp);
            intsec_painter.drawPoint( toQPoint( beta_intsec.mPoint ) );
          }
        }
      }
      intsec_painter.end();

      QPainter text_painter(this);
      QPen text_pen( TEXT_COLOR );
      text_painter.setPen(text_pen);
      for( std::vector<Obstacle*>::iterator it = obstacles.begin();
           it != obstacles.end(); it++ ) {
        Obstacle* p_obstacle = (*it);
        if( p_obstacle ) {
          int c_x = (p_obstacle->mPgn.bbox().xmax() + p_obstacle->mPgn.bbox().xmin() )/2;
          int c_y = (p_obstacle->mPgn.bbox().ymax() + p_obstacle->mPgn.bbox().ymin() )/2;
          text_painter.drawText( c_x, c_y, QString::number(p_obstacle->getIndex()) );
        }
      }
      text_painter.end();

      if( mpMgr ) {
        if( mpMgr->mStartX >= 0 && mpMgr->mStartY >= 0 ) {
          QPainter st_painter(this);
          QPen st_paintpen( START_COLOR );
          st_paintpen.setWidth( POINT_SIZE );
          st_painter.setPen( st_paintpen );
          st_painter.drawPoint( QPoint( mpMgr->mStartX, mpMgr->mStartY ) );
          st_painter.end();
        }

        if( mpMgr->mGoalX >= 0 && mpMgr->mGoalY >= 0 ) {
          QPainter gt_painter(this);
          QPen gt_paintpen( GOAL_COLOR );
          gt_paintpen.setWidth( POINT_SIZE );
          gt_painter.setPen( gt_paintpen );
          gt_painter.drawPoint( QPoint( mpMgr->mGoalX, mpMgr->mGoalY ) );
          gt_painter.end();
        }
      }

      QPainter pos_ref_painter(this);
      QPen pos_ref_paintpen( RULE_POS_COLOR );
      pos_ref_paintpen.setWidth( RULE_LINE_WIDTH );
      pos_ref_painter.setPen( pos_ref_paintpen );
      for( vector< pair< ReferenceFrame*, bool > >::iterator it =  mpMgr->mRules.begin();
           it != mpMgr->mRules.end(); it++ ) {
        pair< ReferenceFrame*, bool > rule = (*it);
        if( true == rule.second ) {
          pos_ref_painter.drawLine( toQPoint( rule.first->mSegment.source() ), 
                                    toQPoint( rule.first->mSegment.target() ));
        }
      }
      pos_ref_painter.end();

      QPainter neg_ref_painter(this);
      QPen neg_ref_paintpen( RULE_NEG_COLOR );
      neg_ref_paintpen.setWidth( RULE_LINE_WIDTH );
      neg_ref_painter.setPen( neg_ref_paintpen );
      for( vector< pair< ReferenceFrame*, bool > >::iterator it =  mpMgr->mRules.begin();
           it != mpMgr->mRules.end(); it++ ) {
        pair< ReferenceFrame*, bool > rule = (*it);
        if( false == rule.second ) {
          neg_ref_painter.drawLine( toQPoint( rule.first->mSegment.source() ),
                                    toQPoint( rule.first->mSegment.target() ));
        }
      }
      neg_ref_painter.end();

      if( mpVizStringClass ) {
        QPainter st_cls_painter(this);
        QPen st_cls_paintpen( STRING_CLASS_POINT_COLOR );
        st_cls_paintpen.setWidth( STRING_CLASS_POINT_SIZE );
        st_cls_painter.setPen( st_cls_paintpen ); 

        if( mpVizStringClass->mpReferenceFrames.size() > 0 ) {
          
          st_cls_painter.drawLine( QPoint( mpMgr->mStartX, mpMgr->mStartY ),
                                   toQPoint( mpVizStringClass->mpReferenceFrames[0]->mMidPoint ) );
          for( unsigned int i=0; i < mpVizStringClass->mpReferenceFrames.size()-1; i++ ) {

            ReferenceFrame* p_curr_rf_str_cls = mpVizStringClass->mpReferenceFrames[i];
            ReferenceFrame* p_next_rf_str_cls = mpVizStringClass->mpReferenceFrames[i+1];
            if( p_curr_rf_str_cls && p_next_rf_str_cls ) {
              st_cls_painter.drawLine( toQPoint( p_curr_rf_str_cls->mMidPoint ),
                                       toQPoint( p_next_rf_str_cls->mMidPoint ) );
            }
          }
          st_cls_painter.drawLine( toQPoint( mpVizStringClass->mpReferenceFrames.back()->mMidPoint ),
                                   QPoint( mpMgr->mGoalX, mpMgr->mGoalY ) );
        }
 
        st_cls_painter.end();     
      }

    } 
  }
}

void SpatialInferViz::prevRegion() {
  if( mpMgr->mpWorldmap ) {
    if ( mRegionIdx >= 0 ) {
      mRegionIdx--;
      mSubRegionIdx = 0;
      updateVizSubregions();
      updateVizLineSubsegments();
    }
    else {
      mRegionIdx = static_cast<int>(mpMgr->mpWorldmap->getSubregionSet().size())-1;
      mSubRegionIdx = 0;
      updateVizSubregions();
      updateVizLineSubsegments();
    }
  }
}

void SpatialInferViz::nextRegion() {
  if( mpMgr->mpWorldmap ) {
    if ( mRegionIdx < static_cast<int>(mpMgr->mpWorldmap->getSubregionSet().size())-1 ) {
      mRegionIdx++;
      mSubRegionIdx = 0;
      updateVizSubregions();
      updateVizLineSubsegments();
    }
    else {
      mRegionIdx = -1;
      mSubRegionIdx = 0;
      updateVizSubregions();
      updateVizLineSubsegments();
    }
  }
}

void SpatialInferViz::prevSubregion() {
  if ( mpMgr->mpWorldmap ) {
    if ( mRegionIdx >= 0 && mRegionIdx < static_cast<int>(mpMgr->mpWorldmap->getSubregionSet().size()) ) {
      SubRegionSet* p_subregions = mpMgr->mpWorldmap->getSubregionSet() [mRegionIdx];
      int sub_num = static_cast<int>( p_subregions->mSubregions.size() );
      if ( mSubRegionIdx > 0) {
        mSubRegionIdx --;
        updateVizSubregions();
        updateVizLineSubsegments();
      }
      else{
        mSubRegionIdx = sub_num - 1;
        updateVizSubregions();
        updateVizLineSubsegments();
      }
    }
  }
}

void SpatialInferViz::nextSubregion() {
  if ( mpMgr->mpWorldmap ) {
    if ( mRegionIdx >= 0 && mRegionIdx < static_cast<int>(mpMgr->mpWorldmap->getSubregionSet().size()) ) {
         SubRegionSet* p_subregions = mpMgr->mpWorldmap->getSubregionSet() [mRegionIdx];
      int sub_num = static_cast<int>( p_subregions->mSubregions.size() );
      if ( mSubRegionIdx < sub_num-1) {
        mSubRegionIdx ++;
        updateVizSubregions();
        updateVizLineSubsegments();
      }
      else{
        mSubRegionIdx = 0;
        updateVizSubregions();
        updateVizLineSubsegments();
      }
    }
  }
}

void SpatialInferViz::prevLineSubsegmentSet() {
  if( mpMgr->mpWorldmap ) {
    if ( mSubsegmentSetIdx >= 0 ) {
      mSubsegmentSetIdx--;
      mSubsegmentIdx = 0;
      updateVizSubregions();
      updateVizLineSubsegments();
    }
    else {
      mSubsegmentSetIdx = static_cast<int>(mpMgr->mpWorldmap->getLinesubsegmentSet().size())-1;
      mSubsegmentIdx = 0;
      updateVizSubregions();
      updateVizLineSubsegments();
    }
  }
}

void SpatialInferViz::nextLineSubsegmentSet() {
  if( mpMgr->mpWorldmap ) {
    if ( mSubsegmentSetIdx < static_cast<int>(mpMgr->mpWorldmap->getLinesubsegmentSet().size())-1 ) {
      mSubsegmentSetIdx++;
      mSubsegmentIdx = 0;
      updateVizSubregions();
      updateVizLineSubsegments();
    }
    else {
      mSubsegmentSetIdx = -1;
      mSubsegmentIdx = 0;
      updateVizSubregions();
      updateVizLineSubsegments();
    }
  }
}

void SpatialInferViz::prevStringClass() {
  if( mpMgr ) {
    if( mStringClassIdx >= 0 ) {
      mStringClassIdx --;
      updateVizStringClass();
    }
    else {
      mStringClassIdx = mpMgr->mpStringClasses.size()-1;
      updateVizStringClass();
    }
  }
}

void SpatialInferViz::nextStringClass() {
  if( mpMgr ) {
    if( mStringClassIdx < (signed)mpMgr->mpStringClasses.size()-1 ) {
      mStringClassIdx ++;
      updateVizStringClass();
    }
    else {
      mStringClassIdx = -1;
      updateVizStringClass();
    }
  }
}

void SpatialInferViz::prevLineSubsegment() {
  if ( mpMgr->mpWorldmap ) {
    if ( mSubsegmentSetIdx >= 0 && mSubsegmentSetIdx < static_cast<int>(mpMgr->mpWorldmap->getLinesubsegmentSet().size()) ) {
      LineSubSegmentSet* p_subsegment_set = mpMgr->mpWorldmap->getLinesubsegmentSet() [mSubsegmentSetIdx];
      int sub_num = static_cast<int>( p_subsegment_set->mSubsegs.size() );
      if ( mSubsegmentIdx > 0) {
        mSubsegmentIdx --;
        updateVizSubregions();
        updateVizLineSubsegments();
      }
      else{
        mSubsegmentIdx = sub_num - 1;
        updateVizSubregions();
        updateVizLineSubsegments();
      }
    }
  }
}

void SpatialInferViz::nextLineSubsegment() {
  if ( mpMgr->mpWorldmap ) {
    if ( mSubsegmentSetIdx >= 0 && mSubsegmentSetIdx < static_cast<int>(mpMgr->mpWorldmap->getLinesubsegmentSet().size()) ) {
      LineSubSegmentSet* p_subsegment_set = mpMgr->mpWorldmap->getLinesubsegmentSet() [mSubsegmentSetIdx];
      int sub_num = static_cast<int>( p_subsegment_set->mSubsegs.size() );
      if ( mSubsegmentIdx < sub_num-1) {
        mSubsegmentIdx ++;
        updateVizSubregions();
        updateVizLineSubsegments();
      }
      else{
        mSubsegmentIdx = 0;
        updateVizSubregions();
        updateVizLineSubsegments();
      }
    }
  }
}

bool SpatialInferViz::save( QString filename ) {
  if( mpMgr->mpWorldmap ) {
    mpMgr->mpWorldmap->toXml(filename.toStdString());
    return true;
  }
  return false;
}

bool SpatialInferViz::load( QString filename ) {
  if ( mpMgr->mpWorldmap == NULL) {
    mpMgr->mpWorldmap = new WorldMap();
  }

  mpMgr->mpWorldmap->fromXml(filename.toStdString());

  QPixmap emptyPix( mpMgr->getWorldMap()->getWidth(), mpMgr->getWorldMap()->getHeight() );
  emptyPix.fill(QColor("white"));
  std::cout << " EMPTY PIX " << emptyPix.width() << " * " << emptyPix.height() << std::endl;
  //setPixmap(pix);
  setPixmap(emptyPix);

  mpMgr->getWorldMap()->init(false);
  repaint();

  return true;
}

SubRegionSet* SpatialInferViz::getSelectedRegion() {
  SubRegionSet* p_region = NULL;
  if ( mpMgr->getWorldMap() ) {
    if ( mpMgr->getWorldMap()->getSubregionSet().size() > 0 ) {
      if( mRegionIdx >= 0 && mRegionIdx < (signed)mpMgr->getWorldMap()->getSubregionSet().size() ) {
        return mpMgr->getWorldMap()->getSubregionSet()[ mRegionIdx ];
      }
    }  
  }
  return p_region;
}


SubRegion* SpatialInferViz::getSelectedSubregion() {
  SubRegion* p_subregion = NULL;
  SubRegionSet* p_region = getSelectedRegion(); 
  if( p_region ) {
    if( p_region->mSubregions.size() > 0 ) {
      if( mSubRegionIdx >= 0 && mSubRegionIdx < (signed)p_region->mSubregions.size() ) {
        return p_region->mSubregions[mSubRegionIdx];
      }
    }
  }
  return p_subregion;
}

StringClass* SpatialInferViz::getSelectedStringClass() {
  StringClass* p_string_class = NULL;
  if( mpMgr->mpStringClasses.size() > 0 ) {
    if( mStringClassIdx >= 0 && mStringClassIdx < (signed)mpMgr->mpStringClasses.size() ) {
      return mpMgr->mpStringClasses[mStringClassIdx];
    }
  } 
  return p_string_class;
}

LineSubSegmentSet* SpatialInferViz::getSelectedLineSubsegmentSet() {
  LineSubSegmentSet* p_subseg_set = NULL;
  if ( mpMgr->getWorldMap() ) {
    if ( mpMgr->getWorldMap()->getLinesubsegmentSet().size() > 0 ) {
      if( mSubsegmentSetIdx >= 0 && mSubsegmentSetIdx < (signed)mpMgr->getWorldMap()->getLinesubsegmentSet().size() ) {
        return mpMgr->getWorldMap()->getLinesubsegmentSet()[ mSubsegmentSetIdx ];
      }
    }  
  }
  return p_subseg_set;
}

LineSubSegment* SpatialInferViz::getSelectedLineSubsegment() {
  LineSubSegment* p_subseg= NULL;  
  LineSubSegmentSet* p_subseg_set = getSelectedLineSubsegmentSet(); 
  if( p_subseg_set ) {
    if( p_subseg_set->mSubsegs.size() > 0 ) {
      if( mSubsegmentIdx >= 0 && mSubsegmentIdx < (signed)p_subseg_set->mSubsegs.size() ) {
        return p_subseg_set->mSubsegs[mSubsegmentIdx];
      }
    }
  }
  return p_subseg;
}

void SpatialInferViz::updateVizSubregions() {
  mVizSubregions.clear();
  if( SUBREGION == mMode ) {
    SubRegionSet* p_region = getSelectedRegion();
    if( p_region ) {
      SubRegion* p_subregion = getSelectedSubregion(); 
      if (p_subregion) {
        mVizSubregions.push_back( p_subregion );
      }
      else {
        for( unsigned int i=0; i < p_region->mSubregions.size(); i++ ) {
          SubRegion* p_subregion = p_region->mSubregions[i];
          mVizSubregions.push_back( p_subregion );
        }
      }
    }
  }
  else if( LINE_SUBSEGMENT == mMode ) {
    LineSubSegmentSet* p_subseg_set = getSelectedLineSubsegmentSet();
    if( p_subseg_set ) {
      LineSubSegment* p_subseg = getSelectedLineSubsegment();
      if( p_subseg ) {
        for( unsigned int i=0; i < p_subseg->mNeighbors.size(); i++ ) {
          SubRegion* p_subregion = p_subseg->mNeighbors[i];
          mVizSubregions.push_back( p_subregion );
        }
      }
    }
  }
}

void SpatialInferViz::updateVizLineSubsegments() {
  mVizSubsegments.clear();
  if( SUBREGION == mMode ) {
    SubRegionSet* p_region = getSelectedRegion();
    if( p_region ) {
      SubRegion* p_subregion = getSelectedSubregion(); 
      if (p_subregion) {
        for( unsigned int i=0; i < p_subregion->mNeighbors.size(); i++ ) {
          LineSubSegment* p_subseg = p_subregion->mNeighbors[i];
          if( p_subseg ) {
            mVizSubsegments.push_back( p_subseg );
          }
        }
      }
      else {
        if( p_region->mpLineSegmentsA ){
          for( unsigned int i=0; i < p_region->mpLineSegmentsA->mSubsegs.size(); i++ ) {
            LineSubSegment* p_subseg = p_region->mpLineSegmentsB->mSubsegs[i];
            if( p_subseg ) {
              mVizSubsegments.push_back( p_subseg );
            }
          }
        }

        if( p_region->mpLineSegmentsB ){
          for( unsigned int i=0; i < p_region->mpLineSegmentsB->mSubsegs.size(); i++ ) {
            LineSubSegment* p_subseg = p_region->mpLineSegmentsB->mSubsegs[i];
            if( p_subseg ) {
              mVizSubsegments.push_back( p_subseg );
            }
          }
        }
          
      } 
    }
  }
  else if( LINE_SUBSEGMENT == mMode ) {
    LineSubSegmentSet* p_subseg_set = getSelectedLineSubsegmentSet();
    if( p_subseg_set ) {
      LineSubSegment* p_subseg = getSelectedLineSubsegment();
      if( p_subseg ) {
        mVizSubsegments.push_back( p_subseg ); 
      }
      else {
        for( unsigned int i=0; i < p_subseg_set->mSubsegs.size(); i++ ) {
          LineSubSegment* p_subseg = p_subseg_set->mSubsegs[i];
          if( p_subseg ){
            mVizSubsegments.push_back( p_subseg ); 
          }
        }
      }
    }
  }

}

void SpatialInferViz::updateVizStringClass() {
  mpVizStringClass = getSelectedStringClass();
}

void SpatialInferViz::setMode( SpatialInferVizMode mode ) {
  mMode = mode;
  mRegionIdx = -1;
  mSubRegionIdx = -1;
  mSubsegmentSetIdx = -1;
  mSubsegmentIdx = -1;
  updateVizSubregions();
  updateVizLineSubsegments();
}

void SpatialInferViz::mousePressEvent( QMouseEvent * event ) {
  if( event->button() == Qt::LeftButton ) {
    Point2D clicked_point( event->x(), event->y() );
    Obstacle* p_selected_obstacle = mpReferenceFrameSet->getWorldMap()->findObstacle( clicked_point );
    if( p_selected_obstacle ) {
      if( isSelectedObstacle( p_selected_obstacle ) ) {
        unselectObstacle( p_selected_obstacle );
      }
      else{ 
        mSelectedObstacles.push_back( p_selected_obstacle );
      }
      repaint(); 
    }
  }
}

bool SpatialInferViz::isSelectedObstacle( Obstacle* p_obstacle ) {
  for(unsigned int i=0; i < mSelectedObstacles.size(); i++ ) {
    Obstacle* p_current_obstacle = mSelectedObstacles[i];
    if( p_current_obstacle ) {
      if( p_current_obstacle == p_obstacle ) {
        return true;
      }
    }
  }
  return false;

}
  
bool SpatialInferViz::unselectObstacle( Obstacle* p_obstacle ) {
  for( std::vector< Obstacle* >::iterator it = mSelectedObstacles.begin();
       it != mSelectedObstacles.end(); it ++ ) {
    Obstacle* p_current_obstacle = (*it);
    if( p_current_obstacle ) {
      if( p_current_obstacle == p_obstacle ) {
        mSelectedObstacles.erase(it);
        return true;
      }
    }
  }
  return false;
}

} // topologyinference

} // topologyPathPlanning
