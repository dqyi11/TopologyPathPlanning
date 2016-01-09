#include <cstdlib>
#include <QPainter>
#include <QMouseEvent>
#include "spatialinfer_viz.h"
#include "img_load_util.h"

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


using namespace homotopy;
using namespace topology_inference;

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
  mShowSubsegment = true;
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

  load_map_info( filename.toStdString(), map_width, map_height, conts );   
  //std::cout << "CREATE WORLD " << map_width << " * " << map_height << std::endl;
  mpReferenceFrameSet = new ReferenceFrameSet();
  mpReferenceFrameSet->init( map_width, map_height, conts );
  //std::cout << "NUM OF OBS " << conts.size() << std::endl;
  mpMgr = new SpatialRelationMgr( mpReferenceFrameSet->get_world_map() );
  return true;
}

void SpatialInferViz::paintEvent(QPaintEvent * e) {
    QLabel::paintEvent(e);
  if (mpMgr) {
    if (mpMgr->mp_worldmap) {

      QPainter region_painter(this);
      region_painter.setRenderHint(QPainter::Antialiasing);
      QBrush region_brush( SUBREGION_COLOR );
      region_painter.setPen(Qt::NoPen);
      for( std::vector<SubRegion*>::iterator itr = m_viz_subregions.begin();
           itr != m_viz_subregions.end(); itr++ ) {  
  
        SubRegion* p_subreg = (*itr);
        if (p_subreg) {
          QPolygon poly;
          for( unsigned int j=0; j < p_subreg->m_points.size(); j++ ) {
            double x = CGAL::to_double( p_subreg->m_points[j].x() );
            double y = CGAL::to_double( p_subreg->m_points[j].y() );
            poly << QPoint( x, y );
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
 
      for( std::vector< LineSubSegment* >::iterator itLSS = m_viz_subsegments.begin();
           itLSS != m_viz_subsegments.end(); itLSS++ ) {
        LineSubSegment* p_line_subsegment = (*itLSS);
        if( p_line_subsegment ) {
          Point2D line_hl_src = p_line_subsegment->m_subseg.source();
          Point2D line_hl_end = p_line_subsegment->m_subseg.target();
          double line_hl_src_x = CGAL::to_double( line_hl_src.x() );
          double line_hl_src_y = CGAL::to_double( line_hl_src.y() );
          double line_hl_end_x = CGAL::to_double( line_hl_end.x() );
          double line_hl_end_y = CGAL::to_double( line_hl_end.y() );

          line_hl_painter.drawLine( QPoint( line_hl_src_x, line_hl_src_y ),
                                    QPoint( line_hl_end_x, line_hl_end_y ) );
        }
      }
      line_hl_painter.end();

      std::vector<Obstacle*> obstacles =  mpMgr->mp_worldmap->get_obstacles();
  
      QPainter obstacle_painter(this);
      obstacle_painter.setRenderHint(QPainter::Antialiasing);
      QPen obstacle_pen( OBSTACLE_COLOR );
      obstacle_painter.setPen(obstacle_pen);
      for( std::vector<Obstacle*>::iterator it = obstacles.begin();
           it != obstacles.end(); it++ ) {
        Obstacle* p_obstacle = (*it);
        if (p_obstacle) {
          QPolygon poly;
          for( Polygon2D::Vertex_iterator itP=p_obstacle->m_pgn.vertices_begin();
               itP != p_obstacle->m_pgn.vertices_end(); itP++ ) {
            Point2D p = (*itP);
            double p_x = CGAL::to_double( p.x() );
            double p_y = CGAL::to_double( p.y() );
            poly << QPoint( p_x, p_y );
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
        for( std::vector<Obstacle*>::iterator it = p_subseg->m_connected_obstacles.begin();
             it != p_subseg->m_connected_obstacles.end(); it++ ) {
          Obstacle* p_obstacle = (*it);
          if (p_obstacle) {
            QPolygon poly;
            for( Polygon2D::Vertex_iterator itP=p_obstacle->m_pgn.vertices_begin();
                 itP != p_obstacle->m_pgn.vertices_end(); itP++ ) {
              Point2D p = (*itP);
              double p_x = CGAL::to_double( p.x() );
              double p_y = CGAL::to_double( p.y() );
              poly << QPoint( p_x, p_y );
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
      for( std::vector<Obstacle*>::iterator it = m_selected_obstacles.begin();
           it != m_selected_obstacles.end(); it++ ) {
        Obstacle* p_obstacle = (*it);
        if (p_obstacle) {
          QPolygon poly;
          for( Polygon2D::Vertex_iterator itP=p_obstacle->m_pgn.vertices_begin();
               itP != p_obstacle->m_pgn.vertices_end(); itP++ ) {
            Point2D p = (*itP);
            double p_x = CGAL::to_double( p.x() );
            double p_y = CGAL::to_double( p.y() );
            poly << QPoint( p_x, p_y );
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
            for( std::vector< LineSubSegment* >::iterator itap = p_obstacle->mp_alpha_seg->m_subsegs.begin();
                 itap != p_obstacle->mp_alpha_seg->m_subsegs.end(); itap++ ) {
              LineSubSegment* p_subseg_a = (*itap);
              double a_src_x = CGAL::to_double( p_subseg_a->m_subseg.source().x() );
              double a_src_y = CGAL::to_double( p_subseg_a->m_subseg.source().y() );
              double a_end_x = CGAL::to_double( p_subseg_a->m_subseg.target().x() );
              double a_end_y = CGAL::to_double( p_subseg_a->m_subseg.target().y() );
              a_subseg_painter.drawLine( QPoint( a_src_x , a_src_y ), QPoint( a_end_x , a_end_y ));
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
            for( std::vector< LineSubSegment* >::iterator itbp = p_obstacle->mp_beta_seg->m_subsegs.begin();
                 itbp != p_obstacle->mp_beta_seg->m_subsegs.end(); itbp++ ) {
              LineSubSegment* p_subseg_b = (*itbp);
              double b_src_x = CGAL::to_double( p_subseg_b->m_subseg.source().x() );
              double b_src_y = CGAL::to_double( p_subseg_b->m_subseg.source().y() );
              double b_end_x = CGAL::to_double( p_subseg_b->m_subseg.target().x() );
              double b_end_y = CGAL::to_double( p_subseg_b->m_subseg.target().y() );
              b_subseg_painter.drawLine( QPoint( b_src_x , b_src_y ), QPoint( b_end_x , b_end_y ));
            }
          }
        }
        b_subseg_painter.end();
      }
  
      QPainter cp_painter(this);
      QPen cp_pen( CENTER_POINT_COLOR );
      cp_pen.setWidth( POINT_SIZE );
      cp_painter.setPen( cp_pen );
      double cp_x = CGAL::to_double( mpMgr->mp_worldmap->get_central_point().x() );
      double cp_y = CGAL::to_double( mpMgr->mp_worldmap->get_central_point().y() );
      cp_painter.drawPoint( QPoint( cp_x , cp_y ) );
      cp_painter.end();

      QPainter bk_painter(this);
      QPen bk_pen( BK_COLOR );
      bk_pen.setWidth( POINT_SIZE );
      bk_painter.setPen( bk_pen );
      for( std::vector<Obstacle*>::iterator it = obstacles.begin();
           it != obstacles.end(); it++ ) {
        Obstacle* p_obstacle = (*it);
        if ( p_obstacle ) {
          double bk_x = CGAL::to_double( p_obstacle->m_bk.x() );
          double bk_y = CGAL::to_double( p_obstacle->m_bk.y() );
          bk_painter.drawPoint( QPoint( bk_x , bk_y ) );
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
          for( std::vector< IntersectionPoint >::iterator itap = p_obstacle->m_alpha_intersection_points.begin();
               itap != p_obstacle->m_alpha_intersection_points.end(); itap++ ) {
            IntersectionPoint alpha_intsec = (*itap);
            double alpha_intsec_x = CGAL::to_double( alpha_intsec.m_point.x() );
            double alpha_intsec_y = CGAL::to_double( alpha_intsec.m_point.y() );
            intsec_painter.drawPoint( QPoint( alpha_intsec_x , alpha_intsec_y ) );
          }
          for( std::vector< IntersectionPoint >::iterator itbp = p_obstacle->m_beta_intersection_points.begin();
               itbp != p_obstacle->m_beta_intersection_points.end(); itbp++ ) {
            IntersectionPoint beta_intsec = (*itbp);
            double beta_intsec_x = CGAL::to_double( beta_intsec.m_point.x() );
            double beta_intsec_y = CGAL::to_double( beta_intsec.m_point.y() );
            intsec_painter.drawPoint( QPoint( beta_intsec_x, beta_intsec_y ) );
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
          int c_x = (p_obstacle->m_pgn.bbox().xmax() + p_obstacle->m_pgn.bbox().xmin() )/2;
          int c_y = (p_obstacle->m_pgn.bbox().ymax() + p_obstacle->m_pgn.bbox().ymin() )/2;
          text_painter.drawText( c_x, c_y, QString::number(p_obstacle->get_index()) );
        }
      }
      text_painter.end();

      if( mpMgr ) {
        if( mpMgr->m_start_x >= 0 && mpMgr->m_start_y >= 0 ) {
          QPainter st_painter(this);
          QPen st_paintpen( START_COLOR );
          st_paintpen.setWidth( POINT_SIZE );
          st_painter.setPen( st_paintpen );
          st_painter.drawPoint( QPoint( mpMgr->m_start_x, mpMgr->m_start_y ) );
          st_painter.end();
        }

        if( mpMgr->m_goal_x >= 0 && mpMgr->m_goal_y >= 0 ) {
          QPainter gt_painter(this);
          QPen gt_paintpen( GOAL_COLOR );
          gt_paintpen.setWidth( POINT_SIZE );
          gt_painter.setPen( gt_paintpen );
          gt_painter.drawPoint( QPoint( mpMgr->m_goal_x, mpMgr->m_goal_y ) );
          gt_painter.end();
        }
      }
    }
  }
}

void SpatialInferViz::prevRegion() {
  if( mpMgr->mp_worldmap ) {
    if ( mRegionIdx >= 0 ) {
      mRegionIdx--;
      mSubRegionIdx = 0;
      updateVizSubregions();
      updateVizLineSubsegments();
    }
    else {
      mRegionIdx = static_cast<int>(mpMgr->mp_worldmap->get_subregion_set().size())-1;
      mSubRegionIdx = 0;
      updateVizSubregions();
      updateVizLineSubsegments();
    }
  }
}

void SpatialInferViz::nextRegion() {
  if( mpMgr->mp_worldmap ) {
    if ( mRegionIdx < static_cast<int>(mpMgr->mp_worldmap->get_subregion_set().size())-1 ) {
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
  if ( mpMgr->mp_worldmap ) {
    if ( mRegionIdx >= 0 && mRegionIdx < static_cast<int>(mpMgr->mp_worldmap->get_subregion_set().size()) ) {
      SubRegionSet* p_subregions = mpMgr->mp_worldmap->get_subregion_set() [mRegionIdx];
      int sub_num = static_cast<int>( p_subregions->m_subregions.size() );
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
  if ( mpMgr->mp_worldmap ) {
    if ( mRegionIdx >= 0 && mRegionIdx < static_cast<int>(mpMgr->mp_worldmap->get_subregion_set().size()) ) {
         SubRegionSet* p_subregions = mpMgr->mp_worldmap->get_subregion_set() [mRegionIdx];
      int sub_num = static_cast<int>( p_subregions->m_subregions.size() );
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
  if( mpMgr->mp_worldmap ) {
    if ( mSubsegmentSetIdx >= 0 ) {
      mSubsegmentSetIdx--;
      mSubsegmentIdx = 0;
      updateVizSubregions();
      updateVizLineSubsegments();
    }
    else {
      mSubsegmentSetIdx = static_cast<int>(mpMgr->mp_worldmap->get_linesubsegment_set().size())-1;
      mSubsegmentIdx = 0;
      updateVizSubregions();
      updateVizLineSubsegments();
    }
  }
}

void SpatialInferViz::nextLineSubsegmentSet() {
  if( mpMgr->mp_worldmap ) {
    if ( mSubsegmentSetIdx < static_cast<int>(mpMgr->mp_worldmap->get_linesubsegment_set().size())-1 ) {
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

void SpatialInferViz::prevLineSubsegment() {
  if ( mpMgr->mp_worldmap ) {
    if ( mSubsegmentSetIdx >= 0 && mSubsegmentSetIdx < static_cast<int>(mpMgr->mp_worldmap->get_linesubsegment_set().size()) ) {
      LineSubSegmentSet* p_subsegment_set = mpMgr->mp_worldmap->get_linesubsegment_set() [mSubsegmentSetIdx];
      int sub_num = static_cast<int>( p_subsegment_set->m_subsegs.size() );
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
  if ( mpMgr->mp_worldmap ) {
    if ( mSubsegmentSetIdx >= 0 && mSubsegmentSetIdx < static_cast<int>(mpMgr->mp_worldmap->get_linesubsegment_set().size()) ) {
      LineSubSegmentSet* p_subsegment_set = mpMgr->mp_worldmap->get_linesubsegment_set() [mSubsegmentSetIdx];
      int sub_num = static_cast<int>( p_subsegment_set->m_subsegs.size() );
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
  if( mpMgr->mp_worldmap ) {
    mpMgr->mp_worldmap->to_xml(filename.toStdString());
    return true;
  }
  return false;
}

bool SpatialInferViz::load( QString filename ) {
  if ( mpMgr->mp_worldmap == NULL) {
    mpMgr->mp_worldmap = new WorldMap();
  }

  mpMgr->mp_worldmap->from_xml(filename.toStdString());

  QPixmap emptyPix( mpMgr->mp_worldmap->get_width(), mpMgr->mp_worldmap->get_height() );
  emptyPix.fill(QColor("white"));
  std::cout << " EMPTY PIX " << emptyPix.width() << " * " << emptyPix.height() << std::endl;
  //setPixmap(pix);
  setPixmap(emptyPix);

  mpMgr->mp_worldmap->init(false);
  repaint();

  return true;
}

SubRegionSet* SpatialInferViz::getSelectedRegion() {
  SubRegionSet* p_region = NULL;
  if ( mpMgr->mp_worldmap ) {
    if ( mpMgr->mp_worldmap->get_subregion_set().size() > 0 ) {
      if( mRegionIdx >= 0 && mRegionIdx < mpMgr->mp_worldmap->get_subregion_set().size() ) {
        return mpMgr->mp_worldmap->get_subregion_set()[ mRegionIdx ];
      }
    }  
  }
  return p_region;
}


SubRegion* SpatialInferViz::getSelectedSubregion() {
  SubRegion* p_subregion = NULL;
  SubRegionSet* p_region = getSelectedRegion(); 
  if( p_region ) {
    if( p_region->m_subregions.size() > 0 ) {
      if( mSubRegionIdx >= 0 && mSubRegionIdx < p_region->m_subregions.size() ) {
        return p_region->m_subregions[mSubRegionIdx];
      }
    }
  }
  return p_subregion;
}

LineSubSegmentSet* SpatialInferViz::getSelectedLineSubsegmentSet() {
  LineSubSegmentSet* p_subseg_set = NULL;
  if ( mpMgr->mp_worldmap ) {
    if ( mpMgr->mp_worldmap->get_linesubsegment_set().size() > 0 ) {
      if( mSubsegmentSetIdx >= 0 && mSubsegmentSetIdx < mpMgr->mp_worldmap->get_linesubsegment_set().size() ) {
        return mpMgr->mp_worldmap->get_linesubsegment_set()[ mSubsegmentSetIdx ];
      }
    }  
  }
  return p_subseg_set;
}

LineSubSegment* SpatialInferViz::getSelectedLineSubsegment() {
  LineSubSegment* p_subseg= NULL;  
  LineSubSegmentSet* p_subseg_set = getSelectedLineSubsegmentSet(); 
  if( p_subseg_set ) {
    if( p_subseg_set->m_subsegs.size() > 0 ) {
      if( mSubsegmentIdx >= 0 && mSubsegmentIdx < p_subseg_set->m_subsegs.size() ) {
        return p_subseg_set->m_subsegs[mSubsegmentIdx];
      }
    }
  }
  return p_subseg;
}

void SpatialInferViz::updateVizSubregions() {
  m_viz_subregions.clear();
  if( SUBREGION == mMode ) {
    SubRegionSet* p_region = getSelectedRegion();
    if( p_region ) {
      SubRegion* p_subregion = getSelectedSubregion(); 
      if (p_subregion) {
        m_viz_subregions.push_back( p_subregion );
      }
      else {
        for( unsigned int i=0; i < p_region->m_subregions.size(); i++ ) {
          SubRegion* p_subregion = p_region->m_subregions[i];
          m_viz_subregions.push_back( p_subregion );
        }
      }
    }
  }
  else if( LINE_SUBSEGMENT == mMode ) {
    LineSubSegmentSet* p_subseg_set = getSelectedLineSubsegmentSet();
    if( p_subseg_set ) {
      LineSubSegment* p_subseg = getSelectedLineSubsegment();
      if( p_subseg ) {
        for( unsigned int i=0; i < p_subseg->m_neighbors.size(); i++ ) {
          SubRegion* p_subregion = p_subseg->m_neighbors[i];
          m_viz_subregions.push_back( p_subregion );
        }
      }
    }
  }
}

void SpatialInferViz::updateVizLineSubsegments() {
  m_viz_subsegments.clear();
  if( SUBREGION == mMode ) {
    SubRegionSet* p_region = getSelectedRegion();
    if( p_region ) {
      SubRegion* p_subregion = getSelectedSubregion(); 
      if (p_subregion) {
        for( unsigned int i=0; i < p_subregion->m_neighbors.size(); i++ ) {
          LineSubSegment* p_subseg = p_subregion->m_neighbors[i];
          if( p_subseg ) {
            m_viz_subsegments.push_back( p_subseg );
          }
        }
      }
      else {
        if( p_region->mp_line_segments_a ){
          for( unsigned int i=0; i < p_region->mp_line_segments_a->m_subsegs.size(); i++ ) {
            LineSubSegment* p_subseg = p_region->mp_line_segments_b->m_subsegs[i];
            if( p_subseg ) {
              m_viz_subsegments.push_back( p_subseg );
            }
          }
        }

        if( p_region->mp_line_segments_b ){
          for( unsigned int i=0; i < p_region->mp_line_segments_b->m_subsegs.size(); i++ ) {
            LineSubSegment* p_subseg = p_region->mp_line_segments_b->m_subsegs[i];
            if( p_subseg ) {
              m_viz_subsegments.push_back( p_subseg );
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
        m_viz_subsegments.push_back( p_subseg ); 
      }
      else {
        for( unsigned int i=0; i < p_subseg_set->m_subsegs.size(); i++ ) {
          LineSubSegment* p_subseg = p_subseg_set->m_subsegs[i];
          if( p_subseg ){
            m_viz_subsegments.push_back( p_subseg ); 
          }
        }
      }
    }
  }

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
    Obstacle* p_selected_obstacle = mpReferenceFrameSet->get_world_map()->find_obstacle( clicked_point );
    if( p_selected_obstacle ) {
      if( is_selected_obstacle( p_selected_obstacle ) ) {
        unselect_obstacle( p_selected_obstacle );
      }
      else{ 
        m_selected_obstacles.push_back( p_selected_obstacle );
      }
      repaint(); 
    }
  }
}

bool SpatialInferViz::is_selected_obstacle( Obstacle* p_obstacle ) {
  for(unsigned int i=0; i < m_selected_obstacles.size(); i++ ) {
    Obstacle* p_current_obstacle = m_selected_obstacles[i];
    if( p_current_obstacle ) {
      if( p_current_obstacle == p_obstacle ) {
        return true;
      }
    }
  }
  return false;

}
  
bool SpatialInferViz::unselect_obstacle( Obstacle* p_obstacle ) {
  for( std::vector< Obstacle* >::iterator it = m_selected_obstacles.begin();
       it != m_selected_obstacles.end(); it ++ ) {
    Obstacle* p_current_obstacle = (*it);
    if( p_current_obstacle ) {
      if( p_current_obstacle == p_obstacle ) {
        m_selected_obstacles.erase(it);
        return true;
      }
    }
  }
  return false;
}
