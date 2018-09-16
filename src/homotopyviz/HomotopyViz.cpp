#include <cstdlib>
#include <QPainter>
#include <QMouseEvent>
#include "topologyPathPlanning/homotopy/ImgLoadUtil.hpp"
#include "topologyPathPlanning/homotopyviz/HomotopyViz.hpp"

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
#define SELECTED_OBSTACLE_COLOR QColor(255,0,0)
#define LINE_HIGHLIGHTED_COLOR  QColor(204,204,0)
#define DRAWING_LINE_COLOR      QColor(153,76,0)
#define SUBREGION_COLOR         QColor(204,229,255)

namespace topologyPathPlanning {

namespace homotopy {

HomotopyViz::HomotopyViz(QWidget *parent) :
    QLabel(parent) {

  mpWorld = NULL;
  mpReferenceFrameSet = NULL;
  mWorldWidth = 0;
  mWorldHeight = 0;
  mRegionIdx = -1;
  mSubRegionIdx = -1;
  mSubsegmentSetIdx = -1;
  mSubsegmentIdx = -1;
  mShowSubsegment = true;
  mDragging = false;
  mMode = SUBREGION;
}

bool HomotopyViz::loadMap( QString filename ) {

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

bool HomotopyViz::initWorld(QString filename) {

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
  mpWorld = mpReferenceFrameSet->getWorldMap();
  return true;
}

void HomotopyViz::paintEvent(QPaintEvent * e) {
    QLabel::paintEvent(e);

  if (mpWorld) {

    QPainter region_painter(this);
    region_painter.setRenderHint(QPainter::Antialiasing);
    QBrush region_brush( SUBREGION_COLOR );
    region_painter.setPen(Qt::NoPen);
    for( std::vector<SubRegion*>::iterator itr = m_viz_subregions.begin();
         itr != m_viz_subregions.end(); itr++ ) {  
  
      SubRegion* p_subreg = (*itr);
      if (p_subreg) {
        QPolygon poly;
        for( unsigned int j=0; j < p_subreg->mPoints.size(); j++ ) {
          double x = CGAL::to_double( p_subreg->mPoints[j].x() );
          double y = CGAL::to_double( p_subreg->mPoints[j].y() );
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
        Point2D line_hl_src = p_line_subsegment->mSubseg.source();
        Point2D line_hl_end = p_line_subsegment->mSubseg.target();
        double line_hl_src_x = CGAL::to_double( line_hl_src.x() );
        double line_hl_src_y = CGAL::to_double( line_hl_src.y() );
        double line_hl_end_x = CGAL::to_double( line_hl_end.x() );
        double line_hl_end_y = CGAL::to_double( line_hl_end.y() );

        line_hl_painter.drawLine( QPoint( line_hl_src_x, line_hl_src_y ),
                                  QPoint( line_hl_end_x, line_hl_end_y ) );
      }
    }
    line_hl_painter.end();

    std::vector<Obstacle*> obstacles =  mpWorld->getObstacles();

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
    QPen hl_obs_pen( SELECTED_OBSTACLE_COLOR );
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
            double p_x = CGAL::to_double( p.x() );
            double p_y = CGAL::to_double( p.y() );
            poly << QPoint( p_x, p_y );
          }
          hl_obs_painter.drawPolygon(poly);
        }
      }
    }
    obstacle_painter.end();

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
            double a_src_x = CGAL::to_double( p_subseg_a->mSubseg.source().x() );
            double a_src_y = CGAL::to_double( p_subseg_a->mSubseg.source().y() );
            double a_end_x = CGAL::to_double( p_subseg_a->mSubseg.target().x() );
            double a_end_y = CGAL::to_double( p_subseg_a->mSubseg.target().y() );
            //std::cout << p_subseg_a << std::endl;
            //std::cout << p_subseg_a->get_name() << " (" << a_src_x << "," << a_src_y << ") (" << a_end_x << "," << a_end_y << ")" << std::endl;
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
          //std::cout << "OBS " << p_obstacle->get_index() << " BETA:" << p_obstacle->mp_beta_seg->m_subsegs.size() << std::endl;
          for( std::vector< LineSubSegment* >::iterator itbp = p_obstacle->mpBetaSeg->mSubsegs.begin();
               itbp != p_obstacle->mpBetaSeg->mSubsegs.end(); itbp++ ) {
            LineSubSegment* p_subseg_b = (*itbp);
            double b_src_x = CGAL::to_double( p_subseg_b->mSubseg.source().x() );
            double b_src_y = CGAL::to_double( p_subseg_b->mSubseg.source().y() );
            double b_end_x = CGAL::to_double( p_subseg_b->mSubseg.target().x() );
            double b_end_y = CGAL::to_double( p_subseg_b->mSubseg.target().y() );
            //std::cout << p_subseg_b << std::endl;
            //std::cout << p_subseg_b->get_name() << " (" << b_src_x << "," << b_src_y << ") (" << b_end_x << "," << b_end_y << ")" << std::endl;
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
    double cp_x = CGAL::to_double( mpWorld->getCentralPoint().x() );
    double cp_y = CGAL::to_double( mpWorld->getCentralPoint().y() );
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
        double bk_x = CGAL::to_double( p_obstacle->mBk.x() );
        double bk_y = CGAL::to_double( p_obstacle->mBk.y() );
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
        for( std::vector< IntersectionPoint >::iterator itap = p_obstacle->mAlphaIntersectionPoints.begin();
             itap != p_obstacle->mAlphaIntersectionPoints.end(); itap++ ) {
          IntersectionPoint alpha_intsec = (*itap);
          double alpha_intsec_x = CGAL::to_double( alpha_intsec.mPoint.x() );
          double alpha_intsec_y = CGAL::to_double( alpha_intsec.mPoint.y() );
          intsec_painter.drawPoint( QPoint( alpha_intsec_x , alpha_intsec_y ) );
        }
        for( std::vector< IntersectionPoint >::iterator itbp = p_obstacle->mBetaIntersectionPoints.begin();
             itbp != p_obstacle->mBetaIntersectionPoints.end(); itbp++ ) {
          IntersectionPoint beta_intsec = (*itbp);
          double beta_intsec_x = CGAL::to_double( beta_intsec.mPoint.x() );
          double beta_intsec_y = CGAL::to_double( beta_intsec.mPoint.y() );
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
        int c_x = (p_obstacle->mPgn.bbox().xmax() + p_obstacle->mPgn.bbox().xmin() )/2;
        int c_y = (p_obstacle->mPgn.bbox().ymax() + p_obstacle->mPgn.bbox().ymin() )/2;
        text_painter.drawText( c_x, c_y, QString::number(p_obstacle->getIndex()) );
      }
    }
    text_painter.end();

    QPainter draw_line_painter(this);
    QPen draw_line_pen( DRAWING_LINE_COLOR );
    draw_line_pen.setWidth( LINE_WIDTH );
    draw_line_painter.setPen(draw_line_pen);
    if( mPoints.size() > 1 ) {
      for( unsigned int pi = 0; pi < mPoints.size()-1 ; pi ++ ) {
        draw_line_painter.drawLine( mPoints[pi], mPoints[pi+1] );    
      }
    }
    draw_line_painter.end();
  }
}

void HomotopyViz::prevRegion() {
  if( mpWorld ) {
    if ( mRegionIdx >= 0 ) {
      mRegionIdx--;
      mSubRegionIdx = 0;
      updateVizSubregions();
      updateVizLineSubsegments();
    }
    else {
      mRegionIdx = static_cast<int>(mpWorld->getSubregionSet().size())-1;
      mSubRegionIdx = 0;
      updateVizSubregions();
      updateVizLineSubsegments();
    }
  }
}

void HomotopyViz::nextRegion() {
  if( mpWorld ) {
    if ( mRegionIdx < static_cast<int>(mpWorld->getSubregionSet().size())-1 ) {
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

void HomotopyViz::prevSubregion() {
  if ( mpWorld ) {
    if ( mRegionIdx >= 0 && mRegionIdx < static_cast<int>(mpWorld->getSubregionSet().size()) ) {
      SubRegionSet* p_subregions = mpWorld->getSubregionSet() [mRegionIdx];
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

void HomotopyViz::nextSubregion() {
  if ( mpWorld ) {
    if ( mRegionIdx >= 0 && mRegionIdx < static_cast<int>(mpWorld->getSubregionSet().size()) ) {
         SubRegionSet* p_subregions = mpWorld->getSubregionSet() [mRegionIdx];
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

void HomotopyViz::prevLineSubsegmentSet() {
  if( mpWorld ) {
    if ( mSubsegmentSetIdx >= 0 ) {
      mSubsegmentSetIdx--;
      mSubsegmentIdx = 0;
      updateVizSubregions();
      updateVizLineSubsegments();
    }
    else {
      mSubsegmentSetIdx = static_cast<int>(mpWorld->getLinesubsegmentSet().size())-1;
      mSubsegmentIdx = 0;
      updateVizSubregions();
      updateVizLineSubsegments();
    }
  }
}

void HomotopyViz::nextLineSubsegmentSet() {
  if( mpWorld ) {
    if ( mSubsegmentSetIdx < static_cast<int>(mpWorld->getLinesubsegmentSet().size())-1 ) {
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

void HomotopyViz::prevLineSubsegment() {
  if ( mpWorld ) {
    if ( mSubsegmentSetIdx >= 0 && mSubsegmentSetIdx < static_cast<int>(mpWorld->getLinesubsegmentSet().size()) ) {
      LineSubSegmentSet* p_subsegment_set = mpWorld->getLinesubsegmentSet() [mSubsegmentSetIdx];
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

void HomotopyViz::nextLineSubsegment() {
  if ( mpWorld ) {
    if ( mSubsegmentSetIdx >= 0 && mSubsegmentSetIdx < static_cast<int>(mpWorld->getLinesubsegmentSet().size()) ) {
      LineSubSegmentSet* p_subsegment_set = mpWorld->getLinesubsegmentSet() [mSubsegmentSetIdx];
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

bool HomotopyViz::save( QString filename ) {
  if( mpWorld ) {
    mpWorld->toXml(filename.toStdString());
    return true;
  }
  return false;
}

bool HomotopyViz::load( QString filename ) {
  if ( mpWorld == NULL) {
    mpWorld = new WorldMap();
  }

  mpWorld->fromXml(filename.toStdString());

  QPixmap emptyPix( mpWorld->getWidth(), mpWorld->getHeight() );
  emptyPix.fill(QColor("white"));
  std::cout << " EMPTY PIX " << emptyPix.width() << " * " << emptyPix.height() << std::endl;
  //setPixmap(pix);
  setPixmap(emptyPix);

  mpWorld->init(false);
  repaint();

  return true;
}

void HomotopyViz::mousePressEvent( QMouseEvent * event ) {
  //std::cout << "mousePressEvent" << std::endl;
  if ( event->button() == Qt::LeftButton ) {
    mDragging = true;
    mPoints.clear();    
  }
}

void HomotopyViz::mouseMoveEvent( QMouseEvent * event ) {
  //std::cout << "mouseMoveEvent" << mPoints.size() << std::endl;
  if ( mDragging == true ) {
    //std::cout << event->x() << " " << event->y() << std::endl;
    QPoint new_point( event->x(), event->y() );
    if( mPoints.size() > 0 ) {
      QPoint last_point = mPoints.back();
      if( std::abs( new_point.x() - last_point.x() ) > 1 &&
          std::abs( new_point.y() - last_point.y() ) > 1 ) {
        mPoints.push_back( new_point );
      }
    }
    else {
      mPoints.push_back( new_point );
    }
    repaint();
  }
}

void HomotopyViz::mouseReleaseEvent( QMouseEvent * event ){
  //std::cout << "mouseReleaseEvent" << std::endl;
  if ( event->button() == Qt::LeftButton ) {
    mDragging = false;
  }
}

QString HomotopyViz::generate_string() {

  QString ref_str = "";
  /*
  std::vector< Point2D > cgal_points;
  for( unsigned int i = 0; i < mPoints.size(); i ++ ) {
    Point2D p( mPoints[i].x(), mPoints[i].y() );
    cgal_points.push_back( p );
  }
  std::vector< std::string > refs = mpReferenceFrameSet->get_string( cgal_points, STRING_GRAMMAR_TYPE );
  */
  homotopy::PointSequence path;
  for( unsigned int i = 0; i < mPoints.size(); i ++ ) {
    path.addPoint( mPoints[i].x(), mPoints[i].y() );
  }
  std::vector< std::string > refs = mpReferenceFrameSet->getString( path, STRING_GRAMMAR_TYPE );

  for( unsigned int i = 0; i < refs.size(); i ++ ) {
    if ( i > 0 ) {
      ref_str += "  ";
    }
    ref_str += QString::fromStdString( refs[i] );
  } 
  return ref_str;
}

SubRegionSet* HomotopyViz::getSelectedRegion() {
  SubRegionSet* p_region = NULL;
  if ( mpWorld ) {
    if ( mpWorld->getSubregionSet().size() > 0 ) {
      if( mRegionIdx >= 0 && mRegionIdx < mpWorld->getSubregionSet().size() ) {
        return mpWorld->getSubregionSet()[ mRegionIdx ];
      }
    }  
  }
  return p_region;
}


SubRegion* HomotopyViz::getSelectedSubregion() {
  SubRegion* p_subregion = NULL;
  SubRegionSet* p_region = getSelectedRegion(); 
  if( p_region ) {
    if( p_region->mSubregions.size() > 0 ) {
      if( mSubRegionIdx >= 0 && mSubRegionIdx < p_region->mSubregions.size() ) {
        return p_region->mSubregions[mSubRegionIdx];
      }
    }
  }
  return p_subregion;
}

LineSubSegmentSet* HomotopyViz::getSelectedLineSubsegmentSet() {
  LineSubSegmentSet* p_subseg_set = NULL;
  if ( mpWorld ) {
    if ( mpWorld->getLinesubsegmentSet().size() > 0 ) {
      if( mSubsegmentSetIdx >= 0 && mSubsegmentSetIdx < mpWorld->getLinesubsegmentSet().size() ) {
        return mpWorld->getLinesubsegmentSet()[ mSubsegmentSetIdx ];
      }
    }  
  }
  return p_subseg_set;
}

LineSubSegment* HomotopyViz::getSelectedLineSubsegment() {
  LineSubSegment* p_subseg= NULL;  
  LineSubSegmentSet* p_subseg_set = getSelectedLineSubsegmentSet(); 
  if( p_subseg_set ) {
    if( p_subseg_set->mSubsegs.size() > 0 ) {
      if( mSubsegmentIdx >= 0 && mSubsegmentIdx < p_subseg_set->mSubsegs.size() ) {
        return p_subseg_set->mSubsegs[mSubsegmentIdx];
      }
    }
  }
  return p_subseg;
}

void HomotopyViz::updateVizSubregions() {
  m_viz_subregions.clear();
  if( SUBREGION == mMode ) {
    SubRegionSet* p_region = getSelectedRegion();
    if( p_region ) {
      SubRegion* p_subregion = getSelectedSubregion(); 
      if (p_subregion) {
        m_viz_subregions.push_back( p_subregion );
      }
      else {
        for( unsigned int i=0; i < p_region->mSubregions.size(); i++ ) {
          SubRegion* p_subregion = p_region->mSubregions[i];
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
        for( unsigned int i=0; i < p_subseg->mNeighbors.size(); i++ ) {
          SubRegion* p_subregion = p_subseg->mNeighbors[i];
          m_viz_subregions.push_back( p_subregion );
        }
      }
    }
  }
}

void HomotopyViz::updateVizLineSubsegments() {
  m_viz_subsegments.clear();
  if( SUBREGION == mMode ) {
    SubRegionSet* p_region = getSelectedRegion();
    if( p_region ) {
      SubRegion* p_subregion = getSelectedSubregion(); 
      if (p_subregion) {
        for( unsigned int i=0; i < p_subregion->mNeighbors.size(); i++ ) {
          LineSubSegment* p_subseg = p_subregion->mNeighbors[i];
          if( p_subseg ) {
            m_viz_subsegments.push_back( p_subseg );
          }
        }
      }
      else {
        if( p_region->mpLineSegmentsA ){
          for( unsigned int i=0; i < p_region->mpLineSegmentsA->mSubsegs.size(); i++ ) {
            LineSubSegment* p_subseg = p_region->mpLineSegmentsB->mSubsegs[i];
            if( p_subseg ) {
              m_viz_subsegments.push_back( p_subseg );
            }
          }
        }

        if( p_region->mpLineSegmentsB ){
          for( unsigned int i=0; i < p_region->mpLineSegmentsB->mSubsegs.size(); i++ ) {
            LineSubSegment* p_subseg = p_region->mpLineSegmentsB->mSubsegs[i];
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
        for( unsigned int i=0; i < p_subseg_set->mSubsegs.size(); i++ ) {
          LineSubSegment* p_subseg = p_subseg_set->mSubsegs[i];
          if( p_subseg ){
            m_viz_subsegments.push_back( p_subseg ); 
          }
        }
      }
    }
  }

}

void HomotopyViz::setMode( HomotopyVizMode mode ) {
  mMode = mode;
  mRegionIdx = -1;
  mSubRegionIdx = -1;
  mSubsegmentSetIdx = -1;
  mSubsegmentIdx = -1;
  updateVizSubregions();
  updateVizLineSubsegments();
}

} // homotopy

} // topologyPathPlanning
