#ifndef SPATIALINFER_VIZ_H
#define SPATIALINFER_VIZ_H

#include <vector>
#include <QLabel>
#include "worldmap.h"
#include "reference_frames.h"

namespace homotopy {
  
  enum SpatialInferVizMode {
    SUBREGION,
    LINE_SUBSEGMENT 
  };

  class SpatialInferViz : public QLabel {
    Q_OBJECT
  public:
    explicit SpatialInferViz( QWidget *parent = 0 );
    bool loadMap( QString filename );

    bool mShowSubsegment;
    void prevRegion();
    void nextRegion();

    void prevSubregion();
    void nextSubregion();

    int  getRegionIdx()    { return mRegionIdx; }
    int  getSubregionIdx() { return mSubRegionIdx; }

    SubRegionSet* getSelectedRegion();
    SubRegion* getSelectedSubregion();

    void prevLineSubsegmentSet();
    void nextLineSubsegmentSet();

    void prevLineSubsegment();
    void nextLineSubsegment();
    
    int  getLineSubsegmentSetIdx() { return mSubsegmentSetIdx; }
    int  getLineSubsegmentIdx() { return mSubsegmentIdx; } 

    LineSubSegmentSet* getSelectedLineSubsegmentSet();
    LineSubSegment*    getSelectedLineSubsegment();
    

    bool save( QString filename );
    bool load( QString filename );
    
    QString generate_string();

    void setMode( SpatialInferVizMode mode );
    SpatialInferVizMode getMode() { return mMode; }

  protected:
    void mousePressEvent( QMouseEvent * event );
    void mouseMoveEvent( QMouseEvent * event );
    void mouseReleaseEvent( QMouseEvent * event );
    bool initWorld(QString filename);

    void updateVizSubregions();
    void updateVizLineSubsegments();

    WorldMap*            mpWorld;
    ReferenceFrameSet*   mpReferenceFrameSet;
    int                  mWorldWidth;
    int                  mWorldHeight;
    std::vector<QColor>  mColors;
    std::vector<QPoint>  mPoints;
    bool                 mDragging;
    SpatialInferVizMode      mMode;

    int                  mRegionIdx;
    int                  mSubRegionIdx;
    
    int                  mSubsegmentSetIdx;
    int                  mSubsegmentIdx;

    std::vector<SubRegion*>      m_viz_subregions;
    std::vector<LineSubSegment*> m_viz_subsegments;
  signals:

  public slots:

  private slots:
    void paintEvent(QPaintEvent * e);
  };

}
#endif // SPATIALINFER_VIZ_H
