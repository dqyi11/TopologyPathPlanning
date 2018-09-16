#ifndef TOPOLOGYPATHPLANNING_HOMOTOPYVIZ_HPP
#define TOPOLOGYPATHPLANNING_HOMOTOPYVIZ_HPP

#include <vector>
#include <QLabel>
#include "topologyPathPlanning/homotopy/Worldmap.hpp"
#include "topologyPathPlanning/homotopy/ReferenceFrames.hpp"

namespace topologyPathPlanning {

namespace homotopy {
  
  enum HomotopyVizMode {
    SUBREGION,
    LINE_SUBSEGMENT 
  };

  class HomotopyViz : public QLabel {
    Q_OBJECT
  public:
    explicit HomotopyViz( QWidget *parent = 0 );
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

    void setMode( HomotopyVizMode mode );
    HomotopyVizMode getMode() { return mMode; }

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
    std::vector<QPoint>  mPoints;
    bool                 mDragging;
    HomotopyVizMode      mMode;

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

} // homotopy

} // topologyPathPlanning

#endif // TOPOLOGYPATHPLANNING_HOMOTOPYVIZ_HPP
