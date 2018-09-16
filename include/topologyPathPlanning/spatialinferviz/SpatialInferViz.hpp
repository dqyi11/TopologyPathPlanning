#ifndef TOPOLOGYPATHPLANNING_SPATIALINFER_SPATIALINFERVIZ_HPP
#define TOPOLOGYPATHPLANNING_SPATIALINFER_SPATIALINFERVIZ_HPP

#include <vector>
#include <QLabel>
#include "topologyPathPlanning/homotopy/ReferenceFrames.hpp"
#include "topologyPathPlanning/spatialinfer/SpatialRelationMgr.hpp"

namespace topologyPathPlanning {

namespace topologyinference {
  
  enum SpatialInferVizMode {
    SUBREGION,
    LINE_SUBSEGMENT,
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

    homotopy::SubRegionSet* getSelectedRegion();
    homotopy::SubRegion* getSelectedSubregion();

    void prevLineSubsegmentSet();
    void nextLineSubsegmentSet();

    void prevLineSubsegment();
    void nextLineSubsegment();

    void prevStringClass();
    void nextStringClass();
    
    int  getLineSubsegmentSetIdx() { return mSubsegmentSetIdx; }
    int  getLineSubsegmentIdx() { return mSubsegmentIdx; } 

    homotopy::LineSubSegmentSet* getSelectedLineSubsegmentSet();
    homotopy::LineSubSegment*    getSelectedLineSubsegment();
    StringClass*                 getSelectedStringClass();  

    bool save( QString filename );
    bool load( QString filename );
    
    void setMode( SpatialInferVizMode mode );
    SpatialInferVizMode getMode() { return mMode; }
    
    SpatialRelationMgr* getSpatialRelationMgr() { return mpMgr; }
    bool isSelectedObstacle( homotopy::Obstacle* p_obstacle );
    bool unselectObstacle( homotopy::Obstacle* p_obstacle );

    std::vector<homotopy::Obstacle*> getSelectedObstacles() { return mSelectedObstacles; }
    void clearSelectedObstacles() { mSelectedObstacles.clear(); }

    homotopy::ReferenceFrameSet* getReferenceFrameSet() {  return mpReferenceFrameSet; }

  protected:
    bool initWorld(QString filename);

    void updateVizSubregions();
    void updateVizLineSubsegments();
    void updateVizStringClass();

    void mousePressEvent( QMouseEvent * event );

    SpatialRelationMgr*           mpMgr;
    homotopy::ReferenceFrameSet*  mpReferenceFrameSet;
    int                  mWorldWidth;
    int                  mWorldHeight;
    
    SpatialInferVizMode  mMode;

    int                  mRegionIdx;
    int                  mSubRegionIdx;
    
    int                  mSubsegmentSetIdx;
    int                  mSubsegmentIdx;

    int                  mStringClassIdx;
 
    std::vector<homotopy::SubRegion*>      mVizSubregions;
    std::vector<homotopy::LineSubSegment*> mVizSubsegments;

    std::vector<homotopy::Obstacle*>       mSelectedObstacles;

    StringClass*                           mpVizStringClass;

  signals:

  public slots:

  private slots:
    void paintEvent(QPaintEvent * e);
  };

} // topologyinference

} // topologyPathPlanning

#endif // TOPOLOGYPATHPLANNING_SPATIALINFER_SPATIALINFERVIZ_HPP
