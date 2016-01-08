#ifndef SPATIALINFER_VIZ_H
#define SPATIALINFER_VIZ_H

#include <vector>
#include <QLabel>
#include "spatial_relation_mgr.h"
#include "reference_frames.h"

namespace topology_inference {
  
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

    homotopy::SubRegionSet* getSelectedRegion();
    homotopy::SubRegion* getSelectedSubregion();

    void prevLineSubsegmentSet();
    void nextLineSubsegmentSet();

    void prevLineSubsegment();
    void nextLineSubsegment();
    
    int  getLineSubsegmentSetIdx() { return mSubsegmentSetIdx; }
    int  getLineSubsegmentIdx() { return mSubsegmentIdx; } 

    homotopy::LineSubSegmentSet* getSelectedLineSubsegmentSet();
    homotopy::LineSubSegment*    getSelectedLineSubsegment();
    

    bool save( QString filename );
    bool load( QString filename );
    
    void setMode( SpatialInferVizMode mode );
    SpatialInferVizMode getMode() { return mMode; }
    
    SpatialRelationMgr* get_spatial_relation_mgr() { return mpMgr; }

  protected:
    bool initWorld(QString filename);

    void updateVizSubregions();
    void updateVizLineSubsegments();

    SpatialRelationMgr*           mpMgr;
    homotopy::ReferenceFrameSet*  mpReferenceFrameSet;
    int                  mWorldWidth;
    int                  mWorldHeight;
    
    SpatialInferVizMode  mMode;

    int                  mRegionIdx;
    int                  mSubRegionIdx;
    
    int                  mSubsegmentSetIdx;
    int                  mSubsegmentIdx;

    std::vector<homotopy::SubRegion*>      m_viz_subregions;
    std::vector<homotopy::LineSubSegment*> m_viz_subsegments;
  signals:

  public slots:

  private slots:
    void paintEvent(QPaintEvent * e);
  };

}
#endif // SPATIALINFER_VIZ_H
