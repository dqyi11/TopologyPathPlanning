#ifndef SPATIALINFER_VIZ_H
#define SPATIALINFER_VIZ_H

#include <vector>
#include <QLabel>
#include "tpp/homotopy/reference_frames.h"
#include "tpp/spatial_infer/spatial_relation_mgr.h"

namespace topology_inference {
  
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
    
    SpatialRelationMgr* get_spatial_relation_mgr() { return mpMgr; }
    bool is_selected_obstacle( homotopy::Obstacle* p_obstacle );
    bool unselect_obstacle( homotopy::Obstacle* p_obstacle );

    std::vector<homotopy::Obstacle*> get_selected_obstacles() { return m_selected_obstacles; }
    void clear_selected_obstacles() { m_selected_obstacles.clear(); }  

    homotopy::ReferenceFrameSet* get_reference_frame_set() {  return mpReferenceFrameSet; }

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
 
    std::vector<homotopy::SubRegion*>      m_viz_subregions;
    std::vector<homotopy::LineSubSegment*> m_viz_subsegments;

    std::vector<homotopy::Obstacle*>       m_selected_obstacles;   

    StringClass*                           mp_viz_string_class;

  signals:

  public slots:

  private slots:
    void paintEvent(QPaintEvent * e);
  };

}
#endif // SPATIALINFER_VIZ_H
