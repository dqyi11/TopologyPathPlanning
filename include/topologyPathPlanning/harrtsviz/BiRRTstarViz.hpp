#ifndef TOPOLOGYPATHPLANNING_HARRTS_BIRRTSTAR_VIZ_HPP
#define TOPOLOGYPATHPLANNING_HARRTS_BIRRTSTAR_VIZ_HPP

#include <QLabel>

#include "topologyPathPlanning/homotopy/ReferenceFrames.hpp"
#include "topologyPathPlanning/harrts/BiRRTstar.hpp"
#include "topologyPathPlanning/harrtsviz/BiRRTstarPathPlanningInfo.hpp"

namespace topologyPathPlanning {

namespace harrts {
  typedef enum{
    NONE_TREE_SHOW,
    START_TREE_SHOW,
    GOAL_TREE_SHOW,
    BOTH_TREES_SHOW
  } tree_show_type_t;

  class BIRRTstarViz : public QLabel
  {
    Q_OBJECT
  public:
    explicit BIRRTstarViz(QWidget *parent = 0);
    void setTree(BIRRTstar* p_tree);
    void setReferenceFrameSet(homotopy::ReferenceFrameSet* p_rf);
    bool drawPath(QString filename);
    bool saveCurrentViz(QString filename);

    void setShowReferenceFrames( bool show );
    void setShowRegions( bool show );
    void setFinishedPlanning( bool finished ) { mFinishedPlanning = finished; }
    bool getFinishedPlanning() { return mFinishedPlanning; }

    bool showReferenceFrames() { return mShowReferenceFrames; }
    bool showRegions() { return mShowRegions; }
    bool isFinishedPlanning() { return mFinishedPlanning; }
 
    homotopy::ReferenceFrame* getSelectedReferenceFrame();
    homotopy::SubRegionSet*   getSelectedSubregionSet();
    homotopy::SubRegion*      getSelectedSubregion();

    void reset();

    void prevRegion();
    void nextRegion();
    void prevSubregion();
    void nextSubregion();

    void prevReferenceFrame();
    void nextReferenceFrame();
    void prevFoundPath();
    void nextFoundPath();
    int  getReferenceFrameIndex() { return mReferenceFrameIndex; }
    std::string getReferenceFrameName();
    std::string getRegionName();

    void importStringConstraint( std::vector< QPoint > points, homotopy::grammar_type_t type );
    QString generateString();

    tree_show_type_t getTreeShowType() { return mTreeShowType; }
    void switchTreeShowType();
    
    BIRRTstarPathPlanningInfo m_PPInfo;
    
    std::vector<QPoint>& getDrawedPoints() { return mDrawedPoints; }
    void setShowDrawedPoints( bool show ) { mShowPoints = show; }
    bool getShowDrawedPoints() { return mShowPoints; }

    int getFoundPathIndex() { return mFoundPathIndex; }
  signals:
    
  public slots:

  protected:
    void mousePressEvent( QMouseEvent * event );
    void mouseMoveEvent( QMouseEvent * event );
    void mouseReleaseEvent( QMouseEvent * event );
    
    std::vector<QPoint>  mDrawedPoints;
    bool                 mDragging;
    bool                 mShowPoints;

  private:
    void drawPathOnMap(QPixmap& map);
    void drawCurrentViz(QPixmap& map);

    void paint(QPaintDevice * device);
    BIRRTstar*          mpTree;
    homotopy::ReferenceFrameSet*  mpReferenceFrames;
    bool                          mShowReferenceFrames;
    bool                          mShowRegions;
    bool                          mFinishedPlanning;
    int                           mReferenceFrameIndex;
    int                           mFoundPathIndex;
    int                           mRegionIndex;
    int                           mSubregionIndex;
    std::vector<QColor>           mColors;
    tree_show_type_t              mTreeShowType;

  private slots:
    void paintEvent(QPaintEvent * e);
  };

} // harrts

} // topologyPathPlanning

#endif // TOPOLOGYPATHPLANNING_HARRTS_BIRRTSTAR_VIZ_HPP
