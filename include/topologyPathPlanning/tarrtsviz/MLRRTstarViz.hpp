#ifndef TOPOLOGYPATHPLANNING_TARRTS_MLRRTSTARVIZ_HPP
#define TOPOLOGYPATHPLANNING_TARRTS_MLRRTSTARVIZ_HPP

#include <QLabel>

#include "topologyPathPlanning/homotopy/ReferenceFrames.hpp"
#include "topologyPathPlanning/homotopy/Region.hpp"
#include "topologyPathPlanning/tarrts/MLRRTstar.hpp"
#include "topologyPathPlanning/tarrtsviz/MLRRTstarPathPlanningInfo.hpp"

namespace topologyPathPlanning {

namespace tarrts {

  enum MLRRTstarVizMode{
    NORMAL,
    DRAWING
  };

  class MLRRTstarViz : public QLabel
  {
    Q_OBJECT
  public:
    explicit MLRRTstarViz(QWidget* parent = 0);
    void setTree(MLRRTstar* p_tree);
    void setReferenceFrameSet(homotopy::ReferenceFrameSet* p_rf);
    bool drawPath(QString filename);
    bool saveCurrentViz(QString filename);

    void setShowReferenceFrames( bool show );
    void setShowSubregions( bool show );
    void setShowTree(bool show);
    void setFinishedPlanning( bool finished ) { mFinishedPlanning = finished; }
    void setShowPaths( bool show );

    bool showReferenceFrames() { return mShowReferenceFrames; }
    bool showSubregions() { return mShowSubregions; }
    bool showTree() { return mShowTree; }
    bool showPaths() { return mShowPaths; }
    bool isFinishedPlanning() { return mFinishedPlanning; }

    void setMode( MLRRTstarVizMode mode ) { mMode = mode; }
    MLRRTstarVizMode getMode() { return mMode; }

    homotopy::ReferenceFrame* getSelectedReferenceFrame();
    homotopy::SubRegion*      getSelectedSubregion();

    void prevSubregion();
    void nextSubregion();

    void prevStringClass();
    void nextStringClass();
    void prevExpNode();
    void nextExpNode();
    int getStringClassIndex() { return mStringClassIndex; }
    QString getStringClassInfo();

    void prevReferenceFrame();
    void nextReferenceFrame();
    void prevFoundPath();
    void nextFoundPath();
    int  getReferenceFrameIndex() { return mReferenceFrameIndex; }
    std::string getReferenceFrameName();
    std::string getSubregionName();

    QString generateString();
    void importStringConstraint( std::vector< QPoint > points, homotopy::grammar_type_t type );

    MLRRTstarPathPlanningInfo m_PPInfo;

    std::vector<QPoint>& getDrawedPoints() { return mDrawedPoints; }
    void setShowDrawedPoints( bool show ) { mShowPoints = show; mDrawedPoints.clear(); }
    bool getShowDrawedPoints() { return mShowPoints; }

    int getFoundPathIndex() { return mFoundPathIndex; }
    QString itemSelected( QPoint pos );

    Path* getVizPath();
  signals:

  public slots:

  protected:
    void mousePressEvent( QMouseEvent * event );
    void mouseMoveEvent( QMouseEvent * event );
    void mouseReleaseEvent( QMouseEvent * event );

    void updateVizSubregions();
    void updateVizReferenceFrames();

    std::vector<QPoint>  mDrawedPoints;
    bool                 mDragging;
    bool                 mShowPoints;
    MLRRTstarVizMode     mMode;
    QString              mItemSelectedName;
  private:
    void drawPathOnMap(QPixmap& map);
    void drawCurrentViz(QPixmap& map);
    void paint(QPaintDevice * device);

    MLRRTstar*                   mpTree;
    homotopy::ReferenceFrameSet* mpReferenceFrames;
    bool                         mShowReferenceFrames;
    bool                         mShowSubregions;
    bool                         mShowPaths;
    bool                         mShowTree;
    bool                         mFinishedPlanning;
    int                          mStringClassIndex;
    int                          mExpNodeIndex;
    int                          mReferenceFrameIndex;
    int                          mFoundPathIndex;
    int                          mSubregionIndex;

    int                          mExpNodeNum;

    std::vector<homotopy::SubRegion*>      mVizSubregions;
    std::vector<homotopy::ReferenceFrame*> mVizReferenceFrames;

  private slots:
    void paintEvent( QPaintEvent* e );
  };

} // tarrts

} // topologyPathPlanning

#endif // TOPOLOGYPATHPLANNING_TARRTS_MLRRTSTARVIZ_HPP
