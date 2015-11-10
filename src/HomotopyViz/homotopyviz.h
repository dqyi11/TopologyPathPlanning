#ifndef HOMOTOPYVIZ_H
#define HOMOTOPYVIZ_H

#include <vector>
#include <QLabel>
#include "worldmap.h"
#include "reference_frames.h"

namespace homotopy {

  class HomotopyViz : public QLabel {
    Q_OBJECT
  public:
    explicit HomotopyViz( QWidget *parent = 0 );
    bool loadMap( QString filename );

    bool                 mShowSubregion;
    bool                 mShowSubsegment;
    void prevRegion();
    void nextRegion();

    void prevSubregion();
    void nextSubregion();

    int  getRegionIdx()    { return mRegionIdx; }
    int  getSubregionIdx() { return mSubRegionIdx; }

    SubRegionSet* getSelectedRegion();
    SubRegion* getSelectedSubregion();

    bool save( QString filename );
    bool load( QString filename );
    
    QString generate_string();

  protected:
    void mousePressEvent( QMouseEvent * event );
    void mouseMoveEvent( QMouseEvent * event );
    void mouseReleaseEvent( QMouseEvent * event );
    bool initWorld(QString filename);

    WorldMap*            mpWorld;
    ReferenceFrameSet*   mpReferenceFrameSet;
    int                  mWorldWidth;
    int                  mWorldHeight;
    std::vector<QColor>  mColors;
    std::vector<QPoint>  mPoints;
    bool                 mDragging;

    int                  mRegionIdx;
    int                  mSubRegionIdx;
  signals:

  public slots:

  private slots:
    void paintEvent(QPaintEvent * e);
  };

}
#endif // HOMOTOPYVIZ_H
