#ifndef HOMOTOPYVIZ_H
#define HOMOTOPYVIZ_H

#include <vector>
#include <QLabel>
#include "worldmap.h"

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

    bool save( QString filename );
    bool load( QString filename );
protected:
    bool initWorld(QString filename);

    WorldMap*            mpWorld;
    int                  mWorldWidth;
    int                  mWorldHeight;
    std::vector<QColor>  mColors;

    int                  mRegionIdx;
    int                  mSubRegionIdx;
signals:

public slots:

private slots:
    void paintEvent(QPaintEvent * e);
};

#endif // HOMOTOPYVIZ_H
