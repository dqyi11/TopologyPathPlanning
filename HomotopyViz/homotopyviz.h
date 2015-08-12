#ifndef HOMOTOPYVIZ_H
#define HOMOTOPYVIZ_H

#include <QLabel>
#include "worldmap.h"

class HomotopyViz : public QLabel {
    Q_OBJECT
public:
    explicit HomotopyViz( QWidget *parent = 0 );
    bool loadMap( QString filename );


protected:
    WorldMap* mpWorld;
    int mWorldWidth;
    int mWorldHeight;
signals:

public slots:

};

#endif // HOMOTOPYVIZ_H
