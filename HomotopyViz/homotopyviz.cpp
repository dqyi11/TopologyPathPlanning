#include "homotopyviz.h"

HomotopyViz::HomotopyViz(QWidget *parent) :
    QLabel(parent) {

    mpWorld = NULL;
    mWorldWidth = 0;
    mWorldHeight = 0;
}

bool HomotopyViz::loadMap( QString filename ) {

    QPixmap pix(filename);
    if( pix.isNull() == true ) {
        return false;
    }

    setPixmap(pix);

    return true;
}
