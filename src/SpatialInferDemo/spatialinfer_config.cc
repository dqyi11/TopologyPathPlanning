#include <QVBoxLayout>
#include <QHBoxLayout>
#include "spatialinfer_window.h"
#include "spatialinfer_config.h"

using namespace topology_inference;

SpatialInferConfig::SpatialInferConfig( SpatialInferWindow* parent ) {
  mpParentWindow = parent;

  mpLabelSpatialRelations = new QLabel("Spatial Relation Functions");
  
  mpListWidget = new QListWidget();
  mpListWidget->setViewMode(QListView::IconMode);
  mpListWidget->show();

  mpBtnRemove = new QPushButton(tr("Remove"));
  mpBtnOK = new QPushButton(tr("OK"));
  QHBoxLayout* buttonsLayout = new QHBoxLayout();
  buttonsLayout->addWidget( mpBtnRemove );
  buttonsLayout->addWidget( mpBtnOK );
  
  connect(mpBtnRemove, SIGNAL(clicked()), this, SLOT(onBtnRemoveClicked()) );
  connect(mpBtnOK, SIGNAL(clicked()), this, SLOT(onBtnOKClicked()) );

  QVBoxLayout* mainLayout = new QVBoxLayout();
  mainLayout->addWidget( mpLabelSpatialRelations );
  mainLayout->addWidget( mpListWidget );
  mainLayout->addLayout( buttonsLayout );

  setLayout( mainLayout );
}

void SpatialInferConfig::onBtnOKClicked() {
  close();
}
 
void SpatialInferConfig::onBtnRemoveClicked() {
  qDeleteAll( mpListWidget->selectedItems() );
}


