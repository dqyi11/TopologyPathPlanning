#ifndef SPATIAL_INFER_CONFIG_H
#define SPATIAL_INFER_CONFIG_H

#include <QDialog>
#include <QListWidget>
#include <QPushButton>
#include <QLabel>

namespace topology_inference {
  
  class SpatialInferWindow;

  class SpatialInferConfig : public QDialog {
    Q_OBJECT
  public:
    SpatialInferConfig( SpatialInferWindow* parent );
   
  private:
    QListWidget* mpListWidget;
    QPushButton* mpBtnRemove;
    QPushButton* mpBtnOK;
  
    QLabel* mpLabelSpatialRelations;
  
    SpatialInferWindow* mpParentWindow;

  public slots:
    void onBtnOKClicked();
    void onBtnRemoveClicked();
 
  };


}

#endif // SPATIAL_INFER_CONFIG_H
