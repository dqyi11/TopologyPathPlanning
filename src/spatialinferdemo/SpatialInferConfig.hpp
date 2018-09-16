#ifndef TOPOLOGYPATHPLANNING_TOPOLOGYINFERENCE_SPATIALINFERCONFIG_HPP
#define TOPOLOGYPATHPLANNING_TOPOLOGYINFERENCE_SPATIALINFERCONFIG_HPP

#include <QDialog>
#include <QListWidget>
#include <QPushButton>
#include <QLabel>

namespace topologyPathPlanning {

namespace topologyinference {
  
  class SpatialInferWindow;

  class SpatialInferConfig : public QDialog {
    Q_OBJECT
  public:
    SpatialInferConfig( SpatialInferWindow* parent );
    void updateDisplay();   
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

} // topologyinference

} // topologyPathPlanning

#endif // TOPOLOGYPATHPLANNING_TOPOLOGYINFERENCE_SPATIALINFERCONFIG_HPP
