#ifndef TOPOLOGYPATHPLANNING_TARRTS_MLRRTSTARCONFIG_HPP
#define TOPOLOGYPATHPLANNING_TARRTS_MLRRTSTARCONFIG_HPP

#include <QDialog>
#include <QListWidget>
#include <QPushButton>
#include <QCheckBox>
#include <QLabel>
#include <QLineEdit>

namespace topologyPathPlanning {

namespace tarrts {
  
  class MLRRTstarWindow;

  class MLRRTstarConfig : public QDialog {
    Q_OBJECT
  public:
    MLRRTstarConfig(MLRRTstarWindow* parent);
    
    void updateConfiguration();
    void updateDisplay();

  private:
    QPushButton* mpBtnOK;
    QPushButton* mpBtnCancel;

    QCheckBox * mpCheckMinDist;
    QLabel    * mpLabelMinDist;

    QLabel      * mpLabelCost;
    QLineEdit   * mpLineEditCost;
    QPushButton * mpBtnAdd;

    QLabel    * mpLabelIterationNum;
    QLineEdit * mpLineEditIterationNum;
    QLabel    * mpLabelSegmentLength;
    QLineEdit * mpLineEditSegmentLength;

    QCheckBox * mpCheckHomotopicEnforcement;
    QLabel    * mpLabelHomotopicEnforcement;
    
    MLRRTstarWindow* mpParentWindow;

    bool isCompatible(QString fitnessFile);
  public slots:
    void checkMinDistStateChange(int state);
    void onBtnOKClicked();
    void onBtnCancelClicked();
    void onBtnAddClicked();
  };

} // tarrts

} // topologyPathPlanning

#endif // TOPOLOGYPATHPLANNING_TARRTS_MLRRTSTARCONFIG_HPP
