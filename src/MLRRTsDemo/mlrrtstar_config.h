#ifndef MLRRTSTARCONFIG_H
#define MLRRTSTARCONFIG_H

#include <QDialog>
#include <QListWidget>
#include <QPushButton>
#include <QCheckBox>
#include <QLabel>
#include <QLineEdit>

namespace mlrrts {
  
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

    MLRRTstarWindow* mpParentWindow;

    bool isCompatible(QString fitnessFile);
  public slots:
    void checkBoxStateChanged(int state);
    void onBtnOKClicked();
    void onBtnCancelClicked();
    void onBtnAddClicked();
  };

}

#endif /* MLRRTSTARCONFIG_H */
