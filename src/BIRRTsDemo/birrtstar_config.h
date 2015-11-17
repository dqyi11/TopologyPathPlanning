#ifndef BIRRTSTARCONFIG_H
#define BIRRTSTARCONFIG_H

#include <QDialog>
#include <QListWidget>
#include <QPushButton>
#include <QCheckBox>
#include <QLabel>
#include <QLineEdit>
#include <QComboBox>

namespace birrts {

  class BIRRTstarWindow;

  class BIRRTstarConfig : public QDialog {
    Q_OBJECT
  public:
    BIRRTstarConfig(BIRRTstarWindow * parent);

    void updateConfiguration();
    void updateDisplay();

  private:
    QPushButton * mpBtnOK;
    QPushButton * mpBtnCancel;

    QCheckBox * mpCheckMinDist;
    QLabel    * mpLabelMinDist;

    QLabel      * mpLabelCost;
    QLineEdit   * mpLineEditCost;
    QPushButton * mpBtnAdd;

    QLabel    * mpLabelIterationNum;
    QLineEdit * mpLineEditIterationNum;
    QLabel    * mpLabelSegmentLength;
    QLineEdit * mpLineEditSegmentLength;
    
    QLabel    * mpLabelGrammarType;
    QComboBox * mpComboGrammarType;

    QLabel    * mpLabelRunType;
    QComboBox * mpComboRunType;

    BIRRTstarWindow * mpParentWindow;

    bool isCompatible(QString fitnessFile);

  public slots:
    void checkBoxStateChanged(int state);
    void onBtnOKClicked();
    void onBtnCancelClicked();
    void onBtnAddClicked();
  };

}

#endif // BIRRTSTARCONFIG_H
