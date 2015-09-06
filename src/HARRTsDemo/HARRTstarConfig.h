#ifndef HARRTCONFIG_H
#define HARRTCONFIG_H

#include <QDialog>
#include <QListWidget>
#include <QPushButton>
#include <QCheckBox>
#include <QLabel>
#include <QLineEdit>
#include <QComboBox>

class HARRTWindow;

class HARRTConfig : public QDialog {
    Q_OBJECT
public:
    HARRTConfig(HARRTWindow * parent);

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

    HARRTWindow * mpParentWindow;

    bool isCompatible(QString fitnessFile);

public slots:
    void checkBoxStateChanged(int state);
    void onBtnOKClicked();
    void onBtnCancelClicked();
    void onBtnAddClicked();

};

#endif // HARRTCONFIG_H