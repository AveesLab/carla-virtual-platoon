/********************************************************************************
** Form generated from reading UI file 'controller.ui'
**
** Created by: Qt User Interface Compiler version 5.9.5
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_CONTROLLER_H
#define UI_CONTROLLER_H

#include <QtCore/QVariant>
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QButtonGroup>
#include <QtWidgets/QComboBox>
#include <QtWidgets/QGridLayout>
#include <QtWidgets/QGroupBox>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QLabel>
#include <QtWidgets/QLineEdit>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QMenuBar>
#include <QtWidgets/QProgressBar>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QSlider>
#include <QtWidgets/QStatusBar>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_Controller
{
public:
    QWidget *centralwidget;
    QGridLayout *gridLayout_2;
    QGroupBox *FV1GBOX;
    QGridLayout *gridLayout_4;
    QLabel *label_24;
    QLabel *label_3;
    QLabel *label_33;
    QLabel *label_28;
    QLabel *FV1CurDist;
    QSlider *FV1DistSlider;
    QLabel *label_16;
    QProgressBar *FV1DistBar;
    QLabel *label_23;
    QLabel *label_8;
    QLabel *label_10;
    QLabel *label_9;
    QLabel *FV1CurVel;
    QSlider *FV1VelSlider;
    QLabel *FV1TarDist;
    QComboBox *FV1Box;
    QProgressBar *FV1VelBar;
    QLabel *FV1TarVel;
    QHBoxLayout *FV1LaneChange;
    QPushButton *FV1_Left_LC;
    QPushButton *FV1_Rear;
    QPushButton *FV1_Right_LC;
    QGroupBox *LVGBOX;
    QGridLayout *gridLayout_3;
    QLabel *LVCurDist;
    QLabel *label_13;
    QLabel *label_15;
    QLabel *LVCurVel;
    QLabel *label_12;
    QLabel *LVTarDist;
    QComboBox *LVBox;
    QLabel *label_2;
    QLabel *label_27;
    QLabel *label_11;
    QLabel *label_21;
    QSlider *LVDistSlider;
    QProgressBar *LVVelBar;
    QLabel *LVTarVel;
    QLabel *label_34;
    QProgressBar *LVDistBar;
    QSlider *LVVelSlider;
    QLabel *label_22;
    QHBoxLayout *LVLaneChange;
    QPushButton *LV_Left_LC;
    QPushButton *LV_Rear;
    QPushButton *LV_Right_LC;
    QGroupBox *MGBOX;
    QGridLayout *gridLayout;
    QLineEdit *MTarVel;
    QLabel *label_39;
    QPushButton *pushButton;
    QSlider *MVelSlider;
    QSlider *MDistSlider;
    QLabel *label;
    QLineEdit *MTarDist;
    QLabel *label_40;
    QPushButton *Send;
    QLabel *label_14;
    QLabel *LV_MAP;
    QLabel *FV1_MAP;
    QLabel *FV2_MAP;
    QGroupBox *FV2GBOX;
    QGridLayout *gridLayout_5;
    QLabel *label_17;
    QHBoxLayout *FV2LaneChange;
    QPushButton *FV2_Left_LC;
    QPushButton *FV2_Rear;
    QPushButton *FV2_Right_LC;
    QLabel *FV2CurDist;
    QSlider *FV2DistSlider;
    QLabel *label_7;
    QLabel *label_4;
    QLabel *label_32;
    QProgressBar *FV2DistBar;
    QComboBox *FV2Box;
    QLabel *label_25;
    QLabel *label_29;
    QLabel *FV2CurVel;
    QLabel *FV2TarDist;
    QLabel *label_5;
    QLabel *label_26;
    QProgressBar *FV2VelBar;
    QLabel *FV2TarVel;
    QSlider *FV2VelSlider;
    QLabel *label_6;
    QMenuBar *menubar;
    QStatusBar *statusbar;

    void setupUi(QMainWindow *Controller)
    {
        if (Controller->objectName().isEmpty())
            Controller->setObjectName(QStringLiteral("Controller"));
        Controller->resize(1151, 600);
        centralwidget = new QWidget(Controller);
        centralwidget->setObjectName(QStringLiteral("centralwidget"));
        centralwidget->setEnabled(true);
        gridLayout_2 = new QGridLayout(centralwidget);
        gridLayout_2->setObjectName(QStringLiteral("gridLayout_2"));
        FV1GBOX = new QGroupBox(centralwidget);
        FV1GBOX->setObjectName(QStringLiteral("FV1GBOX"));
        gridLayout_4 = new QGridLayout(FV1GBOX);
        gridLayout_4->setObjectName(QStringLiteral("gridLayout_4"));
        label_24 = new QLabel(FV1GBOX);
        label_24->setObjectName(QStringLiteral("label_24"));

        gridLayout_4->addWidget(label_24, 7, 3, 1, 1);

        label_3 = new QLabel(FV1GBOX);
        label_3->setObjectName(QStringLiteral("label_3"));

        gridLayout_4->addWidget(label_3, 2, 0, 1, 1);

        label_33 = new QLabel(FV1GBOX);
        label_33->setObjectName(QStringLiteral("label_33"));

        gridLayout_4->addWidget(label_33, 6, 3, 1, 1);

        label_28 = new QLabel(FV1GBOX);
        label_28->setObjectName(QStringLiteral("label_28"));

        gridLayout_4->addWidget(label_28, 2, 3, 1, 1);

        FV1CurDist = new QLabel(FV1GBOX);
        FV1CurDist->setObjectName(QStringLiteral("FV1CurDist"));
        FV1CurDist->setMinimumSize(QSize(60, 0));
        FV1CurDist->setSizeIncrement(QSize(0, 0));
        FV1CurDist->setMidLineWidth(0);
        FV1CurDist->setScaledContents(false);
        FV1CurDist->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        FV1CurDist->setMargin(5);

        gridLayout_4->addWidget(FV1CurDist, 7, 2, 1, 1);

        FV1DistSlider = new QSlider(FV1GBOX);
        FV1DistSlider->setObjectName(QStringLiteral("FV1DistSlider"));
        FV1DistSlider->setEnabled(false);
        FV1DistSlider->setOrientation(Qt::Horizontal);

        gridLayout_4->addWidget(FV1DistSlider, 6, 1, 1, 1);

        label_16 = new QLabel(FV1GBOX);
        label_16->setObjectName(QStringLiteral("label_16"));

        gridLayout_4->addWidget(label_16, 1, 0, 1, 1);

        FV1DistBar = new QProgressBar(FV1GBOX);
        FV1DistBar->setObjectName(QStringLiteral("FV1DistBar"));
        FV1DistBar->setValue(24);
        FV1DistBar->setTextVisible(false);

        gridLayout_4->addWidget(FV1DistBar, 7, 1, 1, 1);

        label_23 = new QLabel(FV1GBOX);
        label_23->setObjectName(QStringLiteral("label_23"));

        gridLayout_4->addWidget(label_23, 3, 3, 1, 1);

        label_8 = new QLabel(FV1GBOX);
        label_8->setObjectName(QStringLiteral("label_8"));

        gridLayout_4->addWidget(label_8, 3, 0, 1, 1);

        label_10 = new QLabel(FV1GBOX);
        label_10->setObjectName(QStringLiteral("label_10"));

        gridLayout_4->addWidget(label_10, 7, 0, 1, 1);

        label_9 = new QLabel(FV1GBOX);
        label_9->setObjectName(QStringLiteral("label_9"));

        gridLayout_4->addWidget(label_9, 6, 0, 1, 1);

        FV1CurVel = new QLabel(FV1GBOX);
        FV1CurVel->setObjectName(QStringLiteral("FV1CurVel"));
        FV1CurVel->setMinimumSize(QSize(60, 0));
        FV1CurVel->setSizeIncrement(QSize(0, 0));
        FV1CurVel->setMidLineWidth(0);
        FV1CurVel->setScaledContents(false);
        FV1CurVel->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        FV1CurVel->setMargin(5);

        gridLayout_4->addWidget(FV1CurVel, 3, 2, 1, 1);

        FV1VelSlider = new QSlider(FV1GBOX);
        FV1VelSlider->setObjectName(QStringLiteral("FV1VelSlider"));
        FV1VelSlider->setEnabled(false);
        FV1VelSlider->setOrientation(Qt::Horizontal);

        gridLayout_4->addWidget(FV1VelSlider, 2, 1, 1, 1);

        FV1TarDist = new QLabel(FV1GBOX);
        FV1TarDist->setObjectName(QStringLiteral("FV1TarDist"));
        FV1TarDist->setMinimumSize(QSize(60, 0));
        FV1TarDist->setSizeIncrement(QSize(0, 0));
        FV1TarDist->setMidLineWidth(0);
        FV1TarDist->setScaledContents(false);
        FV1TarDist->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        FV1TarDist->setMargin(5);

        gridLayout_4->addWidget(FV1TarDist, 6, 2, 1, 1);

        FV1Box = new QComboBox(FV1GBOX);
        FV1Box->setObjectName(QStringLiteral("FV1Box"));

        gridLayout_4->addWidget(FV1Box, 1, 1, 1, 1);

        FV1VelBar = new QProgressBar(FV1GBOX);
        FV1VelBar->setObjectName(QStringLiteral("FV1VelBar"));
        FV1VelBar->setValue(24);
        FV1VelBar->setTextVisible(false);

        gridLayout_4->addWidget(FV1VelBar, 3, 1, 1, 1);

        FV1TarVel = new QLabel(FV1GBOX);
        FV1TarVel->setObjectName(QStringLiteral("FV1TarVel"));
        FV1TarVel->setMinimumSize(QSize(60, 0));
        FV1TarVel->setSizeIncrement(QSize(0, 0));
        FV1TarVel->setMidLineWidth(0);
        FV1TarVel->setScaledContents(false);
        FV1TarVel->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        FV1TarVel->setMargin(5);

        gridLayout_4->addWidget(FV1TarVel, 2, 2, 1, 1);

        FV1LaneChange = new QHBoxLayout();
        FV1LaneChange->setObjectName(QStringLiteral("FV1LaneChange"));
        FV1_Left_LC = new QPushButton(FV1GBOX);
        FV1_Left_LC->setObjectName(QStringLiteral("FV1_Left_LC"));
        FV1_Left_LC->setCheckable(true);

//        FV1LaneChange->addWidget(FV1_Left_LC);

        FV1_Rear = new QPushButton(FV1GBOX);
        FV1_Rear->setObjectName(QStringLiteral("FV1_Rear"));
  FV1_Rear->setCheckable(true);

//        FV1LaneChange->addWidget(FV1_Rear);

        FV1_Right_LC = new QPushButton(FV1GBOX);
        FV1_Right_LC->setObjectName(QStringLiteral("FV1_Right_LC"));
  FV1_Right_LC->setCheckable(true);

//        FV1LaneChange->addWidget(FV1_Right_LC);


        gridLayout_4->addLayout(FV1LaneChange, 8, 0, 1, 4);


        gridLayout_2->addWidget(FV1GBOX, 2, 1, 1, 1);

        LVGBOX = new QGroupBox(centralwidget);
        LVGBOX->setObjectName(QStringLiteral("LVGBOX"));
        gridLayout_3 = new QGridLayout(LVGBOX);
        gridLayout_3->setObjectName(QStringLiteral("gridLayout_3"));
        LVCurDist = new QLabel(LVGBOX);
        LVCurDist->setObjectName(QStringLiteral("LVCurDist"));
        LVCurDist->setMinimumSize(QSize(60, 0));
        LVCurDist->setSizeIncrement(QSize(0, 0));
        LVCurDist->setMidLineWidth(0);
        LVCurDist->setScaledContents(false);
        LVCurDist->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        LVCurDist->setMargin(5);

        gridLayout_3->addWidget(LVCurDist, 4, 2, 1, 1);

        label_13 = new QLabel(LVGBOX);
        label_13->setObjectName(QStringLiteral("label_13"));

        gridLayout_3->addWidget(label_13, 3, 0, 1, 1);

        label_15 = new QLabel(LVGBOX);
        label_15->setObjectName(QStringLiteral("label_15"));

        gridLayout_3->addWidget(label_15, 0, 0, 1, 1);

        LVCurVel = new QLabel(LVGBOX);
        LVCurVel->setObjectName(QStringLiteral("LVCurVel"));
        LVCurVel->setMinimumSize(QSize(60, 0));
        LVCurVel->setSizeIncrement(QSize(0, 0));
        LVCurVel->setMidLineWidth(0);
        LVCurVel->setScaledContents(false);
        LVCurVel->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        LVCurVel->setMargin(5);

        gridLayout_3->addWidget(LVCurVel, 2, 2, 1, 1);

        label_12 = new QLabel(LVGBOX);
        label_12->setObjectName(QStringLiteral("label_12"));

        gridLayout_3->addWidget(label_12, 2, 0, 1, 1);

        LVTarDist = new QLabel(LVGBOX);
        LVTarDist->setObjectName(QStringLiteral("LVTarDist"));
        LVTarDist->setMinimumSize(QSize(60, 0));
        LVTarDist->setSizeIncrement(QSize(0, 0));
        LVTarDist->setMidLineWidth(0);
        LVTarDist->setScaledContents(false);
        LVTarDist->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        LVTarDist->setMargin(5);

        gridLayout_3->addWidget(LVTarDist, 3, 2, 1, 1);

        LVBox = new QComboBox(LVGBOX);
        LVBox->setObjectName(QStringLiteral("LVBox"));

        gridLayout_3->addWidget(LVBox, 0, 1, 1, 1);

        label_2 = new QLabel(LVGBOX);
        label_2->setObjectName(QStringLiteral("label_2"));

        gridLayout_3->addWidget(label_2, 1, 0, 1, 1);

        label_27 = new QLabel(LVGBOX);
        label_27->setObjectName(QStringLiteral("label_27"));

        gridLayout_3->addWidget(label_27, 1, 3, 1, 1);

        label_11 = new QLabel(LVGBOX);
        label_11->setObjectName(QStringLiteral("label_11"));

        gridLayout_3->addWidget(label_11, 4, 0, 1, 1);

        label_21 = new QLabel(LVGBOX);
        label_21->setObjectName(QStringLiteral("label_21"));

        gridLayout_3->addWidget(label_21, 2, 3, 1, 1);

        LVDistSlider = new QSlider(LVGBOX);
        LVDistSlider->setObjectName(QStringLiteral("LVDistSlider"));
        LVDistSlider->setOrientation(Qt::Horizontal);

        gridLayout_3->addWidget(LVDistSlider, 3, 1, 1, 1);

        LVVelBar = new QProgressBar(LVGBOX);
        LVVelBar->setObjectName(QStringLiteral("LVVelBar"));
        LVVelBar->setValue(24);
        LVVelBar->setTextVisible(false);

        gridLayout_3->addWidget(LVVelBar, 2, 1, 1, 1);

        LVTarVel = new QLabel(LVGBOX);
        LVTarVel->setObjectName(QStringLiteral("LVTarVel"));
        LVTarVel->setMinimumSize(QSize(60, 0));
        LVTarVel->setSizeIncrement(QSize(0, 0));
        LVTarVel->setMidLineWidth(0);
        LVTarVel->setScaledContents(false);
        LVTarVel->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        LVTarVel->setMargin(5);

        gridLayout_3->addWidget(LVTarVel, 1, 2, 1, 1);

        label_34 = new QLabel(LVGBOX);
        label_34->setObjectName(QStringLiteral("label_34"));

        gridLayout_3->addWidget(label_34, 3, 3, 1, 1);

        LVDistBar = new QProgressBar(LVGBOX);
        LVDistBar->setObjectName(QStringLiteral("LVDistBar"));
        LVDistBar->setValue(24);
        LVDistBar->setTextVisible(false);

        gridLayout_3->addWidget(LVDistBar, 4, 1, 1, 1);

        LVVelSlider = new QSlider(LVGBOX);
        LVVelSlider->setObjectName(QStringLiteral("LVVelSlider"));
        LVVelSlider->setOrientation(Qt::Horizontal);

        gridLayout_3->addWidget(LVVelSlider, 1, 1, 1, 1);

        label_22 = new QLabel(LVGBOX);
        label_22->setObjectName(QStringLiteral("label_22"));

        gridLayout_3->addWidget(label_22, 4, 3, 1, 1);

        LVLaneChange = new QHBoxLayout();
        LVLaneChange->setObjectName(QStringLiteral("LVLaneChange"));
        LV_Left_LC = new QPushButton(LVGBOX);
        LV_Left_LC->setObjectName(QStringLiteral("LV_Left_LC"));
        LV_Left_LC->setCheckable(true);

//        LVLaneChange->addWidget(LV_Left_LC);

        LV_Rear = new QPushButton(LVGBOX);
        LV_Rear->setObjectName(QStringLiteral("LV_Rear"));
  LV_Rear->setCheckable(true);

//        LVLaneChange->addWidget(LV_Rear);

        LV_Right_LC = new QPushButton(LVGBOX);
        LV_Right_LC->setObjectName(QStringLiteral("LV_Right_LC"));
  LV_Right_LC->setCheckable(true);

//        LVLaneChange->addWidget(LV_Right_LC);


        gridLayout_3->addLayout(LVLaneChange, 5, 0, 1, 4);


        gridLayout_2->addWidget(LVGBOX, 2, 0, 1, 1);

        MGBOX = new QGroupBox(centralwidget);
        MGBOX->setObjectName(QStringLiteral("MGBOX"));
        gridLayout = new QGridLayout(MGBOX);
        gridLayout->setObjectName(QStringLiteral("gridLayout"));
        MTarVel = new QLineEdit(MGBOX);
        MTarVel->setObjectName(QStringLiteral("MTarVel"));
        QSizePolicy sizePolicy(QSizePolicy::Minimum, QSizePolicy::Fixed);
        sizePolicy.setHorizontalStretch(0);
        sizePolicy.setVerticalStretch(0);
        sizePolicy.setHeightForWidth(MTarVel->sizePolicy().hasHeightForWidth());
        MTarVel->setSizePolicy(sizePolicy);
        MTarVel->setMaximumSize(QSize(50, 50));
        MTarVel->setMaxLength(32767);
        MTarVel->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);

        gridLayout->addWidget(MTarVel, 1, 2, 1, 1);

        label_39 = new QLabel(MGBOX);
        label_39->setObjectName(QStringLiteral("label_39"));

        gridLayout->addWidget(label_39, 1, 4, 1, 1);

        pushButton = new QPushButton(MGBOX);
        pushButton->setObjectName(QStringLiteral("pushButton"));

        gridLayout->addWidget(pushButton, 0, 1, 1, 1);

        MVelSlider = new QSlider(MGBOX);
        MVelSlider->setObjectName(QStringLiteral("MVelSlider"));
        MVelSlider->setOrientation(Qt::Horizontal);

        gridLayout->addWidget(MVelSlider, 1, 1, 1, 1);

        MDistSlider = new QSlider(MGBOX);
        MDistSlider->setObjectName(QStringLiteral("MDistSlider"));
        MDistSlider->setOrientation(Qt::Horizontal);

        gridLayout->addWidget(MDistSlider, 2, 1, 1, 1);

        label = new QLabel(MGBOX);
        label->setObjectName(QStringLiteral("label"));

        gridLayout->addWidget(label, 1, 0, 1, 1);

        MTarDist = new QLineEdit(MGBOX);
        MTarDist->setObjectName(QStringLiteral("MTarDist"));
        sizePolicy.setHeightForWidth(MTarDist->sizePolicy().hasHeightForWidth());
        MTarDist->setSizePolicy(sizePolicy);
        MTarDist->setMaximumSize(QSize(50, 50));
        MTarDist->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);

        gridLayout->addWidget(MTarDist, 2, 2, 1, 1);

        label_40 = new QLabel(MGBOX);
        label_40->setObjectName(QStringLiteral("label_40"));

        gridLayout->addWidget(label_40, 2, 4, 1, 1);

        Send = new QPushButton(MGBOX);
        Send->setObjectName(QStringLiteral("Send"));

        gridLayout->addWidget(Send, 0, 2, 1, 1);

        label_14 = new QLabel(MGBOX);
        label_14->setObjectName(QStringLiteral("label_14"));

        gridLayout->addWidget(label_14, 2, 0, 1, 1);


        gridLayout_2->addWidget(MGBOX, 0, 0, 1, 4);

        LV_MAP = new QLabel(centralwidget);
        LV_MAP->setObjectName(QStringLiteral("LV_MAP"));
        LV_MAP->setAlignment(Qt::AlignCenter);

        gridLayout_2->addWidget(LV_MAP, 4, 0, 1, 1);

        FV1_MAP = new QLabel(centralwidget);
        FV1_MAP->setObjectName(QStringLiteral("FV1_MAP"));
        FV1_MAP->setAlignment(Qt::AlignCenter);

        gridLayout_2->addWidget(FV1_MAP, 4, 1, 1, 1);

        FV2_MAP = new QLabel(centralwidget);
        FV2_MAP->setObjectName(QStringLiteral("FV2_MAP"));
        FV2_MAP->setAlignment(Qt::AlignCenter);

        gridLayout_2->addWidget(FV2_MAP, 4, 3, 1, 1);

        FV2GBOX = new QGroupBox(centralwidget);
        FV2GBOX->setObjectName(QStringLiteral("FV2GBOX"));
        gridLayout_5 = new QGridLayout(FV2GBOX);
        gridLayout_5->setObjectName(QStringLiteral("gridLayout_5"));
        label_17 = new QLabel(FV2GBOX);
        label_17->setObjectName(QStringLiteral("label_17"));

        gridLayout_5->addWidget(label_17, 0, 0, 1, 1);

        FV2LaneChange = new QHBoxLayout();
        FV2LaneChange->setObjectName(QStringLiteral("FV2LaneChange"));
        FV2_Left_LC = new QPushButton(FV2GBOX);
        FV2_Left_LC->setObjectName(QStringLiteral("FV2_Left_LC"));
        FV2_Left_LC->setCheckable(true);

//        FV2LaneChange->addWidget(FV2_Left_LC);

        FV2_Rear = new QPushButton(FV2GBOX);
        FV2_Rear->setObjectName(QStringLiteral("FV2_Rear"));
  FV2_Rear->setCheckable(true);

//        FV2LaneChange->addWidget(FV2_Rear);

        FV2_Right_LC = new QPushButton(FV2GBOX);
        FV2_Right_LC->setObjectName(QStringLiteral("FV2_Right_LC"));
        FV2_Right_LC->setCheckable(true);

//        FV2LaneChange->addWidget(FV2_Right_LC);

        gridLayout_5->addLayout(FV2LaneChange, 6, 0, 1, 4);

        FV2CurDist = new QLabel(FV2GBOX);
        FV2CurDist->setObjectName(QStringLiteral("FV2CurDist"));
        FV2CurDist->setMinimumSize(QSize(60, 0));
        FV2CurDist->setSizeIncrement(QSize(0, 0));
        FV2CurDist->setMidLineWidth(0);
        FV2CurDist->setScaledContents(false);
        FV2CurDist->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        FV2CurDist->setMargin(5);

        gridLayout_5->addWidget(FV2CurDist, 4, 2, 1, 1);

        FV2DistSlider = new QSlider(FV2GBOX);
        FV2DistSlider->setObjectName(QStringLiteral("FV2DistSlider"));
        FV2DistSlider->setEnabled(false);
        FV2DistSlider->setOrientation(Qt::Horizontal);

        gridLayout_5->addWidget(FV2DistSlider, 3, 1, 1, 1);

        label_7 = new QLabel(FV2GBOX);
        label_7->setObjectName(QStringLiteral("label_7"));

        gridLayout_5->addWidget(label_7, 4, 0, 1, 1);

        label_4 = new QLabel(FV2GBOX);
        label_4->setObjectName(QStringLiteral("label_4"));

        gridLayout_5->addWidget(label_4, 1, 0, 1, 1);

        label_32 = new QLabel(FV2GBOX);
        label_32->setObjectName(QStringLiteral("label_32"));

        gridLayout_5->addWidget(label_32, 3, 3, 1, 1);

        FV2DistBar = new QProgressBar(FV2GBOX);
        FV2DistBar->setObjectName(QStringLiteral("FV2DistBar"));
        FV2DistBar->setValue(24);
        FV2DistBar->setTextVisible(false);

        gridLayout_5->addWidget(FV2DistBar, 4, 1, 1, 1);

        FV2Box = new QComboBox(FV2GBOX);
        FV2Box->setObjectName(QStringLiteral("FV2Box"));

        gridLayout_5->addWidget(FV2Box, 0, 1, 1, 1);

        label_25 = new QLabel(FV2GBOX);
        label_25->setObjectName(QStringLiteral("label_25"));

        gridLayout_5->addWidget(label_25, 2, 3, 1, 1);

        label_29 = new QLabel(FV2GBOX);
        label_29->setObjectName(QStringLiteral("label_29"));

        gridLayout_5->addWidget(label_29, 1, 3, 1, 1);

        FV2CurVel = new QLabel(FV2GBOX);
        FV2CurVel->setObjectName(QStringLiteral("FV2CurVel"));
        FV2CurVel->setMinimumSize(QSize(60, 0));
        FV2CurVel->setSizeIncrement(QSize(0, 0));
        FV2CurVel->setMidLineWidth(0);
        FV2CurVel->setScaledContents(false);
        FV2CurVel->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        FV2CurVel->setMargin(5);

        gridLayout_5->addWidget(FV2CurVel, 2, 2, 1, 1);

        FV2TarDist = new QLabel(FV2GBOX);
        FV2TarDist->setObjectName(QStringLiteral("FV2TarDist"));
        FV2TarDist->setMinimumSize(QSize(60, 0));
        FV2TarDist->setSizeIncrement(QSize(0, 0));
        FV2TarDist->setMidLineWidth(0);
        FV2TarDist->setScaledContents(false);
        FV2TarDist->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        FV2TarDist->setMargin(5);

        gridLayout_5->addWidget(FV2TarDist, 3, 2, 1, 1);

        label_5 = new QLabel(FV2GBOX);
        label_5->setObjectName(QStringLiteral("label_5"));

        gridLayout_5->addWidget(label_5, 2, 0, 1, 1);

        label_26 = new QLabel(FV2GBOX);
        label_26->setObjectName(QStringLiteral("label_26"));

        gridLayout_5->addWidget(label_26, 4, 3, 1, 1);

        FV2VelBar = new QProgressBar(FV2GBOX);
        FV2VelBar->setObjectName(QStringLiteral("FV2VelBar"));
        FV2VelBar->setValue(24);
        FV2VelBar->setTextVisible(false);

        gridLayout_5->addWidget(FV2VelBar, 2, 1, 1, 1);

        FV2TarVel = new QLabel(FV2GBOX);
        FV2TarVel->setObjectName(QStringLiteral("FV2TarVel"));
        FV2TarVel->setMinimumSize(QSize(60, 0));
        FV2TarVel->setSizeIncrement(QSize(0, 0));
        FV2TarVel->setMidLineWidth(0);
        FV2TarVel->setScaledContents(false);
        FV2TarVel->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        FV2TarVel->setMargin(5);

        gridLayout_5->addWidget(FV2TarVel, 1, 2, 1, 1);

        FV2VelSlider = new QSlider(FV2GBOX);
        FV2VelSlider->setObjectName(QStringLiteral("FV2VelSlider"));
        FV2VelSlider->setEnabled(false);
        FV2VelSlider->setOrientation(Qt::Horizontal);

        gridLayout_5->addWidget(FV2VelSlider, 1, 1, 1, 1);

        label_6 = new QLabel(FV2GBOX);
        label_6->setObjectName(QStringLiteral("label_6"));

        gridLayout_5->addWidget(label_6, 3, 0, 1, 1);


        gridLayout_2->addWidget(FV2GBOX, 2, 3, 1, 1);

        Controller->setCentralWidget(centralwidget);
        menubar = new QMenuBar(Controller);
        menubar->setObjectName(QStringLiteral("menubar"));
        menubar->setGeometry(QRect(0, 0, 1151, 22));
        Controller->setMenuBar(menubar);
        statusbar = new QStatusBar(Controller);
        statusbar->setObjectName(QStringLiteral("statusbar"));
        Controller->setStatusBar(statusbar);

        retranslateUi(Controller);

        FV1Box->setCurrentIndex(2);
        LVBox->setCurrentIndex(1);
        FV2Box->setCurrentIndex(3);


        QMetaObject::connectSlotsByName(Controller);
    } // setupUi

    void retranslateUi(QMainWindow *Controller)
    {
        Controller->setWindowTitle(QApplication::translate("Controller", "Controller", Q_NULLPTR));
        FV1GBOX->setTitle(QApplication::translate("Controller", "FV1", Q_NULLPTR));
        label_24->setText(QApplication::translate("Controller", "m", Q_NULLPTR));
        label_3->setText(QApplication::translate("Controller", "Target Velocity", Q_NULLPTR));
        label_33->setText(QApplication::translate("Controller", "m", Q_NULLPTR));
        label_28->setText(QApplication::translate("Controller", "km/h", Q_NULLPTR));
        FV1CurDist->setText(QApplication::translate("Controller", "0", Q_NULLPTR));
        label_16->setText(QApplication::translate("Controller", "Mode", Q_NULLPTR));
        label_23->setText(QApplication::translate("Controller", "km/h", Q_NULLPTR));
        label_8->setText(QApplication::translate("Controller", "Current Velocity", Q_NULLPTR));
        label_10->setText(QApplication::translate("Controller", "Current Distance", Q_NULLPTR));
        label_9->setText(QApplication::translate("Controller", "Target Distance", Q_NULLPTR));
        FV1CurVel->setText(QApplication::translate("Controller", "0", Q_NULLPTR));
        FV1TarDist->setText(QApplication::translate("Controller", "0", Q_NULLPTR));
        FV1Box->clear();
        FV1Box->insertItems(0, QStringList()
         << QApplication::translate("Controller", "None", Q_NULLPTR)
         << QApplication::translate("Controller", "LV", Q_NULLPTR)
         << QApplication::translate("Controller", "FV1", Q_NULLPTR)
         << QApplication::translate("Controller", "FV2", Q_NULLPTR)
        );
        FV1TarVel->setText(QApplication::translate("Controller", "0", Q_NULLPTR));
//        FV1_Left_LC->setText(QApplication::translate("Controller", "Left LC", Q_NULLPTR));
//        FV1_Rear->setText(QApplication::translate("Controller", "Rear", Q_NULLPTR));
//        FV1_Right_LC->setText(QApplication::translate("Controller", "Right LC", Q_NULLPTR));
        LVGBOX->setTitle(QApplication::translate("Controller", "LV", Q_NULLPTR));
        LVCurDist->setText(QApplication::translate("Controller", "0", Q_NULLPTR));
        label_13->setText(QApplication::translate("Controller", "Target Distance", Q_NULLPTR));
        label_15->setText(QApplication::translate("Controller", "Mode", Q_NULLPTR));
        LVCurVel->setText(QApplication::translate("Controller", "0", Q_NULLPTR));
        label_12->setText(QApplication::translate("Controller", "Current Velocity", Q_NULLPTR));
        LVTarDist->setText(QApplication::translate("Controller", "0", Q_NULLPTR));
        LVBox->clear();
        LVBox->insertItems(0, QStringList()
         << QApplication::translate("Controller", "None", Q_NULLPTR)
         << QApplication::translate("Controller", "LV", Q_NULLPTR)
         << QApplication::translate("Controller", "FV1", Q_NULLPTR)
         << QApplication::translate("Controller", "FV2", Q_NULLPTR)
        );
        label_2->setText(QApplication::translate("Controller", "Target Velocity", Q_NULLPTR));
        label_27->setText(QApplication::translate("Controller", "m/s", Q_NULLPTR));
        label_11->setText(QApplication::translate("Controller", "Current Distance", Q_NULLPTR));
        label_21->setText(QApplication::translate("Controller", "km/h", Q_NULLPTR));
        LVTarVel->setText(QApplication::translate("Controller", "0", Q_NULLPTR));
        label_34->setText(QApplication::translate("Controller", "m", Q_NULLPTR));
        label_22->setText(QApplication::translate("Controller", "m", Q_NULLPTR));
//        LV_Left_LC->setText(QApplication::translate("Controller", "Left LC", Q_NULLPTR));
//        LV_Rear->setText(QApplication::translate("Controller", "Rear", Q_NULLPTR));
//        LV_Right_LC->setText(QApplication::translate("Controller", "Right LC", Q_NULLPTR));
        MGBOX->setTitle(QApplication::translate("Controller", "Master", Q_NULLPTR));
        MTarVel->setText(QApplication::translate("Controller", "0", Q_NULLPTR));
        label_39->setText(QApplication::translate("Controller", "km/h", Q_NULLPTR));
        pushButton->setText(QApplication::translate("Controller", "Emergency Stop", Q_NULLPTR));
        label->setText(QApplication::translate("Controller", "Target Velocity", Q_NULLPTR));
        MTarDist->setText(QApplication::translate("Controller", "0", Q_NULLPTR));
        label_40->setText(QApplication::translate("Controller", "m", Q_NULLPTR));
        Send->setText(QApplication::translate("Controller", "send", Q_NULLPTR));
        label_14->setText(QApplication::translate("Controller", "Target Distance", Q_NULLPTR));
        LV_MAP->setText(QApplication::translate("Controller", "LV", Q_NULLPTR));
        FV1_MAP->setText(QApplication::translate("Controller", "FV1", Q_NULLPTR));
        FV2_MAP->setText(QApplication::translate("Controller", "FV2", Q_NULLPTR));
        FV2GBOX->setTitle(QApplication::translate("Controller", "FV2", Q_NULLPTR));
        label_17->setText(QApplication::translate("Controller", "Mode", Q_NULLPTR));
//        FV2_Left_LC->setText(QApplication::translate("Controller", "Left LC", Q_NULLPTR));
//        FV2_Rear->setText(QApplication::translate("Controller", "Rear", Q_NULLPTR));
//        FV2_Right_LC->setText(QApplication::translate("Controller", "Right LC", Q_NULLPTR));
        FV2CurDist->setText(QApplication::translate("Controller", "0", Q_NULLPTR));
        label_7->setText(QApplication::translate("Controller", "Current Distance", Q_NULLPTR));
        label_4->setText(QApplication::translate("Controller", "Target Velocity", Q_NULLPTR));
        label_32->setText(QApplication::translate("Controller", "m", Q_NULLPTR));
        FV2Box->clear();
        FV2Box->insertItems(0, QStringList()
         << QApplication::translate("Controller", "None", Q_NULLPTR)
         << QApplication::translate("Controller", "LV", Q_NULLPTR)
         << QApplication::translate("Controller", "FV1", Q_NULLPTR)
         << QApplication::translate("Controller", "FV2", Q_NULLPTR)
        );
        label_25->setText(QApplication::translate("Controller", "km/h", Q_NULLPTR));
        label_29->setText(QApplication::translate("Controller", "km/h", Q_NULLPTR));
        FV2CurVel->setText(QApplication::translate("Controller", "0", Q_NULLPTR));
        FV2TarDist->setText(QApplication::translate("Controller", "0", Q_NULLPTR));
        label_5->setText(QApplication::translate("Controller", "Current Velocity", Q_NULLPTR));
        label_26->setText(QApplication::translate("Controller", "m", Q_NULLPTR));
        FV2TarVel->setText(QApplication::translate("Controller", "0", Q_NULLPTR));
        label_6->setText(QApplication::translate("Controller", "Target Distance", Q_NULLPTR));
    } // retranslateUi

};

namespace Ui {
    class Controller: public Ui_Controller {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_CONTROLLER_H
