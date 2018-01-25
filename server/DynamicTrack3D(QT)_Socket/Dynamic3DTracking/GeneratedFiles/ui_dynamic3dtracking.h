/********************************************************************************
** Form generated from reading UI file 'dynamic3dtracking.ui'
**
** Created by: Qt User Interface Compiler version 5.7.1
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_DYNAMIC3DTRACKING_H
#define UI_DYNAMIC3DTRACKING_H

#include <QtCore/QVariant>
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QButtonGroup>
#include <QtWidgets/QGridLayout>
#include <QtWidgets/QGroupBox>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QLabel>
#include <QtWidgets/QLineEdit>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QMenuBar>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QRadioButton>
#include <QtWidgets/QSpacerItem>
#include <QtWidgets/QStatusBar>
#include <QtWidgets/QToolBar>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_Dynamic3DTrackingClass
{
public:
    QWidget *centralWidget;
    QGridLayout *gridLayout_3;
    QVBoxLayout *verticalLayout_12;
    QVBoxLayout *verticalLayout_13;
    QGroupBox *groupBoxUp;
    QGridLayout *gridLayout_4;
    QHBoxLayout *horizontalLayout;
    QSpacerItem *horizontalSpacer;
    QVBoxLayout *verticalLayout;
    QPushButton *btnSavePath;
    QPushButton *btnOpenCams;
    QVBoxLayout *verticalLayout_3;
    QRadioButton *radioBtnL;
    QRadioButton *radioBtnR;
    QVBoxLayout *verticalLayout_2;
    QLineEdit *lineEditExpoTime;
    QPushButton *btnSetExpTime;
    QVBoxLayout *verticalLayout_4;
    QPushButton *btnSingleCamCapture;
    QPushButton *btnTwoCamsCapture;
    QVBoxLayout *verticalLayout_8;
    QLineEdit *lineEditPicNum;
    QPushButton *btnSetPicNum;
    QVBoxLayout *verticalLayout_10;
    QPushButton *externalTrigButton;
    QPushButton *btnDynamicTracking;
    QVBoxLayout *verticalLayout_9;
    QPushButton *BtnDeformation;
    QPushButton *btnXGraph;
    QVBoxLayout *verticalLayout_11;
    QPushButton *btnCloseCams;
    QPushButton *btnExit;
    QSpacerItem *horizontalSpacer_2;
    QHBoxLayout *horizontalLayout_2;
    QGroupBox *groupBoxL;
    QGridLayout *gridLayout;
    QLabel *labelLeftCam;
    QGroupBox *groupBoxR;
    QGridLayout *gridLayout_2;
    QLabel *labelRightCam;
    QMenuBar *menuBar;
    QToolBar *mainToolBar;
    QStatusBar *statusBar;

    void setupUi(QMainWindow *Dynamic3DTrackingClass)
    {
        if (Dynamic3DTrackingClass->objectName().isEmpty())
            Dynamic3DTrackingClass->setObjectName(QStringLiteral("Dynamic3DTrackingClass"));
        Dynamic3DTrackingClass->resize(1972, 1180);
        QSizePolicy sizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
        sizePolicy.setHorizontalStretch(0);
        sizePolicy.setVerticalStretch(10);
        sizePolicy.setHeightForWidth(Dynamic3DTrackingClass->sizePolicy().hasHeightForWidth());
        Dynamic3DTrackingClass->setSizePolicy(sizePolicy);
        centralWidget = new QWidget(Dynamic3DTrackingClass);
        centralWidget->setObjectName(QStringLiteral("centralWidget"));
        sizePolicy.setHeightForWidth(centralWidget->sizePolicy().hasHeightForWidth());
        centralWidget->setSizePolicy(sizePolicy);
        gridLayout_3 = new QGridLayout(centralWidget);
        gridLayout_3->setSpacing(6);
        gridLayout_3->setContentsMargins(11, 11, 11, 11);
        gridLayout_3->setObjectName(QStringLiteral("gridLayout_3"));
        verticalLayout_12 = new QVBoxLayout();
        verticalLayout_12->setSpacing(6);
        verticalLayout_12->setObjectName(QStringLiteral("verticalLayout_12"));

        gridLayout_3->addLayout(verticalLayout_12, 0, 0, 1, 1);

        verticalLayout_13 = new QVBoxLayout();
        verticalLayout_13->setSpacing(6);
        verticalLayout_13->setObjectName(QStringLiteral("verticalLayout_13"));
        groupBoxUp = new QGroupBox(centralWidget);
        groupBoxUp->setObjectName(QStringLiteral("groupBoxUp"));
        QSizePolicy sizePolicy1(QSizePolicy::Expanding, QSizePolicy::Fixed);
        sizePolicy1.setHorizontalStretch(0);
        sizePolicy1.setVerticalStretch(0);
        sizePolicy1.setHeightForWidth(groupBoxUp->sizePolicy().hasHeightForWidth());
        groupBoxUp->setSizePolicy(sizePolicy1);
        groupBoxUp->setToolTipDuration(-1);
        gridLayout_4 = new QGridLayout(groupBoxUp);
        gridLayout_4->setSpacing(6);
        gridLayout_4->setContentsMargins(11, 11, 11, 11);
        gridLayout_4->setObjectName(QStringLiteral("gridLayout_4"));
        horizontalLayout = new QHBoxLayout();
        horizontalLayout->setSpacing(6);
        horizontalLayout->setObjectName(QStringLiteral("horizontalLayout"));
        horizontalSpacer = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        horizontalLayout->addItem(horizontalSpacer);

        verticalLayout = new QVBoxLayout();
        verticalLayout->setSpacing(6);
        verticalLayout->setObjectName(QStringLiteral("verticalLayout"));
        btnSavePath = new QPushButton(groupBoxUp);
        btnSavePath->setObjectName(QStringLiteral("btnSavePath"));
        QIcon icon;
        icon.addFile(QString::fromUtf8(":/\344\277\235\345\255\230/Resources/Save_48px_1179933_easyicon.net.png"), QSize(), QIcon::Normal, QIcon::Off);
        btnSavePath->setIcon(icon);

        verticalLayout->addWidget(btnSavePath);

        btnOpenCams = new QPushButton(groupBoxUp);
        btnOpenCams->setObjectName(QStringLiteral("btnOpenCams"));
        btnOpenCams->setEnabled(false);
        QIcon icon1;
        icon1.addFile(QString::fromUtf8(":/\346\211\223\345\274\200\347\233\270\346\234\272/Resources/sonyericsson_camera_photo_48px_1135473_easyicon.net.png"), QSize(), QIcon::Normal, QIcon::Off);
        btnOpenCams->setIcon(icon1);

        verticalLayout->addWidget(btnOpenCams);


        horizontalLayout->addLayout(verticalLayout);

        verticalLayout_3 = new QVBoxLayout();
        verticalLayout_3->setSpacing(6);
        verticalLayout_3->setObjectName(QStringLiteral("verticalLayout_3"));
        radioBtnL = new QRadioButton(groupBoxUp);
        radioBtnL->setObjectName(QStringLiteral("radioBtnL"));
        radioBtnL->setEnabled(true);
        radioBtnL->setChecked(true);

        verticalLayout_3->addWidget(radioBtnL);

        radioBtnR = new QRadioButton(groupBoxUp);
        radioBtnR->setObjectName(QStringLiteral("radioBtnR"));

        verticalLayout_3->addWidget(radioBtnR);


        horizontalLayout->addLayout(verticalLayout_3);

        verticalLayout_2 = new QVBoxLayout();
        verticalLayout_2->setSpacing(6);
        verticalLayout_2->setObjectName(QStringLiteral("verticalLayout_2"));
        lineEditExpoTime = new QLineEdit(groupBoxUp);
        lineEditExpoTime->setObjectName(QStringLiteral("lineEditExpoTime"));
        QSizePolicy sizePolicy2(QSizePolicy::Preferred, QSizePolicy::Expanding);
        sizePolicy2.setHorizontalStretch(2);
        sizePolicy2.setVerticalStretch(0);
        sizePolicy2.setHeightForWidth(lineEditExpoTime->sizePolicy().hasHeightForWidth());
        lineEditExpoTime->setSizePolicy(sizePolicy2);
        lineEditExpoTime->setMinimumSize(QSize(100, 0));
        lineEditExpoTime->setMaximumSize(QSize(200, 16777215));
        lineEditExpoTime->setAlignment(Qt::AlignCenter);

        verticalLayout_2->addWidget(lineEditExpoTime);

        btnSetExpTime = new QPushButton(groupBoxUp);
        btnSetExpTime->setObjectName(QStringLiteral("btnSetExpTime"));
        QIcon icon2;
        icon2.addFile(QString::fromUtf8(":/\350\256\276\347\275\256\346\233\235\345\205\211\346\227\266\351\227\264/Resources/exposure_48px_1176623_easyicon.net.png"), QSize(), QIcon::Normal, QIcon::Off);
        btnSetExpTime->setIcon(icon2);

        verticalLayout_2->addWidget(btnSetExpTime);


        horizontalLayout->addLayout(verticalLayout_2);

        verticalLayout_4 = new QVBoxLayout();
        verticalLayout_4->setSpacing(6);
        verticalLayout_4->setObjectName(QStringLiteral("verticalLayout_4"));
        btnSingleCamCapture = new QPushButton(groupBoxUp);
        btnSingleCamCapture->setObjectName(QStringLiteral("btnSingleCamCapture"));
        btnSingleCamCapture->setEnabled(false);
        QIcon icon3;
        icon3.addFile(QString::fromUtf8(":/\345\215\225\347\233\270\346\234\272\346\213\215\346\221\204/Resources/com_motorola_camera_48px_1107083_easyicon.net.png"), QSize(), QIcon::Normal, QIcon::Off);
        btnSingleCamCapture->setIcon(icon3);

        verticalLayout_4->addWidget(btnSingleCamCapture);

        btnTwoCamsCapture = new QPushButton(groupBoxUp);
        btnTwoCamsCapture->setObjectName(QStringLiteral("btnTwoCamsCapture"));
        btnTwoCamsCapture->setEnabled(false);
        QIcon icon4;
        icon4.addFile(QString::fromUtf8(":/\345\217\214\347\233\270\346\234\272\346\213\215\346\221\204/Resources/binocular_Telescope_48px_558106_easyicon.net.png"), QSize(), QIcon::Normal, QIcon::Off);
        btnTwoCamsCapture->setIcon(icon4);

        verticalLayout_4->addWidget(btnTwoCamsCapture);


        horizontalLayout->addLayout(verticalLayout_4);

        verticalLayout_8 = new QVBoxLayout();
        verticalLayout_8->setSpacing(6);
        verticalLayout_8->setObjectName(QStringLiteral("verticalLayout_8"));
        lineEditPicNum = new QLineEdit(groupBoxUp);
        lineEditPicNum->setObjectName(QStringLiteral("lineEditPicNum"));
        sizePolicy2.setHeightForWidth(lineEditPicNum->sizePolicy().hasHeightForWidth());
        lineEditPicNum->setSizePolicy(sizePolicy2);
        lineEditPicNum->setMinimumSize(QSize(100, 0));
        lineEditPicNum->setMaximumSize(QSize(200, 16777215));
        lineEditPicNum->setAlignment(Qt::AlignCenter);

        verticalLayout_8->addWidget(lineEditPicNum);

        btnSetPicNum = new QPushButton(groupBoxUp);
        btnSetPicNum->setObjectName(QStringLiteral("btnSetPicNum"));
        QIcon icon5;
        icon5.addFile(QString::fromUtf8(":/\350\256\276\347\275\256\345\233\276\347\211\207\345\274\240\346\225\260/Resources/\350\256\276\347\275\256\345\233\276\347\211\207\345\274\240\346\225\260.png"), QSize(), QIcon::Normal, QIcon::Off);
        btnSetPicNum->setIcon(icon5);

        verticalLayout_8->addWidget(btnSetPicNum);


        horizontalLayout->addLayout(verticalLayout_8);

        verticalLayout_10 = new QVBoxLayout();
        verticalLayout_10->setSpacing(6);
        verticalLayout_10->setObjectName(QStringLiteral("verticalLayout_10"));
        externalTrigButton = new QPushButton(groupBoxUp);
        externalTrigButton->setObjectName(QStringLiteral("externalTrigButton"));
        QIcon icon6;
        icon6.addFile(QString::fromUtf8(":/\345\244\226\350\247\246\345\217\221/Resources/portal_trigger_48px_1066495_easyicon.net.png"), QSize(), QIcon::Normal, QIcon::Off);
        externalTrigButton->setIcon(icon6);

        verticalLayout_10->addWidget(externalTrigButton);

        btnDynamicTracking = new QPushButton(groupBoxUp);
        btnDynamicTracking->setObjectName(QStringLiteral("btnDynamicTracking"));
        btnDynamicTracking->setEnabled(true);
        QIcon icon7;
        icon7.addFile(QString::fromUtf8(":/\345\212\250\346\200\201\350\267\237\350\270\252/Resources/Short_Track_48px_1134850_easyicon.net.png"), QSize(), QIcon::Normal, QIcon::Off);
        btnDynamicTracking->setIcon(icon7);
        btnDynamicTracking->setCheckable(false);

        verticalLayout_10->addWidget(btnDynamicTracking);


        horizontalLayout->addLayout(verticalLayout_10);

        verticalLayout_9 = new QVBoxLayout();
        verticalLayout_9->setSpacing(6);
        verticalLayout_9->setObjectName(QStringLiteral("verticalLayout_9"));
        BtnDeformation = new QPushButton(groupBoxUp);
        BtnDeformation->setObjectName(QStringLiteral("BtnDeformation"));
        BtnDeformation->setEnabled(false);
        QIcon icon8;
        icon8.addFile(QString::fromUtf8(":/\346\230\276\347\244\272\345\217\230\345\275\242/Resources/software_font_size_47.81954887218px_1183001_easyicon.net.png"), QSize(), QIcon::Normal, QIcon::Off);
        BtnDeformation->setIcon(icon8);

        verticalLayout_9->addWidget(BtnDeformation);

        btnXGraph = new QPushButton(groupBoxUp);
        btnXGraph->setObjectName(QStringLiteral("btnXGraph"));
        btnXGraph->setEnabled(false);
        QIcon icon9;
        icon9.addFile(QString::fromUtf8(":/\344\275\215\347\247\273\345\233\276/Resources/diagram_48px_1200321_easyicon.net.png"), QSize(), QIcon::Normal, QIcon::Off);
        btnXGraph->setIcon(icon9);
        btnXGraph->setCheckable(false);

        verticalLayout_9->addWidget(btnXGraph);


        horizontalLayout->addLayout(verticalLayout_9);

        verticalLayout_11 = new QVBoxLayout();
        verticalLayout_11->setSpacing(6);
        verticalLayout_11->setObjectName(QStringLiteral("verticalLayout_11"));
        btnCloseCams = new QPushButton(groupBoxUp);
        btnCloseCams->setObjectName(QStringLiteral("btnCloseCams"));
        btnCloseCams->setEnabled(false);
        QIcon icon10;
        icon10.addFile(QString::fromUtf8(":/\345\205\263\351\227\255\347\233\270\346\234\272/Resources/camera_close_48px_1110923_easyicon.net.png"), QSize(), QIcon::Normal, QIcon::Off);
        btnCloseCams->setIcon(icon10);

        verticalLayout_11->addWidget(btnCloseCams);

        btnExit = new QPushButton(groupBoxUp);
        btnExit->setObjectName(QStringLiteral("btnExit"));
        QIcon icon11;
        icon11.addFile(QString::fromUtf8(":/\351\200\200\345\207\272/Resources/sign_out_exit_24px_515890_easyicon.net.png"), QSize(), QIcon::Normal, QIcon::Off);
        btnExit->setIcon(icon11);

        verticalLayout_11->addWidget(btnExit);


        horizontalLayout->addLayout(verticalLayout_11);

        horizontalSpacer_2 = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        horizontalLayout->addItem(horizontalSpacer_2);


        gridLayout_4->addLayout(horizontalLayout, 0, 0, 1, 1);


        verticalLayout_13->addWidget(groupBoxUp);

        horizontalLayout_2 = new QHBoxLayout();
        horizontalLayout_2->setSpacing(6);
        horizontalLayout_2->setObjectName(QStringLiteral("horizontalLayout_2"));
        groupBoxL = new QGroupBox(centralWidget);
        groupBoxL->setObjectName(QStringLiteral("groupBoxL"));
        QSizePolicy sizePolicy3(QSizePolicy::Expanding, QSizePolicy::Expanding);
        sizePolicy3.setHorizontalStretch(0);
        sizePolicy3.setVerticalStretch(12);
        sizePolicy3.setHeightForWidth(groupBoxL->sizePolicy().hasHeightForWidth());
        groupBoxL->setSizePolicy(sizePolicy3);
        gridLayout = new QGridLayout(groupBoxL);
        gridLayout->setSpacing(6);
        gridLayout->setContentsMargins(11, 11, 11, 11);
        gridLayout->setObjectName(QStringLiteral("gridLayout"));
        labelLeftCam = new QLabel(groupBoxL);
        labelLeftCam->setObjectName(QStringLiteral("labelLeftCam"));
        QSizePolicy sizePolicy4(QSizePolicy::Fixed, QSizePolicy::Fixed);
        sizePolicy4.setHorizontalStretch(0);
        sizePolicy4.setVerticalStretch(0);
        sizePolicy4.setHeightForWidth(labelLeftCam->sizePolicy().hasHeightForWidth());
        labelLeftCam->setSizePolicy(sizePolicy4);
        labelLeftCam->setMinimumSize(QSize(900, 900));
        labelLeftCam->setMouseTracking(false);
        labelLeftCam->setScaledContents(true);

        gridLayout->addWidget(labelLeftCam, 0, 0, 1, 1);


        horizontalLayout_2->addWidget(groupBoxL);

        groupBoxR = new QGroupBox(centralWidget);
        groupBoxR->setObjectName(QStringLiteral("groupBoxR"));
        sizePolicy3.setHeightForWidth(groupBoxR->sizePolicy().hasHeightForWidth());
        groupBoxR->setSizePolicy(sizePolicy3);
        gridLayout_2 = new QGridLayout(groupBoxR);
        gridLayout_2->setSpacing(6);
        gridLayout_2->setContentsMargins(11, 11, 11, 11);
        gridLayout_2->setObjectName(QStringLiteral("gridLayout_2"));
        labelRightCam = new QLabel(groupBoxR);
        labelRightCam->setObjectName(QStringLiteral("labelRightCam"));
        sizePolicy4.setHeightForWidth(labelRightCam->sizePolicy().hasHeightForWidth());
        labelRightCam->setSizePolicy(sizePolicy4);
        labelRightCam->setMinimumSize(QSize(900, 900));
        labelRightCam->setScaledContents(true);

        gridLayout_2->addWidget(labelRightCam, 0, 0, 1, 1);


        horizontalLayout_2->addWidget(groupBoxR);


        verticalLayout_13->addLayout(horizontalLayout_2);


        gridLayout_3->addLayout(verticalLayout_13, 0, 1, 1, 1);

        Dynamic3DTrackingClass->setCentralWidget(centralWidget);
        menuBar = new QMenuBar(Dynamic3DTrackingClass);
        menuBar->setObjectName(QStringLiteral("menuBar"));
        menuBar->setGeometry(QRect(0, 0, 1972, 34));
        Dynamic3DTrackingClass->setMenuBar(menuBar);
        mainToolBar = new QToolBar(Dynamic3DTrackingClass);
        mainToolBar->setObjectName(QStringLiteral("mainToolBar"));
        Dynamic3DTrackingClass->addToolBar(Qt::TopToolBarArea, mainToolBar);
        statusBar = new QStatusBar(Dynamic3DTrackingClass);
        statusBar->setObjectName(QStringLiteral("statusBar"));
        Dynamic3DTrackingClass->setStatusBar(statusBar);

        retranslateUi(Dynamic3DTrackingClass);

        QMetaObject::connectSlotsByName(Dynamic3DTrackingClass);
    } // setupUi

    void retranslateUi(QMainWindow *Dynamic3DTrackingClass)
    {
        Dynamic3DTrackingClass->setWindowTitle(QApplication::translate("Dynamic3DTrackingClass", "Dynamic3DTracking", Q_NULLPTR));
        groupBoxUp->setTitle(QString());
        btnSavePath->setText(QApplication::translate("Dynamic3DTrackingClass", "\344\277\235\345\255\230", Q_NULLPTR));
        btnOpenCams->setText(QApplication::translate("Dynamic3DTrackingClass", "OpenCams", Q_NULLPTR));
        radioBtnL->setText(QApplication::translate("Dynamic3DTrackingClass", "LCam", Q_NULLPTR));
        radioBtnR->setText(QApplication::translate("Dynamic3DTrackingClass", "RCam", Q_NULLPTR));
        btnSetExpTime->setText(QApplication::translate("Dynamic3DTrackingClass", "\350\256\276\347\275\256\346\233\235\345\205\211\346\227\266\351\227\264", Q_NULLPTR));
        btnSingleCamCapture->setText(QApplication::translate("Dynamic3DTrackingClass", "\345\215\225\347\233\270\346\234\272\346\213\215\347\205\247", Q_NULLPTR));
        btnTwoCamsCapture->setText(QApplication::translate("Dynamic3DTrackingClass", "\345\220\214\346\227\266\346\213\215\347\205\247", Q_NULLPTR));
        btnSetPicNum->setText(QApplication::translate("Dynamic3DTrackingClass", "\350\256\276\347\275\256\346\213\215\346\221\204\345\274\240\346\225\260", Q_NULLPTR));
        externalTrigButton->setText(QApplication::translate("Dynamic3DTrackingClass", "\345\244\226\350\247\246\345\217\221", Q_NULLPTR));
        btnDynamicTracking->setText(QApplication::translate("Dynamic3DTrackingClass", "\345\212\250\346\200\201\350\267\237\350\270\252", Q_NULLPTR));
        BtnDeformation->setText(QApplication::translate("Dynamic3DTrackingClass", "\346\230\276\347\244\272\345\217\230\345\275\242", Q_NULLPTR));
        btnXGraph->setText(QApplication::translate("Dynamic3DTrackingClass", "\344\275\215\347\247\273\345\233\276", Q_NULLPTR));
        btnCloseCams->setText(QApplication::translate("Dynamic3DTrackingClass", "\345\205\263\351\227\255\347\233\270\346\234\272", Q_NULLPTR));
        btnExit->setText(QApplication::translate("Dynamic3DTrackingClass", "\351\200\200\345\207\272", Q_NULLPTR));
        groupBoxL->setTitle(QString());
        labelLeftCam->setText(QApplication::translate("Dynamic3DTrackingClass", "\345\267\246\347\233\270\346\234\272\346\230\276\347\244\272", Q_NULLPTR));
        groupBoxR->setTitle(QString());
        labelRightCam->setText(QApplication::translate("Dynamic3DTrackingClass", "\345\217\263\347\233\270\346\234\272\346\230\276\347\244\272", Q_NULLPTR));
    } // retranslateUi

};

namespace Ui {
    class Dynamic3DTrackingClass: public Ui_Dynamic3DTrackingClass {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_DYNAMIC3DTRACKING_H
