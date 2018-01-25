/********************************************************************************
** Form generated from reading UI file 'dynamictrackdlg.ui'
**
** Created by: Qt User Interface Compiler version 5.7.1
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_DYNAMICTRACKDLG_H
#define UI_DYNAMICTRACKDLG_H

#include <QtCore/QVariant>
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QButtonGroup>
#include <QtWidgets/QDialog>
#include <QtWidgets/QFrame>
#include <QtWidgets/QGridLayout>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QLabel>
#include <QtWidgets/QLineEdit>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QSpacerItem>
#include <QtWidgets/QVBoxLayout>

QT_BEGIN_NAMESPACE

class Ui_DynamicTrackDlg
{
public:
    QGridLayout *gridLayout;
    QVBoxLayout *verticalLayout_4;
    QHBoxLayout *horizontalLayout_21;
    QVBoxLayout *verticalLayout_3;
    QHBoxLayout *horizontalLayout;
    QLabel *label_1;
    QLineEdit *lineEdit_1;
    QHBoxLayout *horizontalLayout_2;
    QLabel *label_2;
    QLineEdit *lineEdit_2;
    QHBoxLayout *horizontalLayout_4;
    QLabel *label_3;
    QLineEdit *lineEdit_3;
    QHBoxLayout *horizontalLayout_5;
    QLabel *label_4;
    QLineEdit *lineEdit_4;
    QHBoxLayout *horizontalLayout_6;
    QLabel *label_5;
    QLineEdit *lineEdit_5;
    QHBoxLayout *horizontalLayout_7;
    QLabel *label_6;
    QLineEdit *lineEdit_6;
    QHBoxLayout *horizontalLayout_8;
    QLabel *label_7;
    QLineEdit *lineEdit_7;
    QHBoxLayout *horizontalLayout_9;
    QLabel *label_8;
    QLineEdit *lineEdit_8;
    QHBoxLayout *horizontalLayout_10;
    QLabel *label_9;
    QLineEdit *lineEdit_9;
    QFrame *line_2;
    QVBoxLayout *verticalLayout_2;
    QVBoxLayout *verticalLayout;
    QHBoxLayout *horizontalLayout_11;
    QLabel *label_10;
    QLineEdit *lineEdit_10;
    QHBoxLayout *horizontalLayout_12;
    QLabel *label_11;
    QLineEdit *lineEdit_11;
    QHBoxLayout *horizontalLayout_13;
    QLabel *label_12;
    QLineEdit *lineEdit_12;
    QHBoxLayout *horizontalLayout_14;
    QLabel *label_13;
    QLineEdit *lineEdit_13;
    QHBoxLayout *horizontalLayout_15;
    QLabel *label_14;
    QLineEdit *lineEdit_14;
    QHBoxLayout *horizontalLayout_20;
    QSpacerItem *horizontalSpacer;
    QHBoxLayout *horizontalLayout_17;
    QLabel *label_16;
    QLineEdit *lineEditTotalNum;
    QHBoxLayout *horizontalLayout_19;
    QSpacerItem *horizontalSpacer_4;
    QHBoxLayout *horizontalLayout_18;
    QLabel *label;
    QLineEdit *lineEditImgNum;
    QSpacerItem *verticalSpacer;
    QFrame *line;
    QHBoxLayout *horizontalLayout_3;
    QPushButton *btnSetTrackPath;
    QSpacerItem *horizontalSpacer_2;
    QPushButton *btnToTrack;
    QSpacerItem *horizontalSpacer_3;
    QPushButton *btnStopTracking;

    void setupUi(QDialog *DynamicTrackDlg)
    {
        if (DynamicTrackDlg->objectName().isEmpty())
            DynamicTrackDlg->setObjectName(QStringLiteral("DynamicTrackDlg"));
        DynamicTrackDlg->resize(1362, 580);
        QSizePolicy sizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
        sizePolicy.setHorizontalStretch(0);
        sizePolicy.setVerticalStretch(0);
        sizePolicy.setHeightForWidth(DynamicTrackDlg->sizePolicy().hasHeightForWidth());
        DynamicTrackDlg->setSizePolicy(sizePolicy);
        DynamicTrackDlg->setMaximumSize(QSize(16777214, 16777215));
        DynamicTrackDlg->setLayoutDirection(Qt::LeftToRight);
        gridLayout = new QGridLayout(DynamicTrackDlg);
        gridLayout->setSpacing(6);
        gridLayout->setContentsMargins(11, 11, 11, 11);
        gridLayout->setObjectName(QStringLiteral("gridLayout"));
        gridLayout->setSizeConstraint(QLayout::SetFixedSize);
        verticalLayout_4 = new QVBoxLayout();
        verticalLayout_4->setSpacing(6);
        verticalLayout_4->setObjectName(QStringLiteral("verticalLayout_4"));
        horizontalLayout_21 = new QHBoxLayout();
        horizontalLayout_21->setSpacing(6);
        horizontalLayout_21->setObjectName(QStringLiteral("horizontalLayout_21"));
        verticalLayout_3 = new QVBoxLayout();
        verticalLayout_3->setSpacing(6);
        verticalLayout_3->setObjectName(QStringLiteral("verticalLayout_3"));
        horizontalLayout = new QHBoxLayout();
        horizontalLayout->setSpacing(6);
        horizontalLayout->setObjectName(QStringLiteral("horizontalLayout"));
        horizontalLayout->setSizeConstraint(QLayout::SetFixedSize);
        label_1 = new QLabel(DynamicTrackDlg);
        label_1->setObjectName(QStringLiteral("label_1"));
        QSizePolicy sizePolicy1(QSizePolicy::Fixed, QSizePolicy::Preferred);
        sizePolicy1.setHorizontalStretch(0);
        sizePolicy1.setVerticalStretch(0);
        sizePolicy1.setHeightForWidth(label_1->sizePolicy().hasHeightForWidth());
        label_1->setSizePolicy(sizePolicy1);
        label_1->setMinimumSize(QSize(50, 0));

        horizontalLayout->addWidget(label_1);

        lineEdit_1 = new QLineEdit(DynamicTrackDlg);
        lineEdit_1->setObjectName(QStringLiteral("lineEdit_1"));
        sizePolicy1.setHeightForWidth(lineEdit_1->sizePolicy().hasHeightForWidth());
        lineEdit_1->setSizePolicy(sizePolicy1);
        lineEdit_1->setMinimumSize(QSize(450, 30));

        horizontalLayout->addWidget(lineEdit_1);


        verticalLayout_3->addLayout(horizontalLayout);

        horizontalLayout_2 = new QHBoxLayout();
        horizontalLayout_2->setSpacing(6);
        horizontalLayout_2->setObjectName(QStringLiteral("horizontalLayout_2"));
        label_2 = new QLabel(DynamicTrackDlg);
        label_2->setObjectName(QStringLiteral("label_2"));
        sizePolicy1.setHeightForWidth(label_2->sizePolicy().hasHeightForWidth());
        label_2->setSizePolicy(sizePolicy1);
        label_2->setMinimumSize(QSize(0, 0));

        horizontalLayout_2->addWidget(label_2);

        lineEdit_2 = new QLineEdit(DynamicTrackDlg);
        lineEdit_2->setObjectName(QStringLiteral("lineEdit_2"));
        sizePolicy1.setHeightForWidth(lineEdit_2->sizePolicy().hasHeightForWidth());
        lineEdit_2->setSizePolicy(sizePolicy1);
        lineEdit_2->setMinimumSize(QSize(450, 30));

        horizontalLayout_2->addWidget(lineEdit_2);


        verticalLayout_3->addLayout(horizontalLayout_2);

        horizontalLayout_4 = new QHBoxLayout();
        horizontalLayout_4->setSpacing(6);
        horizontalLayout_4->setObjectName(QStringLiteral("horizontalLayout_4"));
        label_3 = new QLabel(DynamicTrackDlg);
        label_3->setObjectName(QStringLiteral("label_3"));
        sizePolicy1.setHeightForWidth(label_3->sizePolicy().hasHeightForWidth());
        label_3->setSizePolicy(sizePolicy1);
        label_3->setMinimumSize(QSize(0, 0));

        horizontalLayout_4->addWidget(label_3);

        lineEdit_3 = new QLineEdit(DynamicTrackDlg);
        lineEdit_3->setObjectName(QStringLiteral("lineEdit_3"));
        sizePolicy1.setHeightForWidth(lineEdit_3->sizePolicy().hasHeightForWidth());
        lineEdit_3->setSizePolicy(sizePolicy1);
        lineEdit_3->setMinimumSize(QSize(450, 30));

        horizontalLayout_4->addWidget(lineEdit_3);


        verticalLayout_3->addLayout(horizontalLayout_4);

        horizontalLayout_5 = new QHBoxLayout();
        horizontalLayout_5->setSpacing(6);
        horizontalLayout_5->setObjectName(QStringLiteral("horizontalLayout_5"));
        label_4 = new QLabel(DynamicTrackDlg);
        label_4->setObjectName(QStringLiteral("label_4"));
        sizePolicy1.setHeightForWidth(label_4->sizePolicy().hasHeightForWidth());
        label_4->setSizePolicy(sizePolicy1);
        label_4->setMinimumSize(QSize(0, 0));

        horizontalLayout_5->addWidget(label_4);

        lineEdit_4 = new QLineEdit(DynamicTrackDlg);
        lineEdit_4->setObjectName(QStringLiteral("lineEdit_4"));
        sizePolicy1.setHeightForWidth(lineEdit_4->sizePolicy().hasHeightForWidth());
        lineEdit_4->setSizePolicy(sizePolicy1);
        lineEdit_4->setMinimumSize(QSize(450, 30));

        horizontalLayout_5->addWidget(lineEdit_4);


        verticalLayout_3->addLayout(horizontalLayout_5);

        horizontalLayout_6 = new QHBoxLayout();
        horizontalLayout_6->setSpacing(6);
        horizontalLayout_6->setObjectName(QStringLiteral("horizontalLayout_6"));
        label_5 = new QLabel(DynamicTrackDlg);
        label_5->setObjectName(QStringLiteral("label_5"));
        sizePolicy1.setHeightForWidth(label_5->sizePolicy().hasHeightForWidth());
        label_5->setSizePolicy(sizePolicy1);
        label_5->setMinimumSize(QSize(0, 0));

        horizontalLayout_6->addWidget(label_5);

        lineEdit_5 = new QLineEdit(DynamicTrackDlg);
        lineEdit_5->setObjectName(QStringLiteral("lineEdit_5"));
        sizePolicy1.setHeightForWidth(lineEdit_5->sizePolicy().hasHeightForWidth());
        lineEdit_5->setSizePolicy(sizePolicy1);
        lineEdit_5->setMinimumSize(QSize(450, 30));

        horizontalLayout_6->addWidget(lineEdit_5);


        verticalLayout_3->addLayout(horizontalLayout_6);

        horizontalLayout_7 = new QHBoxLayout();
        horizontalLayout_7->setSpacing(6);
        horizontalLayout_7->setObjectName(QStringLiteral("horizontalLayout_7"));
        label_6 = new QLabel(DynamicTrackDlg);
        label_6->setObjectName(QStringLiteral("label_6"));
        sizePolicy1.setHeightForWidth(label_6->sizePolicy().hasHeightForWidth());
        label_6->setSizePolicy(sizePolicy1);
        label_6->setMinimumSize(QSize(0, 0));

        horizontalLayout_7->addWidget(label_6);

        lineEdit_6 = new QLineEdit(DynamicTrackDlg);
        lineEdit_6->setObjectName(QStringLiteral("lineEdit_6"));
        sizePolicy1.setHeightForWidth(lineEdit_6->sizePolicy().hasHeightForWidth());
        lineEdit_6->setSizePolicy(sizePolicy1);
        lineEdit_6->setMinimumSize(QSize(450, 30));

        horizontalLayout_7->addWidget(lineEdit_6);


        verticalLayout_3->addLayout(horizontalLayout_7);

        horizontalLayout_8 = new QHBoxLayout();
        horizontalLayout_8->setSpacing(6);
        horizontalLayout_8->setObjectName(QStringLiteral("horizontalLayout_8"));
        label_7 = new QLabel(DynamicTrackDlg);
        label_7->setObjectName(QStringLiteral("label_7"));
        sizePolicy1.setHeightForWidth(label_7->sizePolicy().hasHeightForWidth());
        label_7->setSizePolicy(sizePolicy1);
        label_7->setMinimumSize(QSize(0, 0));

        horizontalLayout_8->addWidget(label_7);

        lineEdit_7 = new QLineEdit(DynamicTrackDlg);
        lineEdit_7->setObjectName(QStringLiteral("lineEdit_7"));
        sizePolicy1.setHeightForWidth(lineEdit_7->sizePolicy().hasHeightForWidth());
        lineEdit_7->setSizePolicy(sizePolicy1);
        lineEdit_7->setMinimumSize(QSize(450, 30));

        horizontalLayout_8->addWidget(lineEdit_7);


        verticalLayout_3->addLayout(horizontalLayout_8);

        horizontalLayout_9 = new QHBoxLayout();
        horizontalLayout_9->setSpacing(6);
        horizontalLayout_9->setObjectName(QStringLiteral("horizontalLayout_9"));
        label_8 = new QLabel(DynamicTrackDlg);
        label_8->setObjectName(QStringLiteral("label_8"));
        sizePolicy1.setHeightForWidth(label_8->sizePolicy().hasHeightForWidth());
        label_8->setSizePolicy(sizePolicy1);
        label_8->setMinimumSize(QSize(0, 0));

        horizontalLayout_9->addWidget(label_8);

        lineEdit_8 = new QLineEdit(DynamicTrackDlg);
        lineEdit_8->setObjectName(QStringLiteral("lineEdit_8"));
        sizePolicy1.setHeightForWidth(lineEdit_8->sizePolicy().hasHeightForWidth());
        lineEdit_8->setSizePolicy(sizePolicy1);
        lineEdit_8->setMinimumSize(QSize(450, 30));

        horizontalLayout_9->addWidget(lineEdit_8);


        verticalLayout_3->addLayout(horizontalLayout_9);

        horizontalLayout_10 = new QHBoxLayout();
        horizontalLayout_10->setSpacing(6);
        horizontalLayout_10->setObjectName(QStringLiteral("horizontalLayout_10"));
        label_9 = new QLabel(DynamicTrackDlg);
        label_9->setObjectName(QStringLiteral("label_9"));
        sizePolicy1.setHeightForWidth(label_9->sizePolicy().hasHeightForWidth());
        label_9->setSizePolicy(sizePolicy1);
        label_9->setMinimumSize(QSize(0, 0));

        horizontalLayout_10->addWidget(label_9);

        lineEdit_9 = new QLineEdit(DynamicTrackDlg);
        lineEdit_9->setObjectName(QStringLiteral("lineEdit_9"));
        sizePolicy1.setHeightForWidth(lineEdit_9->sizePolicy().hasHeightForWidth());
        lineEdit_9->setSizePolicy(sizePolicy1);
        lineEdit_9->setMinimumSize(QSize(450, 30));

        horizontalLayout_10->addWidget(lineEdit_9);


        verticalLayout_3->addLayout(horizontalLayout_10);


        horizontalLayout_21->addLayout(verticalLayout_3);

        line_2 = new QFrame(DynamicTrackDlg);
        line_2->setObjectName(QStringLiteral("line_2"));
        line_2->setFrameShape(QFrame::VLine);
        line_2->setFrameShadow(QFrame::Sunken);

        horizontalLayout_21->addWidget(line_2);

        verticalLayout_2 = new QVBoxLayout();
        verticalLayout_2->setSpacing(6);
        verticalLayout_2->setObjectName(QStringLiteral("verticalLayout_2"));
        verticalLayout = new QVBoxLayout();
        verticalLayout->setSpacing(6);
        verticalLayout->setObjectName(QStringLiteral("verticalLayout"));
        horizontalLayout_11 = new QHBoxLayout();
        horizontalLayout_11->setSpacing(6);
        horizontalLayout_11->setObjectName(QStringLiteral("horizontalLayout_11"));
        label_10 = new QLabel(DynamicTrackDlg);
        label_10->setObjectName(QStringLiteral("label_10"));
        sizePolicy1.setHeightForWidth(label_10->sizePolicy().hasHeightForWidth());
        label_10->setSizePolicy(sizePolicy1);
        label_10->setMinimumSize(QSize(50, 0));

        horizontalLayout_11->addWidget(label_10);

        lineEdit_10 = new QLineEdit(DynamicTrackDlg);
        lineEdit_10->setObjectName(QStringLiteral("lineEdit_10"));
        sizePolicy.setHeightForWidth(lineEdit_10->sizePolicy().hasHeightForWidth());
        lineEdit_10->setSizePolicy(sizePolicy);
        lineEdit_10->setMinimumSize(QSize(450, 30));

        horizontalLayout_11->addWidget(lineEdit_10);


        verticalLayout->addLayout(horizontalLayout_11);

        horizontalLayout_12 = new QHBoxLayout();
        horizontalLayout_12->setSpacing(6);
        horizontalLayout_12->setObjectName(QStringLiteral("horizontalLayout_12"));
        label_11 = new QLabel(DynamicTrackDlg);
        label_11->setObjectName(QStringLiteral("label_11"));
        sizePolicy1.setHeightForWidth(label_11->sizePolicy().hasHeightForWidth());
        label_11->setSizePolicy(sizePolicy1);
        label_11->setMinimumSize(QSize(50, 0));

        horizontalLayout_12->addWidget(label_11);

        lineEdit_11 = new QLineEdit(DynamicTrackDlg);
        lineEdit_11->setObjectName(QStringLiteral("lineEdit_11"));
        sizePolicy.setHeightForWidth(lineEdit_11->sizePolicy().hasHeightForWidth());
        lineEdit_11->setSizePolicy(sizePolicy);
        lineEdit_11->setMinimumSize(QSize(450, 30));

        horizontalLayout_12->addWidget(lineEdit_11);


        verticalLayout->addLayout(horizontalLayout_12);

        horizontalLayout_13 = new QHBoxLayout();
        horizontalLayout_13->setSpacing(6);
        horizontalLayout_13->setObjectName(QStringLiteral("horizontalLayout_13"));
        label_12 = new QLabel(DynamicTrackDlg);
        label_12->setObjectName(QStringLiteral("label_12"));
        sizePolicy1.setHeightForWidth(label_12->sizePolicy().hasHeightForWidth());
        label_12->setSizePolicy(sizePolicy1);
        label_12->setMinimumSize(QSize(50, 0));

        horizontalLayout_13->addWidget(label_12);

        lineEdit_12 = new QLineEdit(DynamicTrackDlg);
        lineEdit_12->setObjectName(QStringLiteral("lineEdit_12"));
        sizePolicy.setHeightForWidth(lineEdit_12->sizePolicy().hasHeightForWidth());
        lineEdit_12->setSizePolicy(sizePolicy);
        lineEdit_12->setMinimumSize(QSize(450, 30));

        horizontalLayout_13->addWidget(lineEdit_12);


        verticalLayout->addLayout(horizontalLayout_13);

        horizontalLayout_14 = new QHBoxLayout();
        horizontalLayout_14->setSpacing(6);
        horizontalLayout_14->setObjectName(QStringLiteral("horizontalLayout_14"));
        label_13 = new QLabel(DynamicTrackDlg);
        label_13->setObjectName(QStringLiteral("label_13"));
        sizePolicy1.setHeightForWidth(label_13->sizePolicy().hasHeightForWidth());
        label_13->setSizePolicy(sizePolicy1);
        label_13->setMinimumSize(QSize(50, 0));

        horizontalLayout_14->addWidget(label_13);

        lineEdit_13 = new QLineEdit(DynamicTrackDlg);
        lineEdit_13->setObjectName(QStringLiteral("lineEdit_13"));
        sizePolicy.setHeightForWidth(lineEdit_13->sizePolicy().hasHeightForWidth());
        lineEdit_13->setSizePolicy(sizePolicy);
        lineEdit_13->setMinimumSize(QSize(450, 30));

        horizontalLayout_14->addWidget(lineEdit_13);


        verticalLayout->addLayout(horizontalLayout_14);

        horizontalLayout_15 = new QHBoxLayout();
        horizontalLayout_15->setSpacing(6);
        horizontalLayout_15->setObjectName(QStringLiteral("horizontalLayout_15"));
        label_14 = new QLabel(DynamicTrackDlg);
        label_14->setObjectName(QStringLiteral("label_14"));
        sizePolicy1.setHeightForWidth(label_14->sizePolicy().hasHeightForWidth());
        label_14->setSizePolicy(sizePolicy1);
        label_14->setMinimumSize(QSize(50, 0));
        label_14->setSizeIncrement(QSize(0, 29));

        horizontalLayout_15->addWidget(label_14);

        lineEdit_14 = new QLineEdit(DynamicTrackDlg);
        lineEdit_14->setObjectName(QStringLiteral("lineEdit_14"));
        sizePolicy.setHeightForWidth(lineEdit_14->sizePolicy().hasHeightForWidth());
        lineEdit_14->setSizePolicy(sizePolicy);
        lineEdit_14->setMinimumSize(QSize(450, 30));

        horizontalLayout_15->addWidget(lineEdit_14);


        verticalLayout->addLayout(horizontalLayout_15);

        horizontalLayout_20 = new QHBoxLayout();
        horizontalLayout_20->setSpacing(6);
        horizontalLayout_20->setObjectName(QStringLiteral("horizontalLayout_20"));
        horizontalSpacer = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        horizontalLayout_20->addItem(horizontalSpacer);

        horizontalLayout_17 = new QHBoxLayout();
        horizontalLayout_17->setSpacing(6);
        horizontalLayout_17->setObjectName(QStringLiteral("horizontalLayout_17"));
        label_16 = new QLabel(DynamicTrackDlg);
        label_16->setObjectName(QStringLiteral("label_16"));
        sizePolicy1.setHeightForWidth(label_16->sizePolicy().hasHeightForWidth());
        label_16->setSizePolicy(sizePolicy1);
        label_16->setMinimumSize(QSize(50, 0));
        label_16->setAlignment(Qt::AlignCenter);
        label_16->setIndent(-2);

        horizontalLayout_17->addWidget(label_16);

        lineEditTotalNum = new QLineEdit(DynamicTrackDlg);
        lineEditTotalNum->setObjectName(QStringLiteral("lineEditTotalNum"));
        sizePolicy.setHeightForWidth(lineEditTotalNum->sizePolicy().hasHeightForWidth());
        lineEditTotalNum->setSizePolicy(sizePolicy);
        lineEditTotalNum->setMinimumSize(QSize(450, 30));

        horizontalLayout_17->addWidget(lineEditTotalNum);


        horizontalLayout_20->addLayout(horizontalLayout_17);


        verticalLayout->addLayout(horizontalLayout_20);

        horizontalLayout_19 = new QHBoxLayout();
        horizontalLayout_19->setSpacing(6);
        horizontalLayout_19->setObjectName(QStringLiteral("horizontalLayout_19"));
        horizontalSpacer_4 = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        horizontalLayout_19->addItem(horizontalSpacer_4);

        horizontalLayout_18 = new QHBoxLayout();
        horizontalLayout_18->setSpacing(6);
        horizontalLayout_18->setObjectName(QStringLiteral("horizontalLayout_18"));
        label = new QLabel(DynamicTrackDlg);
        label->setObjectName(QStringLiteral("label"));
        sizePolicy1.setHeightForWidth(label->sizePolicy().hasHeightForWidth());
        label->setSizePolicy(sizePolicy1);
        label->setMinimumSize(QSize(50, 0));
        label->setAlignment(Qt::AlignCenter);

        horizontalLayout_18->addWidget(label);

        lineEditImgNum = new QLineEdit(DynamicTrackDlg);
        lineEditImgNum->setObjectName(QStringLiteral("lineEditImgNum"));
        sizePolicy1.setHeightForWidth(lineEditImgNum->sizePolicy().hasHeightForWidth());
        lineEditImgNum->setSizePolicy(sizePolicy1);
        lineEditImgNum->setMinimumSize(QSize(450, 30));

        horizontalLayout_18->addWidget(lineEditImgNum);


        horizontalLayout_19->addLayout(horizontalLayout_18);


        verticalLayout->addLayout(horizontalLayout_19);


        verticalLayout_2->addLayout(verticalLayout);

        verticalSpacer = new QSpacerItem(20, 40, QSizePolicy::Minimum, QSizePolicy::Expanding);

        verticalLayout_2->addItem(verticalSpacer);


        horizontalLayout_21->addLayout(verticalLayout_2);


        verticalLayout_4->addLayout(horizontalLayout_21);

        line = new QFrame(DynamicTrackDlg);
        line->setObjectName(QStringLiteral("line"));
        line->setFrameShape(QFrame::HLine);
        line->setFrameShadow(QFrame::Sunken);

        verticalLayout_4->addWidget(line);

        horizontalLayout_3 = new QHBoxLayout();
        horizontalLayout_3->setSpacing(6);
        horizontalLayout_3->setObjectName(QStringLiteral("horizontalLayout_3"));
        btnSetTrackPath = new QPushButton(DynamicTrackDlg);
        btnSetTrackPath->setObjectName(QStringLiteral("btnSetTrackPath"));
        QSizePolicy sizePolicy2(QSizePolicy::Preferred, QSizePolicy::Fixed);
        sizePolicy2.setHorizontalStretch(0);
        sizePolicy2.setVerticalStretch(0);
        sizePolicy2.setHeightForWidth(btnSetTrackPath->sizePolicy().hasHeightForWidth());
        btnSetTrackPath->setSizePolicy(sizePolicy2);
        btnSetTrackPath->setMinimumSize(QSize(0, 50));

        horizontalLayout_3->addWidget(btnSetTrackPath);

        horizontalSpacer_2 = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        horizontalLayout_3->addItem(horizontalSpacer_2);

        btnToTrack = new QPushButton(DynamicTrackDlg);
        btnToTrack->setObjectName(QStringLiteral("btnToTrack"));
        btnToTrack->setEnabled(false);
        sizePolicy2.setHeightForWidth(btnToTrack->sizePolicy().hasHeightForWidth());
        btnToTrack->setSizePolicy(sizePolicy2);
        btnToTrack->setMinimumSize(QSize(0, 50));

        horizontalLayout_3->addWidget(btnToTrack);

        horizontalSpacer_3 = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        horizontalLayout_3->addItem(horizontalSpacer_3);

        btnStopTracking = new QPushButton(DynamicTrackDlg);
        btnStopTracking->setObjectName(QStringLiteral("btnStopTracking"));
        sizePolicy2.setHeightForWidth(btnStopTracking->sizePolicy().hasHeightForWidth());
        btnStopTracking->setSizePolicy(sizePolicy2);
        btnStopTracking->setMinimumSize(QSize(0, 50));

        horizontalLayout_3->addWidget(btnStopTracking);


        verticalLayout_4->addLayout(horizontalLayout_3);


        gridLayout->addLayout(verticalLayout_4, 0, 0, 1, 1);


        retranslateUi(DynamicTrackDlg);

        QMetaObject::connectSlotsByName(DynamicTrackDlg);
    } // setupUi

    void retranslateUi(QDialog *DynamicTrackDlg)
    {
        DynamicTrackDlg->setWindowTitle(QApplication::translate("DynamicTrackDlg", "\345\212\250\346\200\201\350\267\237\350\270\252", Q_NULLPTR));
        label_1->setText(QApplication::translate("DynamicTrackDlg", "\347\254\2541\344\270\252\347\274\226\347\240\201\347\202\271\345\235\220\346\240\207\357\274\232", Q_NULLPTR));
        label_2->setText(QApplication::translate("DynamicTrackDlg", "\347\254\2542\344\270\252\347\274\226\347\240\201\347\202\271\345\235\220\346\240\207\357\274\232", Q_NULLPTR));
        label_3->setText(QApplication::translate("DynamicTrackDlg", "\347\254\2543\344\270\252\347\274\226\347\240\201\347\202\271\345\235\220\346\240\207\357\274\232", Q_NULLPTR));
        label_4->setText(QApplication::translate("DynamicTrackDlg", "\347\254\2544\344\270\252\347\274\226\347\240\201\347\202\271\345\235\220\346\240\207\357\274\232", Q_NULLPTR));
        label_5->setText(QApplication::translate("DynamicTrackDlg", "\347\254\2545\344\270\252\347\274\226\347\240\201\347\202\271\345\235\220\346\240\207\357\274\232", Q_NULLPTR));
        label_6->setText(QApplication::translate("DynamicTrackDlg", "\347\254\2546\344\270\252\347\274\226\347\240\201\347\202\271\345\235\220\346\240\207\357\274\232", Q_NULLPTR));
        label_7->setText(QApplication::translate("DynamicTrackDlg", "\347\254\2547\344\270\252\347\274\226\347\240\201\347\202\271\345\235\220\346\240\207\357\274\232", Q_NULLPTR));
        label_8->setText(QApplication::translate("DynamicTrackDlg", "\347\254\2548\344\270\252\347\274\226\347\240\201\347\202\271\345\235\220\346\240\207\357\274\232", Q_NULLPTR));
        label_9->setText(QApplication::translate("DynamicTrackDlg", "\347\254\2549\344\270\252\347\274\226\347\240\201\347\202\271\345\235\220\346\240\207\357\274\232", Q_NULLPTR));
        label_10->setText(QApplication::translate("DynamicTrackDlg", "\347\254\25410\344\270\252\347\274\226\347\240\201\347\202\271\345\235\220\346\240\207\357\274\232", Q_NULLPTR));
        label_11->setText(QApplication::translate("DynamicTrackDlg", "\347\254\25411\344\270\252\347\274\226\347\240\201\347\202\271\345\235\220\346\240\207\357\274\232", Q_NULLPTR));
        label_12->setText(QApplication::translate("DynamicTrackDlg", "\347\254\25412\344\270\252\347\274\226\347\240\201\347\202\271\345\235\220\346\240\207\357\274\232", Q_NULLPTR));
        label_13->setText(QApplication::translate("DynamicTrackDlg", "\347\254\25413\344\270\252\347\274\226\347\240\201\347\202\271\345\235\220\346\240\207\357\274\232", Q_NULLPTR));
        label_14->setText(QApplication::translate("DynamicTrackDlg", "\347\254\25414\344\270\252\347\274\226\347\240\201\347\202\271\345\235\220\346\240\207\357\274\232", Q_NULLPTR));
        label_16->setText(QApplication::translate("DynamicTrackDlg", "\345\233\276\345\203\217\346\200\273\346\225\260:", Q_NULLPTR));
        label->setText(QApplication::translate("DynamicTrackDlg", "\345\233\276\345\203\217\345\272\217\345\217\267:", Q_NULLPTR));
        btnSetTrackPath->setText(QApplication::translate("DynamicTrackDlg", "\350\256\276\347\275\256\350\267\237\350\270\252\347\232\204\345\233\276\347\211\207\350\267\257\345\276\204", Q_NULLPTR));
        btnToTrack->setText(QApplication::translate("DynamicTrackDlg", "\350\267\237\350\270\252", Q_NULLPTR));
        btnStopTracking->setText(QApplication::translate("DynamicTrackDlg", "\345\201\234\346\255\242", Q_NULLPTR));
    } // retranslateUi

};

namespace Ui {
    class DynamicTrackDlg: public Ui_DynamicTrackDlg {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_DYNAMICTRACKDLG_H
