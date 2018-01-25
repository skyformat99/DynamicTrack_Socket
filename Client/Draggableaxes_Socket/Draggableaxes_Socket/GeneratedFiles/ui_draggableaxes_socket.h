/********************************************************************************
** Form generated from reading UI file 'draggableaxes_socket.ui'
**
** Created by: Qt User Interface Compiler version 5.7.1
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_DRAGGABLEAXES_SOCKET_H
#define UI_DRAGGABLEAXES_SOCKET_H

#include <QtCore/QVariant>
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QButtonGroup>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QMenuBar>
#include <QtWidgets/QStatusBar>
#include <QtWidgets/QToolBar>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_Draggableaxes_SocketClass
{
public:
    QMenuBar *menuBar;
    QToolBar *mainToolBar;
    QWidget *centralWidget;
    QStatusBar *statusBar;

    void setupUi(QMainWindow *Draggableaxes_SocketClass)
    {
        if (Draggableaxes_SocketClass->objectName().isEmpty())
            Draggableaxes_SocketClass->setObjectName(QStringLiteral("Draggableaxes_SocketClass"));
        Draggableaxes_SocketClass->resize(600, 400);
        menuBar = new QMenuBar(Draggableaxes_SocketClass);
        menuBar->setObjectName(QStringLiteral("menuBar"));
        Draggableaxes_SocketClass->setMenuBar(menuBar);
        mainToolBar = new QToolBar(Draggableaxes_SocketClass);
        mainToolBar->setObjectName(QStringLiteral("mainToolBar"));
        Draggableaxes_SocketClass->addToolBar(mainToolBar);
        centralWidget = new QWidget(Draggableaxes_SocketClass);
        centralWidget->setObjectName(QStringLiteral("centralWidget"));
        Draggableaxes_SocketClass->setCentralWidget(centralWidget);
        statusBar = new QStatusBar(Draggableaxes_SocketClass);
        statusBar->setObjectName(QStringLiteral("statusBar"));
        Draggableaxes_SocketClass->setStatusBar(statusBar);

        retranslateUi(Draggableaxes_SocketClass);

        QMetaObject::connectSlotsByName(Draggableaxes_SocketClass);
    } // setupUi

    void retranslateUi(QMainWindow *Draggableaxes_SocketClass)
    {
        Draggableaxes_SocketClass->setWindowTitle(QApplication::translate("Draggableaxes_SocketClass", "Draggableaxes_Socket", Q_NULLPTR));
    } // retranslateUi

};

namespace Ui {
    class Draggableaxes_SocketClass: public Ui_Draggableaxes_SocketClass {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_DRAGGABLEAXES_SOCKET_H
