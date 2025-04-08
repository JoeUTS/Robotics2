/********************************************************************************
** Form generated from reading UI file 'mainwindow.ui'
**
** Created by: Qt User Interface Compiler version 5.15.3
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_MAINWINDOW_H
#define UI_MAINWINDOW_H

#include <QtCore/QVariant>
#include <QtWidgets/QApplication>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QMenuBar>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QStatusBar>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_MainWindow
{
public:
    QWidget *centralwidget;
    QWidget *widget;
    QPushButton *startCamera;
    QWidget *viewfinderPlaceholder;
    QPushButton *captureImage;
    QMenuBar *menubar;
    QStatusBar *statusbar;

    void setupUi(QMainWindow *MainWindow)
    {
        if (MainWindow->objectName().isEmpty())
            MainWindow->setObjectName(QString::fromUtf8("MainWindow"));
        MainWindow->resize(800, 600);
        centralwidget = new QWidget(MainWindow);
        centralwidget->setObjectName(QString::fromUtf8("centralwidget"));
        widget = new QWidget(centralwidget);
        widget->setObjectName(QString::fromUtf8("widget"));
        widget->setGeometry(QRect(0, 0, 231, 221));
        widget->setAutoFillBackground(false);
        widget->setStyleSheet(QString::fromUtf8("border: 1px solid black;\n"
"background-color: rgb(191, 205, 206);\n"
"border-radius: 10px;"));
        startCamera = new QPushButton(widget);
        startCamera->setObjectName(QString::fromUtf8("startCamera"));
        startCamera->setGeometry(QRect(10, 0, 121, 31));
        startCamera->setStyleSheet(QString::fromUtf8("background: white;"));
        viewfinderPlaceholder = new QWidget(widget);
        viewfinderPlaceholder->setObjectName(QString::fromUtf8("viewfinderPlaceholder"));
        viewfinderPlaceholder->setGeometry(QRect(10, 40, 211, 131));
        viewfinderPlaceholder->setStyleSheet(QString::fromUtf8("background: black;"));
        captureImage = new QPushButton(widget);
        captureImage->setObjectName(QString::fromUtf8("captureImage"));
        captureImage->setGeometry(QRect(10, 180, 121, 31));
        captureImage->setStyleSheet(QString::fromUtf8("background: white;"));
        MainWindow->setCentralWidget(centralwidget);
        menubar = new QMenuBar(MainWindow);
        menubar->setObjectName(QString::fromUtf8("menubar"));
        menubar->setGeometry(QRect(0, 0, 800, 20));
        MainWindow->setMenuBar(menubar);
        statusbar = new QStatusBar(MainWindow);
        statusbar->setObjectName(QString::fromUtf8("statusbar"));
        MainWindow->setStatusBar(statusbar);

        retranslateUi(MainWindow);

        QMetaObject::connectSlotsByName(MainWindow);
    } // setupUi

    void retranslateUi(QMainWindow *MainWindow)
    {
        MainWindow->setWindowTitle(QCoreApplication::translate("MainWindow", "MainWindow", nullptr));
        startCamera->setText(QCoreApplication::translate("MainWindow", "Start Camera", nullptr));
        captureImage->setText(QCoreApplication::translate("MainWindow", "Capture Image", nullptr));
    } // retranslateUi

};

namespace Ui {
    class MainWindow: public Ui_MainWindow {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MAINWINDOW_H
