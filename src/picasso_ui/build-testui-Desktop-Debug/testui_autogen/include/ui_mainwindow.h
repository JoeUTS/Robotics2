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
#include <QtWidgets/QSlider>
#include <QtWidgets/QStatusBar>
#include <QtWidgets/QWidget>
#include "qwt_text_label.h"

QT_BEGIN_NAMESPACE

class Ui_MainWindow
{
public:
    QWidget *centralwidget;
    QWidget *widget;
    QPushButton *startCamera;
    QWidget *viewfinderPlaceholder;
    QPushButton *captureImage;
    QPushButton *captureImage_2;
    QWidget *widget_2;
    QPushButton *eStopButton;
    QPushButton *eStopButton_2;
    QSlider *horizontalSlider;
    QwtTextLabel *adjustSpeed;
    QWidget *widget_3;
    QPushButton *startCamera_2;
    QWidget *viewfinderPlaceholder_2;
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
        widget->setGeometry(QRect(0, 0, 231, 271));
        widget->setAutoFillBackground(false);
        widget->setStyleSheet(QString::fromUtf8("border: 1px solid black;\n"
"background-color: rgb(191, 205, 206);\n"
"border-radius: 10px;"));
        startCamera = new QPushButton(widget);
        startCamera->setObjectName(QString::fromUtf8("startCamera"));
        startCamera->setGeometry(QRect(10, 10, 121, 31));
        startCamera->setStyleSheet(QString::fromUtf8("background: white;"));
        viewfinderPlaceholder = new QWidget(widget);
        viewfinderPlaceholder->setObjectName(QString::fromUtf8("viewfinderPlaceholder"));
        viewfinderPlaceholder->setGeometry(QRect(10, 50, 211, 131));
        viewfinderPlaceholder->setStyleSheet(QString::fromUtf8("background: black;"));
        captureImage = new QPushButton(widget);
        captureImage->setObjectName(QString::fromUtf8("captureImage"));
        captureImage->setGeometry(QRect(10, 190, 121, 31));
        captureImage->setStyleSheet(QString::fromUtf8("background: white;"));
        captureImage_2 = new QPushButton(widget);
        captureImage_2->setObjectName(QString::fromUtf8("captureImage_2"));
        captureImage_2->setGeometry(QRect(10, 230, 121, 31));
        captureImage_2->setStyleSheet(QString::fromUtf8("background: white;"));
        widget_2 = new QWidget(centralwidget);
        widget_2->setObjectName(QString::fromUtf8("widget_2"));
        widget_2->setGeometry(QRect(460, 0, 231, 271));
        widget_2->setAutoFillBackground(false);
        widget_2->setStyleSheet(QString::fromUtf8("border: 1px solid black;\n"
"background-color: rgb(191, 205, 206);\n"
"border-radius: 10px;"));
        eStopButton = new QPushButton(widget_2);
        eStopButton->setObjectName(QString::fromUtf8("eStopButton"));
        eStopButton->setGeometry(QRect(10, 10, 121, 31));
        eStopButton->setStyleSheet(QString::fromUtf8("background: white;"));
        eStopButton_2 = new QPushButton(widget_2);
        eStopButton_2->setObjectName(QString::fromUtf8("eStopButton_2"));
        eStopButton_2->setGeometry(QRect(10, 90, 121, 31));
        eStopButton_2->setStyleSheet(QString::fromUtf8("background: qlineargradient(spread:pad, x1:0.352632, y1:0.324, x2:0.379, y2:1, stop:0 rgba(255, 0, 0, 255), stop:1 rgba(255, 255, 255, 255))"));
        horizontalSlider = new QSlider(widget_2);
        horizontalSlider->setObjectName(QString::fromUtf8("horizontalSlider"));
        horizontalSlider->setGeometry(QRect(20, 220, 160, 16));
        horizontalSlider->setOrientation(Qt::Horizontal);
        adjustSpeed = new QwtTextLabel(widget_2);
        adjustSpeed->setObjectName(QString::fromUtf8("adjustSpeed"));
        adjustSpeed->setGeometry(QRect(20, 190, 161, 20));
        widget_3 = new QWidget(centralwidget);
        widget_3->setObjectName(QString::fromUtf8("widget_3"));
        widget_3->setGeometry(QRect(230, 0, 231, 271));
        widget_3->setAutoFillBackground(false);
        widget_3->setStyleSheet(QString::fromUtf8("border: 1px solid black;\n"
"background-color: rgb(191, 205, 206);\n"
"border-radius: 10px;"));
        startCamera_2 = new QPushButton(widget_3);
        startCamera_2->setObjectName(QString::fromUtf8("startCamera_2"));
        startCamera_2->setGeometry(QRect(10, 10, 121, 31));
        startCamera_2->setStyleSheet(QString::fromUtf8("background: white;"));
        viewfinderPlaceholder_2 = new QWidget(widget_3);
        viewfinderPlaceholder_2->setObjectName(QString::fromUtf8("viewfinderPlaceholder_2"));
        viewfinderPlaceholder_2->setGeometry(QRect(10, 50, 211, 131));
        viewfinderPlaceholder_2->setStyleSheet(QString::fromUtf8("background: black;"));
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
        captureImage_2->setText(QCoreApplication::translate("MainWindow", "Process Image", nullptr));
        eStopButton->setText(QCoreApplication::translate("MainWindow", "Start Drawing", nullptr));
        eStopButton_2->setText(QCoreApplication::translate("MainWindow", "Emergency Stop", nullptr));
        adjustSpeed->setPlainText(QCoreApplication::translate("MainWindow", "Adjust Robot Speed", nullptr));
        startCamera_2->setText(QCoreApplication::translate("MainWindow", "Preview Sketch", nullptr));
    } // retranslateUi

};

namespace Ui {
    class MainWindow: public Ui_MainWindow {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MAINWINDOW_H
