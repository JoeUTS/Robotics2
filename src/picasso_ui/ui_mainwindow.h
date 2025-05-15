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
#include <QtWidgets/QHBoxLayout>
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
    QWidget *horizontalLayoutWidget;
    QHBoxLayout *horizontalLayout;
    QWidget *widget;
    QPushButton *startCamera;
    QWidget *viewfinderPlaceholder;
    QPushButton *captureImage;
    QPushButton *processImage;
    QPushButton *connectUR3;
    QWidget *widget_2;
    QPushButton *previewSketch;
    QWidget *viewfinderPlaceholder_2;
    QPushButton *discardImage;
    QPushButton *generateToolpath;
    QWidget *widget_3;
    QPushButton *startDrawing;
    QPushButton *eStopButton;
    QMenuBar *menubar;
    QStatusBar *statusbar;

    void setupUi(QMainWindow *MainWindow)
    {
        if (MainWindow->objectName().isEmpty())
            MainWindow->setObjectName(QString::fromUtf8("MainWindow"));
        MainWindow->resize(1038, 643);
        centralwidget = new QWidget(MainWindow);
        centralwidget->setObjectName(QString::fromUtf8("centralwidget"));
        horizontalLayoutWidget = new QWidget(centralwidget);
        horizontalLayoutWidget->setObjectName(QString::fromUtf8("horizontalLayoutWidget"));
        horizontalLayoutWidget->setGeometry(QRect(60, 20, 791, 311));
        horizontalLayout = new QHBoxLayout(horizontalLayoutWidget);
        horizontalLayout->setObjectName(QString::fromUtf8("horizontalLayout"));
        horizontalLayout->setContentsMargins(0, 0, 0, 0);
        widget = new QWidget(horizontalLayoutWidget);
        widget->setObjectName(QString::fromUtf8("widget"));
        widget->setAutoFillBackground(false);
        widget->setStyleSheet(QString::fromUtf8("border: 1px solid black;\n"
"background-color: rgb(191, 205, 206);\n"
"border-radius: 10px;"));
        startCamera = new QPushButton(widget);
        startCamera->setObjectName(QString::fromUtf8("startCamera"));
        startCamera->setGeometry(QRect(10, 10, 111, 31));
        startCamera->setStyleSheet(QString::fromUtf8("background: white;"));
        viewfinderPlaceholder = new QWidget(widget);
        viewfinderPlaceholder->setObjectName(QString::fromUtf8("viewfinderPlaceholder"));
        viewfinderPlaceholder->setGeometry(QRect(10, 50, 241, 131));
        viewfinderPlaceholder->setStyleSheet(QString::fromUtf8("background: black;"));
        captureImage = new QPushButton(widget);
        captureImage->setObjectName(QString::fromUtf8("captureImage"));
        captureImage->setGeometry(QRect(10, 190, 121, 31));
        captureImage->setStyleSheet(QString::fromUtf8("background: white;"));
        processImage = new QPushButton(widget);
        processImage->setObjectName(QString::fromUtf8("processImage"));
        processImage->setGeometry(QRect(10, 230, 121, 31));
        processImage->setStyleSheet(QString::fromUtf8("background: white;"));
        connectUR3 = new QPushButton(widget);
        connectUR3->setObjectName(QString::fromUtf8("connectUR3"));
        connectUR3->setGeometry(QRect(130, 10, 91, 31));
        connectUR3->setStyleSheet(QString::fromUtf8("background: white;"));

        horizontalLayout->addWidget(widget);

        widget_2 = new QWidget(horizontalLayoutWidget);
        widget_2->setObjectName(QString::fromUtf8("widget_2"));
        widget_2->setAutoFillBackground(false);
        widget_2->setStyleSheet(QString::fromUtf8("border: 1px solid black;\n"
"background-color: rgb(191, 205, 206);\n"
"border-radius: 10px;"));
        previewSketch = new QPushButton(widget_2);
        previewSketch->setObjectName(QString::fromUtf8("previewSketch"));
        previewSketch->setGeometry(QRect(10, 10, 121, 31));
        previewSketch->setStyleSheet(QString::fromUtf8("background: white;"));
        viewfinderPlaceholder_2 = new QWidget(widget_2);
        viewfinderPlaceholder_2->setObjectName(QString::fromUtf8("viewfinderPlaceholder_2"));
        viewfinderPlaceholder_2->setGeometry(QRect(10, 50, 241, 131));
        viewfinderPlaceholder_2->setStyleSheet(QString::fromUtf8("background: black;"));
        discardImage = new QPushButton(widget_2);
        discardImage->setObjectName(QString::fromUtf8("discardImage"));
        discardImage->setGeometry(QRect(10, 190, 121, 31));
        discardImage->setStyleSheet(QString::fromUtf8("background: white;"));
        generateToolpath = new QPushButton(widget_2);
        generateToolpath->setObjectName(QString::fromUtf8("generateToolpath"));
        generateToolpath->setGeometry(QRect(10, 230, 121, 31));
        generateToolpath->setStyleSheet(QString::fromUtf8("background: white;"));

        horizontalLayout->addWidget(widget_2);

        widget_3 = new QWidget(horizontalLayoutWidget);
        widget_3->setObjectName(QString::fromUtf8("widget_3"));
        widget_3->setAutoFillBackground(false);
        widget_3->setStyleSheet(QString::fromUtf8("border: 1px solid black;\n"
"background-color: rgb(191, 205, 206);\n"
"border-radius: 10px;"));
        startDrawing = new QPushButton(widget_3);
        startDrawing->setObjectName(QString::fromUtf8("startDrawing"));
        startDrawing->setGeometry(QRect(10, 10, 121, 31));
        startDrawing->setStyleSheet(QString::fromUtf8("background: white;"));
        eStopButton = new QPushButton(widget_3);
        eStopButton->setObjectName(QString::fromUtf8("eStopButton"));
        eStopButton->setGeometry(QRect(10, 90, 121, 31));
        eStopButton->setStyleSheet(QString::fromUtf8("background: qlineargradient(spread:pad, x1:0.352632, y1:0.324, x2:0.379, y2:1, stop:0 rgba(255, 0, 0, 255), stop:1 rgba(255, 255, 255, 255))"));

        horizontalLayout->addWidget(widget_3);

        MainWindow->setCentralWidget(centralwidget);
        menubar = new QMenuBar(MainWindow);
        menubar->setObjectName(QString::fromUtf8("menubar"));
        menubar->setGeometry(QRect(0, 0, 1038, 20));
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
        startCamera->setText(QCoreApplication::translate("MainWindow", "Connect Camera", nullptr));
        captureImage->setText(QCoreApplication::translate("MainWindow", "Capture Image", nullptr));
        processImage->setText(QCoreApplication::translate("MainWindow", "Process Image", nullptr));
        connectUR3->setText(QCoreApplication::translate("MainWindow", "Connect UR3", nullptr));
        previewSketch->setText(QCoreApplication::translate("MainWindow", "Preview Sketch", nullptr));
        discardImage->setText(QCoreApplication::translate("MainWindow", "Discard Image", nullptr));
        generateToolpath->setText(QCoreApplication::translate("MainWindow", "Generate Toolpath", nullptr));
        startDrawing->setText(QCoreApplication::translate("MainWindow", "Start Drawing", nullptr));
        eStopButton->setText(QCoreApplication::translate("MainWindow", "Emergency Stop", nullptr));
    } // retranslateUi

};

namespace Ui {
    class MainWindow: public Ui_MainWindow {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MAINWINDOW_H
