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
#include <QtWidgets/QGridLayout>
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
    QWidget *gridLayoutWidget;
    QGridLayout *gridLayout_5;
    QHBoxLayout *horizontalLayout_5;
    QPushButton *processImage;
    QPushButton *captureImage;
    QWidget *viewfinderPlaceholder;
    QHBoxLayout *horizontalLayout_3;
    QPushButton *connectUR3;
    QPushButton *startCamera;
    QWidget *widget_2;
    QWidget *gridLayoutWidget_2;
    QGridLayout *gridLayout_6;
    QHBoxLayout *horizontalLayout_6;
    QPushButton *discardImage;
    QPushButton *generateToolpath;
    QWidget *viewfinderPlaceholder_2;
    QHBoxLayout *horizontalLayout_2;
    QPushButton *previewSketch;
    QWidget *widget_3;
    QWidget *gridLayoutWidget_3;
    QGridLayout *gridLayout;
    QPushButton *startDrawing;
    QPushButton *eStopButton;
    QMenuBar *menubar;
    QStatusBar *statusbar;

    void setupUi(QMainWindow *MainWindow)
    {
        if (MainWindow->objectName().isEmpty())
            MainWindow->setObjectName(QString::fromUtf8("MainWindow"));
        MainWindow->resize(1339, 575);
        MainWindow->setMinimumSize(QSize(400, 400));
        centralwidget = new QWidget(MainWindow);
        centralwidget->setObjectName(QString::fromUtf8("centralwidget"));
        centralwidget->setMinimumSize(QSize(1339, 531));
        horizontalLayoutWidget = new QWidget(centralwidget);
        horizontalLayoutWidget->setObjectName(QString::fromUtf8("horizontalLayoutWidget"));
        horizontalLayoutWidget->setGeometry(QRect(60, 20, 1261, 491));
        horizontalLayout = new QHBoxLayout(horizontalLayoutWidget);
        horizontalLayout->setSpacing(0);
        horizontalLayout->setObjectName(QString::fromUtf8("horizontalLayout"));
        horizontalLayout->setContentsMargins(0, 0, 0, 0);
        widget = new QWidget(horizontalLayoutWidget);
        widget->setObjectName(QString::fromUtf8("widget"));
        widget->setAutoFillBackground(false);
        widget->setStyleSheet(QString::fromUtf8("border: 1px solid black;\n"
"background-color: rgb(191, 205, 206);\n"
"border-radius: 10px;"));
        gridLayoutWidget = new QWidget(widget);
        gridLayoutWidget->setObjectName(QString::fromUtf8("gridLayoutWidget"));
        gridLayoutWidget->setGeometry(QRect(10, 20, 481, 441));
        gridLayout_5 = new QGridLayout(gridLayoutWidget);
        gridLayout_5->setObjectName(QString::fromUtf8("gridLayout_5"));
        gridLayout_5->setSizeConstraint(QLayout::SetNoConstraint);
        gridLayout_5->setHorizontalSpacing(0);
        gridLayout_5->setVerticalSpacing(20);
        gridLayout_5->setContentsMargins(0, 0, 0, 0);
        horizontalLayout_5 = new QHBoxLayout();
        horizontalLayout_5->setObjectName(QString::fromUtf8("horizontalLayout_5"));
        processImage = new QPushButton(gridLayoutWidget);
        processImage->setObjectName(QString::fromUtf8("processImage"));
        processImage->setStyleSheet(QString::fromUtf8("background: white;"));

        horizontalLayout_5->addWidget(processImage);

        captureImage = new QPushButton(gridLayoutWidget);
        captureImage->setObjectName(QString::fromUtf8("captureImage"));
        captureImage->setStyleSheet(QString::fromUtf8("background: white;"));

        horizontalLayout_5->addWidget(captureImage);


        gridLayout_5->addLayout(horizontalLayout_5, 4, 0, 1, 1);

        viewfinderPlaceholder = new QWidget(gridLayoutWidget);
        viewfinderPlaceholder->setObjectName(QString::fromUtf8("viewfinderPlaceholder"));
        viewfinderPlaceholder->setStyleSheet(QString::fromUtf8("background: black;"));

        gridLayout_5->addWidget(viewfinderPlaceholder, 1, 0, 1, 1);

        horizontalLayout_3 = new QHBoxLayout();
        horizontalLayout_3->setObjectName(QString::fromUtf8("horizontalLayout_3"));
        connectUR3 = new QPushButton(gridLayoutWidget);
        connectUR3->setObjectName(QString::fromUtf8("connectUR3"));
        connectUR3->setStyleSheet(QString::fromUtf8("background: white;"));

        horizontalLayout_3->addWidget(connectUR3);

        startCamera = new QPushButton(gridLayoutWidget);
        startCamera->setObjectName(QString::fromUtf8("startCamera"));
        QFont font;
        font.setPointSize(10);
        startCamera->setFont(font);
        startCamera->setStyleSheet(QString::fromUtf8("background: white;"));

        horizontalLayout_3->addWidget(startCamera);


        gridLayout_5->addLayout(horizontalLayout_3, 0, 0, 1, 1);

        gridLayout_5->setRowStretch(0, 1);
        gridLayout_5->setRowStretch(1, 4);
        gridLayout_5->setRowStretch(4, 1);

        horizontalLayout->addWidget(widget);

        widget_2 = new QWidget(horizontalLayoutWidget);
        widget_2->setObjectName(QString::fromUtf8("widget_2"));
        widget_2->setAutoFillBackground(false);
        widget_2->setStyleSheet(QString::fromUtf8("border: 1px solid black;\n"
"background-color: rgb(156, 182, 224);\n"
"border-radius: 10px;"));
        gridLayoutWidget_2 = new QWidget(widget_2);
        gridLayoutWidget_2->setObjectName(QString::fromUtf8("gridLayoutWidget_2"));
        gridLayoutWidget_2->setGeometry(QRect(10, 20, 481, 441));
        gridLayout_6 = new QGridLayout(gridLayoutWidget_2);
        gridLayout_6->setObjectName(QString::fromUtf8("gridLayout_6"));
        gridLayout_6->setSizeConstraint(QLayout::SetNoConstraint);
        gridLayout_6->setHorizontalSpacing(0);
        gridLayout_6->setVerticalSpacing(20);
        gridLayout_6->setContentsMargins(0, 0, 0, 0);
        horizontalLayout_6 = new QHBoxLayout();
        horizontalLayout_6->setObjectName(QString::fromUtf8("horizontalLayout_6"));
        discardImage = new QPushButton(gridLayoutWidget_2);
        discardImage->setObjectName(QString::fromUtf8("discardImage"));
        discardImage->setStyleSheet(QString::fromUtf8("background: white;"));

        horizontalLayout_6->addWidget(discardImage);

        generateToolpath = new QPushButton(gridLayoutWidget_2);
        generateToolpath->setObjectName(QString::fromUtf8("generateToolpath"));
        generateToolpath->setFont(font);
        generateToolpath->setStyleSheet(QString::fromUtf8("background: white;"));

        horizontalLayout_6->addWidget(generateToolpath);


        gridLayout_6->addLayout(horizontalLayout_6, 5, 0, 1, 1);

        viewfinderPlaceholder_2 = new QWidget(gridLayoutWidget_2);
        viewfinderPlaceholder_2->setObjectName(QString::fromUtf8("viewfinderPlaceholder_2"));
        viewfinderPlaceholder_2->setStyleSheet(QString::fromUtf8("background: black;"));

        gridLayout_6->addWidget(viewfinderPlaceholder_2, 2, 0, 1, 1);

        horizontalLayout_2 = new QHBoxLayout();
        horizontalLayout_2->setObjectName(QString::fromUtf8("horizontalLayout_2"));
        previewSketch = new QPushButton(gridLayoutWidget_2);
        previewSketch->setObjectName(QString::fromUtf8("previewSketch"));
        previewSketch->setStyleSheet(QString::fromUtf8("background: white;"));

        horizontalLayout_2->addWidget(previewSketch);


        gridLayout_6->addLayout(horizontalLayout_2, 0, 0, 1, 1);

        gridLayout_6->setRowStretch(0, 1);
        gridLayout_6->setRowStretch(2, 4);
        gridLayout_6->setRowStretch(5, 1);

        horizontalLayout->addWidget(widget_2);

        widget_3 = new QWidget(horizontalLayoutWidget);
        widget_3->setObjectName(QString::fromUtf8("widget_3"));
        widget_3->setAutoFillBackground(false);
        widget_3->setStyleSheet(QString::fromUtf8("border: 1px solid black;\n"
"background-color: rgb(191, 205, 206);\n"
"border-radius: 10px;"));
        gridLayoutWidget_3 = new QWidget(widget_3);
        gridLayoutWidget_3->setObjectName(QString::fromUtf8("gridLayoutWidget_3"));
        gridLayoutWidget_3->setGeometry(QRect(20, 50, 201, 321));
        gridLayout = new QGridLayout(gridLayoutWidget_3);
        gridLayout->setObjectName(QString::fromUtf8("gridLayout"));
        gridLayout->setContentsMargins(0, 0, 0, 0);
        startDrawing = new QPushButton(gridLayoutWidget_3);
        startDrawing->setObjectName(QString::fromUtf8("startDrawing"));
        startDrawing->setStyleSheet(QString::fromUtf8("background: white;"));

        gridLayout->addWidget(startDrawing, 0, 0, 1, 1);

        eStopButton = new QPushButton(gridLayoutWidget_3);
        eStopButton->setObjectName(QString::fromUtf8("eStopButton"));
        eStopButton->setStyleSheet(QString::fromUtf8("background: qlineargradient(spread:pad, x1:0.352632, y1:0.324, x2:0.379, y2:1, stop:0 rgba(255, 0, 0, 255), stop:1 rgba(255, 255, 255, 255))"));

        gridLayout->addWidget(eStopButton, 1, 0, 1, 1);


        horizontalLayout->addWidget(widget_3);

        horizontalLayout->setStretch(0, 2);
        horizontalLayout->setStretch(1, 2);
        horizontalLayout->setStretch(2, 1);
        MainWindow->setCentralWidget(centralwidget);
        menubar = new QMenuBar(MainWindow);
        menubar->setObjectName(QString::fromUtf8("menubar"));
        menubar->setGeometry(QRect(0, 0, 1339, 22));
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
        processImage->setText(QCoreApplication::translate("MainWindow", "Process Image", nullptr));
        captureImage->setText(QCoreApplication::translate("MainWindow", "Capture Image", nullptr));
        connectUR3->setText(QCoreApplication::translate("MainWindow", "Connect UR3", nullptr));
        startCamera->setText(QCoreApplication::translate("MainWindow", "Connect Camera", nullptr));
        discardImage->setText(QCoreApplication::translate("MainWindow", "Discard Image", nullptr));
        generateToolpath->setText(QCoreApplication::translate("MainWindow", "Generate Toolpath", nullptr));
        previewSketch->setText(QCoreApplication::translate("MainWindow", "Preview Sketch", nullptr));
        startDrawing->setText(QCoreApplication::translate("MainWindow", "Start Drawing", nullptr));
        eStopButton->setText(QCoreApplication::translate("MainWindow", "Emergency Stop", nullptr));
    } // retranslateUi

};

namespace Ui {
    class MainWindow: public Ui_MainWindow {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MAINWINDOW_H
