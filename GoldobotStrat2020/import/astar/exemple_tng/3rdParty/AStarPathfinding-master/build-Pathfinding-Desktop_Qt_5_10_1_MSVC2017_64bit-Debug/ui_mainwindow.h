/********************************************************************************
** Form generated from reading UI file 'mainwindow.ui'
**
** Created by: Qt User Interface Compiler version 5.10.1
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_MAINWINDOW_H
#define UI_MAINWINDOW_H

#include <QtCore/QVariant>
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QButtonGroup>
#include <QtWidgets/QGraphicsView>
#include <QtWidgets/QGroupBox>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QLabel>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QMenuBar>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QRadioButton>
#include <QtWidgets/QSpinBox>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_MainWindow
{
public:
    QWidget *centralWidget;
    QGraphicsView *viewport;
    QGroupBox *functionsGroupBox;
    QSpinBox *spinMapWidth;
    QSpinBox *spinMapHeight;
    QLabel *lblSize;
    QPushButton *btnBuildMap;
    QPushButton *btnShowPath;
    QPushButton *btnShowStep;
    QPushButton *btnShowSteps;
    QRadioButton *radioBtnEuclidean;
    QRadioButton *radioBtnManhattan;
    QLabel *lblScale;
    QSpinBox *spinMapScale;
    QPushButton *btnStart;
    QLabel *lblStart;
    QPushButton *btnEnd;
    QLabel *lblEnd;
    QPushButton *btnWay;
    QPushButton *btnWall;
    QLabel *lblWay;
    QLabel *lblWall;
    QPushButton *btnDirt;
    QLabel *lblDirt;
    QMenuBar *menuBar;

    void setupUi(QMainWindow *MainWindow)
    {
        if (MainWindow->objectName().isEmpty())
            MainWindow->setObjectName(QStringLiteral("MainWindow"));
        MainWindow->resize(600, 450);
        MainWindow->setMinimumSize(QSize(600, 450));
        MainWindow->setMaximumSize(QSize(600, 450));
        MainWindow->setContextMenuPolicy(Qt::NoContextMenu);
        centralWidget = new QWidget(MainWindow);
        centralWidget->setObjectName(QStringLiteral("centralWidget"));
        viewport = new QGraphicsView(centralWidget);
        viewport->setObjectName(QStringLiteral("viewport"));
        viewport->setGeometry(QRect(170, 10, 421, 411));
        viewport->setSceneRect(QRectF(0, 0, 0, 0));
        viewport->setAlignment(Qt::AlignLeading|Qt::AlignLeft|Qt::AlignTop);
        functionsGroupBox = new QGroupBox(centralWidget);
        functionsGroupBox->setObjectName(QStringLiteral("functionsGroupBox"));
        functionsGroupBox->setGeometry(QRect(10, 10, 151, 321));
        QFont font;
        font.setPointSize(8);
        functionsGroupBox->setFont(font);
        spinMapWidth = new QSpinBox(functionsGroupBox);
        spinMapWidth->setObjectName(QStringLiteral("spinMapWidth"));
        spinMapWidth->setGeometry(QRect(10, 40, 61, 22));
        QFont font1;
        font1.setPointSize(10);
        spinMapWidth->setFont(font1);
        spinMapWidth->setMaximum(100);
        spinMapWidth->setValue(25);
        spinMapHeight = new QSpinBox(functionsGroupBox);
        spinMapHeight->setObjectName(QStringLiteral("spinMapHeight"));
        spinMapHeight->setGeometry(QRect(80, 40, 61, 22));
        spinMapHeight->setFont(font1);
        spinMapHeight->setMaximum(100);
        spinMapHeight->setValue(25);
        lblSize = new QLabel(functionsGroupBox);
        lblSize->setObjectName(QStringLiteral("lblSize"));
        lblSize->setGeometry(QRect(10, 20, 46, 13));
        lblSize->setFont(font1);
        lblSize->setTextFormat(Qt::PlainText);
        btnBuildMap = new QPushButton(functionsGroupBox);
        btnBuildMap->setObjectName(QStringLiteral("btnBuildMap"));
        btnBuildMap->setGeometry(QRect(60, 10, 81, 21));
        btnBuildMap->setFont(font1);
        btnShowPath = new QPushButton(functionsGroupBox);
        btnShowPath->setObjectName(QStringLiteral("btnShowPath"));
        btnShowPath->setEnabled(false);
        btnShowPath->setGeometry(QRect(10, 100, 131, 23));
        btnShowPath->setFont(font1);
        btnShowStep = new QPushButton(functionsGroupBox);
        btnShowStep->setObjectName(QStringLiteral("btnShowStep"));
        btnShowStep->setEnabled(false);
        btnShowStep->setGeometry(QRect(10, 130, 131, 23));
        btnShowStep->setFont(font1);
        btnShowSteps = new QPushButton(functionsGroupBox);
        btnShowSteps->setObjectName(QStringLiteral("btnShowSteps"));
        btnShowSteps->setEnabled(false);
        btnShowSteps->setGeometry(QRect(10, 160, 131, 23));
        btnShowSteps->setFont(font1);
        radioBtnEuclidean = new QRadioButton(functionsGroupBox);
        radioBtnEuclidean->setObjectName(QStringLiteral("radioBtnEuclidean"));
        radioBtnEuclidean->setGeometry(QRect(10, 190, 82, 17));
        radioBtnEuclidean->setCheckable(true);
        radioBtnEuclidean->setChecked(true);
        radioBtnManhattan = new QRadioButton(functionsGroupBox);
        radioBtnManhattan->setObjectName(QStringLiteral("radioBtnManhattan"));
        radioBtnManhattan->setGeometry(QRect(10, 210, 82, 17));
        radioBtnManhattan->setCheckable(true);
        radioBtnManhattan->setChecked(false);
        lblScale = new QLabel(functionsGroupBox);
        lblScale->setObjectName(QStringLiteral("lblScale"));
        lblScale->setGeometry(QRect(10, 80, 46, 13));
        lblScale->setFont(font1);
        lblScale->setTextFormat(Qt::PlainText);
        spinMapScale = new QSpinBox(functionsGroupBox);
        spinMapScale->setObjectName(QStringLiteral("spinMapScale"));
        spinMapScale->setGeometry(QRect(60, 70, 61, 22));
        spinMapScale->setFont(font1);
        spinMapScale->setMaximum(4);
        spinMapScale->setValue(2);
        btnStart = new QPushButton(functionsGroupBox);
        btnStart->setObjectName(QStringLiteral("btnStart"));
        btnStart->setGeometry(QRect(10, 230, 21, 21));
        QIcon icon;
        icon.addFile(QStringLiteral(":/images/images/startButton.png"), QSize(), QIcon::Normal, QIcon::Off);
        btnStart->setIcon(icon);
        btnStart->setIconSize(QSize(16, 16));
        lblStart = new QLabel(functionsGroupBox);
        lblStart->setObjectName(QStringLiteral("lblStart"));
        lblStart->setGeometry(QRect(40, 230, 31, 16));
        btnEnd = new QPushButton(functionsGroupBox);
        btnEnd->setObjectName(QStringLiteral("btnEnd"));
        btnEnd->setGeometry(QRect(120, 230, 21, 21));
        QIcon icon1;
        icon1.addFile(QStringLiteral(":/images/images/endButton.png"), QSize(), QIcon::Normal, QIcon::Off);
        btnEnd->setIcon(icon1);
        btnEnd->setIconSize(QSize(16, 16));
        lblEnd = new QLabel(functionsGroupBox);
        lblEnd->setObjectName(QStringLiteral("lblEnd"));
        lblEnd->setGeometry(QRect(80, 230, 31, 16));
        lblEnd->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        btnWay = new QPushButton(functionsGroupBox);
        btnWay->setObjectName(QStringLiteral("btnWay"));
        btnWay->setGeometry(QRect(10, 260, 21, 21));
        QIcon icon2;
        icon2.addFile(QStringLiteral(":/images/images/freeButton.png"), QSize(), QIcon::Normal, QIcon::Off);
        btnWay->setIcon(icon2);
        btnWay->setIconSize(QSize(16, 16));
        btnWall = new QPushButton(functionsGroupBox);
        btnWall->setObjectName(QStringLiteral("btnWall"));
        btnWall->setGeometry(QRect(120, 260, 21, 21));
        QIcon icon3;
        icon3.addFile(QStringLiteral(":/images/images/wallButton.png"), QSize(), QIcon::Normal, QIcon::Off);
        btnWall->setIcon(icon3);
        btnWall->setIconSize(QSize(16, 16));
        lblWay = new QLabel(functionsGroupBox);
        lblWay->setObjectName(QStringLiteral("lblWay"));
        lblWay->setGeometry(QRect(40, 260, 31, 20));
        lblWall = new QLabel(functionsGroupBox);
        lblWall->setObjectName(QStringLiteral("lblWall"));
        lblWall->setGeometry(QRect(80, 260, 31, 20));
        lblWall->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        btnDirt = new QPushButton(functionsGroupBox);
        btnDirt->setObjectName(QStringLiteral("btnDirt"));
        btnDirt->setGeometry(QRect(10, 290, 21, 21));
        QIcon icon4;
        icon4.addFile(QStringLiteral(":/images/images/dirtButton.png"), QSize(), QIcon::Normal, QIcon::Off);
        btnDirt->setIcon(icon4);
        btnDirt->setIconSize(QSize(16, 16));
        lblDirt = new QLabel(functionsGroupBox);
        lblDirt->setObjectName(QStringLiteral("lblDirt"));
        lblDirt->setGeometry(QRect(40, 290, 31, 20));
        MainWindow->setCentralWidget(centralWidget);
        menuBar = new QMenuBar(MainWindow);
        menuBar->setObjectName(QStringLiteral("menuBar"));
        menuBar->setGeometry(QRect(0, 0, 600, 23));
        MainWindow->setMenuBar(menuBar);

        retranslateUi(MainWindow);

        QMetaObject::connectSlotsByName(MainWindow);
    } // setupUi

    void retranslateUi(QMainWindow *MainWindow)
    {
        MainWindow->setWindowTitle(QApplication::translate("MainWindow", "A* Pathfinding", nullptr));
        functionsGroupBox->setTitle(QApplication::translate("MainWindow", "Functions", nullptr));
        spinMapWidth->setPrefix(QString());
        lblSize->setText(QApplication::translate("MainWindow", "Size:", nullptr));
        btnBuildMap->setText(QApplication::translate("MainWindow", "Build Map", nullptr));
        btnShowPath->setText(QApplication::translate("MainWindow", "Show Path", nullptr));
        btnShowStep->setText(QApplication::translate("MainWindow", "Show Step", nullptr));
        btnShowSteps->setText(QApplication::translate("MainWindow", "Show Steps", nullptr));
        radioBtnEuclidean->setText(QApplication::translate("MainWindow", "Euclidean", nullptr));
        radioBtnManhattan->setText(QApplication::translate("MainWindow", "Manhattan", nullptr));
        lblScale->setText(QApplication::translate("MainWindow", "Scale:", nullptr));
        btnStart->setText(QString());
        lblStart->setText(QApplication::translate("MainWindow", ":Start", nullptr));
        btnEnd->setText(QString());
        lblEnd->setText(QApplication::translate("MainWindow", "End:", nullptr));
        btnWay->setText(QString());
        btnWall->setText(QString());
        lblWay->setText(QApplication::translate("MainWindow", ":Way", nullptr));
        lblWall->setText(QApplication::translate("MainWindow", "Wall:", nullptr));
        btnDirt->setText(QString());
        lblDirt->setText(QApplication::translate("MainWindow", ":Dirt", nullptr));
    } // retranslateUi

};

namespace Ui {
    class MainWindow: public Ui_MainWindow {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MAINWINDOW_H
