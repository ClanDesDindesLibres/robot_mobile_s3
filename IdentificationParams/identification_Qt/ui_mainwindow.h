/********************************************************************************
** Form generated from reading UI file 'mainwindow.ui'
**
** Created by: Qt User Interface Compiler version 5.11.3
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_MAINWINDOW_H
#define UI_MAINWINDOW_H

#include <QtCharts>
#include <QtCore/QVariant>
#include <QtWidgets/QApplication>
#include <QtWidgets/QCheckBox>
#include <QtWidgets/QComboBox>
#include <QtWidgets/QDoubleSpinBox>
#include <QtWidgets/QFrame>
#include <QtWidgets/QGridLayout>
#include <QtWidgets/QLabel>
#include <QtWidgets/QLineEdit>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QSpinBox>
#include <QtWidgets/QTabWidget>
#include <QtWidgets/QTextBrowser>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_MainWindow
{
public:
    QWidget *centralWidget;
    QGridLayout *gridLayout;
    QLineEdit *JsonKey;
    QLabel *label_11;
    QLabel *label_10;
    QLineEdit *lineEdit_Ki;
    QLabel *label_7;
    QTabWidget *animTab;
    QWidget *tab;
    QChartView *graph;
    QWidget *tab_2;
    QWidget *Json;
    QTextBrowser *textBrowser;
    QLineEdit *lineEdit_DesVal;
    QLabel *label;
    QFrame *line;
    QLineEdit *lineEdit_Kp;

    QLabel *label_12;
    QLabel *label_13;
    QLabel *label_14;
    QLineEdit *lineEdit_PosSapin;
    QLineEdit *lineEdit_HauSapin;
    QLineEdit *lineEdit_Start;

    QLineEdit *lineEdit_Kd;
    QPushButton *pulseButton;
    QLabel *label_2;
    QLabel *label_3;
    QCheckBox *checkBox;
    QSpinBox *DurationBox;
    QComboBox *comboBoxPort;
    QDoubleSpinBox *PWMBox;
    QLabel *label_6;
    QLabel *label_pathCSV;
    QLabel *label_5;
    QLabel *label_8;
    QPushButton *pushButton_Params;
    QLabel *label_9;
    QLineEdit *lineEdit_Thresh;

    void setupUi(QMainWindow *MainWindow)
    {
        if (MainWindow->objectName().isEmpty())
            MainWindow->setObjectName(QStringLiteral("MainWindow"));
        MainWindow->resize(752, 578);
        MainWindow->setAcceptDrops(false);
        centralWidget = new QWidget(MainWindow);
        centralWidget->setObjectName(QStringLiteral("centralWidget"));
        gridLayout = new QGridLayout(centralWidget);
        gridLayout->setSpacing(6);
        gridLayout->setContentsMargins(11, 11, 11, 11);
        gridLayout->setObjectName(QStringLiteral("gridLayout"));
        JsonKey = new QLineEdit(centralWidget);
        JsonKey->setObjectName(QStringLiteral("JsonKey"));
        JsonKey->setAlignment(Qt::AlignCenter);

        gridLayout->addWidget(JsonKey, 10, 1, 1, 1);

        label_11 = new QLabel(centralWidget);
        label_11->setObjectName(QStringLiteral("label_11"));

        gridLayout->addWidget(label_11, 10, 4, 1, 1);

        label_10 = new QLabel(centralWidget);
        label_10->setObjectName(QStringLiteral("label_10"));

        gridLayout->addWidget(label_10, 9, 4, 1, 1);

        lineEdit_Ki = new QLineEdit(centralWidget);
        lineEdit_Ki->setObjectName(QStringLiteral("lineEdit_Ki"));

        gridLayout->addWidget(lineEdit_Ki, 8, 5, 1, 1);

        label_7 = new QLabel(centralWidget);
        label_7->setObjectName(QStringLiteral("label_7"));

        gridLayout->addWidget(label_7, 6, 4, 1, 1);

        animTab = new QTabWidget(centralWidget);
        animTab->setObjectName(QStringLiteral("animTab"));
        animTab->setMinimumSize(QSize(700, 0));
        tab = new QWidget();
        tab->setObjectName(QStringLiteral("tab"));
        graph = new QChartView(tab);
        graph->setObjectName(QStringLiteral("graph"));
        graph->setGeometry(QRect(0, 0, 691, 241));
        animTab->addTab(tab, QString());
        tab_2 = new QWidget();
        tab_2->setObjectName(QStringLiteral("tab_2"));
        animTab->addTab(tab_2, QString());
        Json = new QWidget();
        Json->setObjectName(QStringLiteral("Json"));
        textBrowser = new QTextBrowser(Json);
        textBrowser->setObjectName(QStringLiteral("textBrowser"));
        textBrowser->setGeometry(QRect(0, 0, 680, 231));
        textBrowser->setMinimumSize(QSize(680, 0));
        textBrowser->setMaximumSize(QSize(206, 280));
        QFont font;
        font.setPointSize(9);
        textBrowser->setFont(font);
        animTab->addTab(Json, QString());

        gridLayout->addWidget(animTab, 12, 0, 1, 3);

        lineEdit_DesVal = new QLineEdit(centralWidget);
        lineEdit_DesVal->setObjectName(QStringLiteral("lineEdit_DesVal"));

        gridLayout->addWidget(lineEdit_DesVal, 6, 5, 1, 1);

        label = new QLabel(centralWidget);
        label->setObjectName(QStringLiteral("label"));

        gridLayout->addWidget(label, 7, 0, 1, 1);

        line = new QFrame(centralWidget);
        line->setObjectName(QStringLiteral("line"));
        line->setFrameShape(QFrame::HLine);
        line->setFrameShadow(QFrame::Sunken);

        gridLayout->addWidget(line, 13, 2, 1, 1);

        lineEdit_Kp = new QLineEdit(centralWidget);
        lineEdit_Kp->setObjectName(QStringLiteral("lineEdit_Kp"));

        gridLayout->addWidget(lineEdit_Kp, 7, 5, 1, 1);





        label_12 = new QLabel(centralWidget);
        label_12->setObjectName(QStringLiteral("label_12"));

        gridLayout->addWidget(label_12, 11, 4, 1, 1);

        label_13 = new QLabel(centralWidget);
        label_13->setObjectName(QStringLiteral("label_13"));

        gridLayout->addWidget(label_13, 12, 4, 1, 1);

        label_14 = new QLabel(centralWidget);
        label_14->setObjectName(QStringLiteral("label_14"));

        gridLayout->addWidget(label_14, 13, 4, 1, 1);

        lineEdit_PosSapin = new QLineEdit(centralWidget);
        lineEdit_PosSapin->setObjectName(QStringLiteral("lineEdit_PosSapin"));

        gridLayout->addWidget(lineEdit_PosSapin, 12, 5, 1, 1);

        lineEdit_HauSapin = new QLineEdit(centralWidget);
        lineEdit_HauSapin->setObjectName(QStringLiteral("lineEdit_HauSapin"));

        gridLayout->addWidget(lineEdit_HauSapin, 13, 5, 1, 1);

        lineEdit_Start = new QLineEdit(centralWidget);
        lineEdit_Start->setObjectName(QStringLiteral("lineEdit_Start"));

        gridLayout->addWidget(lineEdit_Start, 11, 5, 1, 1);




        lineEdit_Kd = new QLineEdit(centralWidget);
        lineEdit_Kd->setObjectName(QStringLiteral("lineEdit_Kd"));

        gridLayout->addWidget(lineEdit_Kd, 9, 5, 1, 1);

        pulseButton = new QPushButton(centralWidget);
        pulseButton->setObjectName(QStringLiteral("pulseButton"));

        gridLayout->addWidget(pulseButton, 9, 0, 1, 2);

        label_2 = new QLabel(centralWidget);
        label_2->setObjectName(QStringLiteral("label_2"));

        gridLayout->addWidget(label_2, 8, 0, 1, 1);

        label_3 = new QLabel(centralWidget);
        label_3->setObjectName(QStringLiteral("label_3"));

        gridLayout->addWidget(label_3, 10, 0, 1, 1);

        checkBox = new QCheckBox(centralWidget);
        checkBox->setObjectName(QStringLiteral("checkBox"));

        gridLayout->addWidget(checkBox, 5, 0, 1, 2);

        DurationBox = new QSpinBox(centralWidget);
        DurationBox->setObjectName(QStringLiteral("DurationBox"));
        DurationBox->setKeyboardTracking(false);
        DurationBox->setMinimum(0);
        DurationBox->setMaximum(5000);
        DurationBox->setSingleStep(25);

        gridLayout->addWidget(DurationBox, 8, 1, 1, 1);

        comboBoxPort = new QComboBox(centralWidget);
        comboBoxPort->setObjectName(QStringLiteral("comboBoxPort"));

        gridLayout->addWidget(comboBoxPort, 4, 1, 1, 1);

        PWMBox = new QDoubleSpinBox(centralWidget);
        PWMBox->setObjectName(QStringLiteral("PWMBox"));
        PWMBox->setMinimum(-1);
        PWMBox->setMaximum(1);
        PWMBox->setSingleStep(0.1);

        gridLayout->addWidget(PWMBox, 7, 1, 1, 1);

        label_6 = new QLabel(centralWidget);
        label_6->setObjectName(QStringLiteral("label_6"));
        label_6->setAlignment(Qt::AlignCenter);

        gridLayout->addWidget(label_6, 4, 4, 1, 2);

        label_pathCSV = new QLabel(centralWidget);
        label_pathCSV->setObjectName(QStringLiteral("label_pathCSV"));
        label_pathCSV->setFrameShape(QFrame::StyledPanel);
        label_pathCSV->setTextFormat(Qt::AutoText);

        gridLayout->addWidget(label_pathCSV, 6, 0, 1, 2);

        label_5 = new QLabel(centralWidget);
        label_5->setObjectName(QStringLiteral("label_5"));

        gridLayout->addWidget(label_5, 4, 0, 1, 1);

        label_8 = new QLabel(centralWidget);
        label_8->setObjectName(QStringLiteral("label_8"));

        gridLayout->addWidget(label_8, 7, 4, 1, 1);

        pushButton_Params = new QPushButton(centralWidget);
        pushButton_Params->setObjectName(QStringLiteral("pushButton_Params"));

        gridLayout->addWidget(pushButton_Params, 5, 4, 1, 1);

        label_9 = new QLabel(centralWidget);
        label_9->setObjectName(QStringLiteral("label_9"));

        gridLayout->addWidget(label_9, 8, 4, 1, 1);

        lineEdit_Thresh = new QLineEdit(centralWidget);
        lineEdit_Thresh->setObjectName(QStringLiteral("lineEdit_Thresh"));
        lineEdit_Thresh->setMinimumSize(QSize(225, 0));

        gridLayout->addWidget(lineEdit_Thresh, 10, 5, 1, 1);

        MainWindow->setCentralWidget(centralWidget);

        retranslateUi(MainWindow);

        animTab->setCurrentIndex(1);


        QMetaObject::connectSlotsByName(MainWindow);
    } // setupUi

    void retranslateUi(QMainWindow *MainWindow)
    {
        MainWindow->setWindowTitle(QApplication::translate("MainWindow", "Interface Identification", nullptr));
        JsonKey->setText(QApplication::translate("MainWindow", "potVex", nullptr));
        label_11->setText(QApplication::translate("MainWindow", "Seuil", nullptr));
        label_12->setText(QApplication::translate("MainWindow", "Position du sapin", nullptr));
        label_13->setText(QApplication::translate("MainWindow", "Hauteur du sapin", nullptr));
        label_14->setText(QApplication::translate("MainWindow", "start/stop", nullptr));
        label_10->setText(QApplication::translate("MainWindow", "Valeur Kd", nullptr));
        label_7->setText(QApplication::translate("MainWindow", "Val. desiree", nullptr));
        animTab->setTabText(animTab->indexOf(tab), QApplication::translate("MainWindow", "Graphiques Json", nullptr));
        animTab->setTabText(animTab->indexOf(tab_2), QApplication::translate("MainWindow", "Animation Robot", nullptr));
#ifndef QT_NO_WHATSTHIS
        Json->setWhatsThis(QApplication::translate("MainWindow", "<html><head/><body><p>json</p></body></html>", nullptr));
#endif // QT_NO_WHATSTHIS
        animTab->setTabText(animTab->indexOf(Json), QApplication::translate("MainWindow", "Json: Messages de Arduino", nullptr));
        label->setText(QApplication::translate("MainWindow", "Tension [-1,1]", nullptr));
        pulseButton->setText(QApplication::translate("MainWindow", "Commande de pulse", nullptr));
        label_2->setText(QApplication::translate("MainWindow", "Duree (ms)", nullptr));
        label_3->setText(QApplication::translate("MainWindow", "Donnees brutes:", nullptr));
        checkBox->setText(QApplication::translate("MainWindow", "Enregistrement des donnees sous:", nullptr));
        label_6->setText(QApplication::translate("MainWindow", "Execice PID", nullptr));
        label_pathCSV->setText(QString());
        label_5->setText(QApplication::translate("MainWindow", "Port:", nullptr));
        label_8->setText(QApplication::translate("MainWindow", "Valeur Kp", nullptr));
        pushButton_Params->setText(QApplication::translate("MainWindow", "Envoie Parametres", nullptr));
        label_9->setText(QApplication::translate("MainWindow", "Valeur Ki", nullptr));
    } // retranslateUi

};

namespace Ui {
    class MainWindow: public Ui_MainWindow {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MAINWINDOW_H
