#ifndef DESIGNROBOT_H
#define DESIGNROBOT_H

#include <QDialog>

namespace Ui {
class DesignRobot;
}

class DesignRobot : public QDialog
{
    Q_OBJECT

public:
    explicit DesignRobot(QWidget *parent = nullptr);
    ~DesignRobot();

private:
    Ui::DesignRobot *ui;
};

#endif // DESIGNROBOT_H
