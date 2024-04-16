#ifndef LVTHREAD_H
#define LVTHREAD_H

#include <QThread>

#include "ros2_msg/msg/xav2cmd.hpp"

typedef ros2_msg::msg::Xav2cmd CmdData;

class Controller;

class LVThread : public QThread
{
    Q_OBJECT
public:
    explicit LVThread(QObject* parent = nullptr);

private:
    void run();
signals:
    void setValue(CmdData cmd_data);
    void request(CmdData cmd_data);
};

#endif // LVTHREAD_H
