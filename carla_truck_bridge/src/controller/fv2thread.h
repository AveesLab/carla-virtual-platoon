#ifndef FV2THREAD_H
#define FV2THREAD_H

#include <QThread>

#include "ros2_msg/msg/xav2cmd.hpp"

typedef ros2_msg::msg::Xav2cmd CmdData;

class Controller;

class FV2Thread : public QThread
{
    Q_OBJECT
public:
    explicit FV2Thread(QObject* parent = nullptr);

private:
    void run();
signals:
    void setValue(CmdData cmd_data);
    void request(CmdData cmd_data);
};

#endif // FV2THEAD_H
