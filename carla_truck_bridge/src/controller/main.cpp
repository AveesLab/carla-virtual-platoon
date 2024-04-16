#include "controller.h"
#include "ros2node.hpp"

#include <QApplication>
#include <QLocale>
#include <QTranslator>

static void siginthandler(int /*param*/)
{
    QApplication::quit();
}

int main(int argc, char *argv[])
{
    QApplication app(argc, argv);

    rclcpp::init(argc, argv);

    auto ros2_node = std::make_shared<Ros2Node>();
    auto gui_node = std::make_shared<Controller>(ros2_node);

    app.processEvents();
    gui_node->show();

    rclcpp::executors::MultiThreadedExecutor exec;
    exec.add_node(ros2_node);

    QTranslator translator;
    const QStringList uiLanguages = QLocale::system().uiLanguages();
    for (const QString &locale : uiLanguages) {
        const QString baseName = "qt_test_" + QLocale(locale).name();
        if (translator.load(":/i18n/" + baseName)) {
            app.installTranslator(&translator);
            break;
        }
    }

    while(rclcpp::ok()){
      exec.spin_some();
      app.processEvents();
    }
    signal(SIGINT, siginthandler);

    rclcpp::shutdown();
    return 0;
}

