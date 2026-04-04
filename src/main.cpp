#include <QApplication>
#include <rclcpp/rclcpp.hpp>
#include "simulator/simulator_node.hpp"
#include "simulator/gui/main_window.hpp"

int main(int argc, char* argv[]) {
    // Init ROS2 before Qt so that RCL parameters are parsed
    rclcpp::init(argc, argv);

    QApplication app(argc, argv);
    app.setApplicationName("ROS2 Simulator");
    app.setStyle("Fusion");

    // Dark palette (optional, looks nice)
    QPalette dark;
    dark.setColor(QPalette::Window,          QColor(53,  53,  53));
    dark.setColor(QPalette::WindowText,      Qt::white);
    dark.setColor(QPalette::Base,            QColor(35,  35,  35));
    dark.setColor(QPalette::AlternateBase,   QColor(53,  53,  53));
    dark.setColor(QPalette::ToolTipBase,     QColor(25,  25,  25));
    dark.setColor(QPalette::ToolTipText,     Qt::white);
    dark.setColor(QPalette::Text,            Qt::white);
    dark.setColor(QPalette::Button,          QColor(53,  53,  53));
    dark.setColor(QPalette::ButtonText,      Qt::white);
    dark.setColor(QPalette::BrightText,      Qt::red);
    dark.setColor(QPalette::Link,            QColor(42, 130, 218));
    dark.setColor(QPalette::Highlight,       QColor(42, 130, 218));
    dark.setColor(QPalette::HighlightedText, QColor(35,  35,  35));
    app.setPalette(dark);

    auto node = std::make_shared<SimulatorNode>();
    node->start();   // start ROS spin thread

    MainWindow window(node);
    window.show();

    int ret = app.exec();
    rclcpp::shutdown();
    return ret;
}
