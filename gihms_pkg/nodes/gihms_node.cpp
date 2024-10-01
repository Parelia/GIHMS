#include <QApplication>
#include <ros/ros.h>
#include <main_window.h>

int main(int argc, char **argv) {
    // Inizializza ROS
    ros::init(argc, argv, "GIHMs");

    // Inizializza Qt
    QApplication app(argc, argv);

    MainWindow m;
    m.setWindowTitle("GIHMs");
    m.setWindowIcon(QIcon(":/icons/icons8-robotic-arm-80.png"));
    m.show();

    // Esegui il loop degli eventi di Qt
    return app.exec();
}
