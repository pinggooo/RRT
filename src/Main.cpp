#include "MainWindow.hpp"
#include <QApplication>

int main(int argc, char** argv) {
    QApplication app(argc, argv);
    MainWindow window;
    window.setWindowTitle("RRT Simulator");
    window.show();

    return app.exec();
}
