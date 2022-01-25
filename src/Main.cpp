#include "rclcpp/rclcpp.hpp"
#include "Application.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::executors::MultiThreadedExecutor executor;
    auto app = std::make_shared<Application>();
    executor.add_node(app);

    while (rclcpp::ok()) {
        if (app->initialize() && !app->isFinished) {
            app->run();
            break;
        }

        executor.spin_once();
    }

    rclcpp::shutdown();

    return 0;
}
