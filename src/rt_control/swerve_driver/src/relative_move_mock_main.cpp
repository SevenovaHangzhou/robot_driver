#include "swerve_driver/relative_move_mock_node.hpp"
#include <cstdio>
#include <exception>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  int result = 0;
  try {
    auto node = std::make_shared<swerve_driver::RelativeMoveMockNode>();
    rclcpp::spin(node);
  } catch (const std::exception & error) {
    // SIGINT/SIGTERM may invalidate the ROS context while entities are being created.
    // Destruction releases the mock's resources; a requested shutdown is not bad config.
    if (rclcpp::ok()) {
      std::fprintf(stderr, "Tier A no-device mock refused: %s\n", error.what());
      result = 1;
    }
  }
  rclcpp::shutdown();
  return result;
}
