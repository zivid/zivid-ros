#include <nodelet/loader.h>
#include <ros/ros.h>

#include <cstdlib>

int main(int argc, char ** argv)
{
  try {
    ros::init(argc, argv, "zivid_camera");
  } catch (const std::exception & e) {
    std::cerr << "Failed to initialize ROS: " << e.what() << std::endl;
    return EXIT_FAILURE;
  } catch (...) {
    std::cerr << "Failed to initialize ROS (unknown exception)" << std::endl;
    return EXIT_FAILURE;
  }

  try {
    ROS_INFO("Creating a nodelet::Loader");
    nodelet::Loader nodelet;
    nodelet::M_string remap(ros::names::getRemappings());
    nodelet::V_string nargv;

    const auto nodelet_name = "zivid_camera/nodelet";
    ROS_INFO("Loading nodelet '%s'", nodelet_name);

    const bool loaded = nodelet.load(ros::this_node::getName(), nodelet_name, remap, nargv);
    if (!loaded) {
      ROS_FATAL("Failed to load nodelet '%s'!", nodelet_name);
      return EXIT_FAILURE;
    }

    ROS_INFO("Successfully loaded nodelet '%s'", nodelet_name);
    ros::spin();
    return EXIT_SUCCESS;
  } catch (const std::exception & e) {
    std::cerr << "Exception occurred: " << e.what() << std::endl;
    return EXIT_FAILURE;
  } catch (...) {
    std::cerr << "Unknown exception occurred" << std::endl;
    return EXIT_FAILURE;
  }
}
