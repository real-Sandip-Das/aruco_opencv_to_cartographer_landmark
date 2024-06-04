#include <ros/ros.h>
#include "aruco_opencv_to_cartographer_landmark/ArucoDetectionToLandmarkTranslator.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "aruco_opencv_to_cartographer_landmark");
  ros::NodeHandle nodeHandle("~");

  aruco_opencv_to_cartographer_landmark::ArucoDetectionToLandmarkTranslator arucoDetectionToLandmarkTranslator(nodeHandle);

  ros::spin();
  return 0;
}