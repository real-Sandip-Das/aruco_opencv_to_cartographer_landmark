#pragma once

// ROS
#include <ros/ros.h>
#include <aruco_opencv_msgs/ArucoDetection.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2_ros/transform_listener.h>

namespace aruco_opencv_to_cartographer_landmark {

/*!
 * Main class for the node to handle the ROS interfacing.
 */
class ArucoDetectionToLandmarkTranslator
{
 public:
  /*!
   * Constructor.
   * @param nodeHandle the ROS node handle.
   */
  ArucoDetectionToLandmarkTranslator(ros::NodeHandle& nodeHandle);

  /*!
   * Destructor.
   */
  virtual ~ArucoDetectionToLandmarkTranslator();

 private:
  /*!
   * Reads and verifies the ROS parameters.
   * @return true if successful.
   */
  bool readParameters();

  /*!
   * ROS topic callback method.
   * @param message the received message.
   */
  void topicCallback(const aruco_opencv_msgs::ArucoDetection& message);

  //! ROS node handle.
  ros::NodeHandle& nodeHandle_;

  //! ROS topic subscriber.
  ros::Subscriber subscriber_;

  //! ROS topic publisher.
  ros::Publisher publisher_;

  //! ROS topic name to subscribe to.
  std::string subscriberTopic_;

  //! ROS topic name to publish to.
  std::string publisherTopic_;

  //! ROS topic subscriber's queue size.
  int subscriberQueueSize_;

  //! ROS topic publisher's queue size.
  int publisherQueueSize_;

  //! Frame being tracked by Google Cartographer SLAM
  std::string trackingFrame_;

  tf2_ros::Buffer tfBuffer_;
  tf2_ros::TransformListener tfListener_;
};

} /* namespace */