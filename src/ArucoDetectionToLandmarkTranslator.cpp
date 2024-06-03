#include "aruco_opencv_to_cartographer_landmark/ArucoDetectionToLandmarkTranslator.hpp"

// STD
#include <string>

// ROS
#include <cartographer_ros_msgs/LandmarkList.h>

namespace aruco_opencv_to_cartographer_landmark {

ArucoDetectionToLandmarkTranslator::ArucoDetectionToLandmarkTranslator(ros::NodeHandle& nodeHandle) :
    nodeHandle_(nodeHandle),
    tfListener_(tfBuffer_)
{
  if (!readParameters()) {
    ROS_ERROR("Could not read parameters.");
    ros::requestShutdown();
  }
  subscriber_ = nodeHandle_.subscribe(subscriberTopic_, subscriberQueueSize_,
                                      &ArucoDetectionToLandmarkTranslator::topicCallback, this);
  publisher_ = nodeHandle_.advertise<cartographer_ros_msgs::LandmarkList>(publisherTopic_, publisherQueueSize_);
  ROS_INFO("Successfully launched node.");
}

ArucoDetectionToLandmarkTranslator::~ArucoDetectionToLandmarkTranslator()
{
}

bool ArucoDetectionToLandmarkTranslator::readParameters()
{
  if (!nodeHandle_.getParam("subscriber_topic", subscriberTopic_)) return false;
  if (!nodeHandle_.getParam("publisher_topic", publisherTopic_)) return false;
  if (!nodeHandle_.getParam("subscriber_queue_size", subscriberQueueSize_)) return false;
  if (!nodeHandle_.getParam("publisher_queue_size", publisherQueueSize_)) return false;
  if (!nodeHandle_.getParam("tracking_frame", trackingFrame_)) return false;

  return true;
}

void ArucoDetectionToLandmarkTranslator::topicCallback(const aruco_opencv_msgs::ArucoDetection& message)
{
  cartographer_ros_msgs::LandmarkList reply;
  reply.header = message.header;
  
  for (aruco_opencv_msgs::MarkerPose marker : message.markers)
  {
    cartographer_ros_msgs::LandmarkEntry landmarkEntry;
    landmarkEntry.id = std::to_string(marker.marker_id);
    
    geometry_msgs::PoseStamped poseIn;
    poseIn.header = message.header;
    poseIn.pose = marker.pose;
    try{
      tfBuffer_.transform(poseIn, landmarkEntry.tracking_from_landmark_transform, trackingFrame_, ros::Duration(0.0));
    }
    catch (tf2::TransformException &ex) {
      ROS_WARN("Failure %s\n", ex.what());
    }
    landmarkEntry.rotation_weight = 1.0;
    landmarkEntry.translation_weight = 1.0;

    reply.landmarks.push_back(landmarkEntry);
  }
  publisher_.publish(reply);
}

} /* namespace */
