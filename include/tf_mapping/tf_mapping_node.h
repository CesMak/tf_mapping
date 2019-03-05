#ifndef TF_MAPPING_NODE_H__
#define TF_MAPPING_NODE_H__

#include <ros/ros.h>

// other
// #include <actionlib/server/simple_action_server.h>
// #include <tf/transform_listener.h>

// messages
// #include <geometry_msgs/PoseStamped.h>
// #include <geometry_msgs/PoseArray.h>
// #include <sensor_msgs/CameraInfo.h>
// #include <sensor_msgs/image_encodings.h>

namespace tf_mapping
{
class TfMappingNode
{
public:
  //typedef actionlib::SimpleActionServer<ball_tracking_msgs::GetBallStatesAction> GetBallStatesActionServer;

  TfMappingNode(ros::NodeHandle& nh);
  virtual ~TfMappingNode();

  void update();


protected:
  // ROS API callbacks
  // void imageCb(const sensor_msgs::ImageConstPtr& image);
  // void updateParamsCb(const std_msgs::Float64& msg);

  // helper
  //void transformPoint(const tf::StampedTransform& transform, const geometry_msgs::Point& point_in, geometry_msgs::Point& point_out) const;


  // class members
  //tf::TransformListener tf_listener_;
  //Detector::Ptr detector_;

  //sensor_msgs::CameraInfo camera_info_;

  // parameters (set in yaml file)
  std::string world_frame_;
 

  // subscriber
  // image_transport::Subscriber img_sub_;
  // ros::Subscriber             init_pos_sub_;  // remove position error!
  // ros::Subscriber              update_params_sub_; 

  // publisher
  // image_transport::Publisher binary_img_pub_;
  // ros::Publisher ball_states_pub_;


  // action server
  //boost::shared_ptr<GetBallStatesActionServer> get_ball_states_as_;
};
}

#endif
