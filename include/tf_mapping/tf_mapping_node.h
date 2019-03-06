#ifndef TF_MAPPING_NODE_H__
#define TF_MAPPING_NODE_H__

#include <ros/ros.h>

// other
// #include <actionlib/server/simple_action_server.h>
// #include <tf/transform_listener.h>

// Tf
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

// messages
#include <tf2_msgs/TFMessage.h>
#include <geometry_msgs/Pose.h>
#include <aruco/aruco.h> // used for temp_markers (not used at the moment)
#include <visualization_msgs/Marker.h> // for rviz 

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

  
  /** \brief Struct to keep marker information */
  struct MarkerInfo
  {

    bool visible;                                   // Marker visibile in actual image?
    int marker_id;                                  // Marker ID
    int previous_marker_id;                         // Used for chaining markers
    geometry_msgs::Pose geometry_msg_to_previous;   // Position with respect to previous marker
    geometry_msgs::Pose geometry_msg_to_world;      // Position with respect to world's origin
    tf::StampedTransform tf_to_previous;            // TF with respect to previous marker
    tf::StampedTransform tf_to_world;               // TF with respect to world's origin
    geometry_msgs::Pose current_camera_pose;        // Position of camera with respect to the marker
    tf::Transform current_camera_tf;                // TF of camera with respect to the marker
  };


protected:
  // ROS API callbacks
  void input_tfCb(const tf2_msgs::TFMessageConstPtr& msg);
  
  /** \brief Function to publish all known markers for visualization purposes*/
  void publishMarker(geometry_msgs::Pose markerPose, int MarkerID, int rank);

  /** \brief Function to publish all known TFs*/
  void publishTfs(bool world_option);
  

  // void imageCb(const sensor_msgs::ImageConstPtr& image);
  // void updateParamsCb(const std_msgs::Float64& msg);

  // helper
  //void transformPoint(const tf::StampedTransform& transform, const geometry_msgs::Point& point_in, geometry_msgs::Point& point_out) const;


  // class members
  std::string space_type_;  // plane or 3D
  int first_tf_id_;
  int tf_counter_;
  int tf_counter_previous_;
  bool first_tf_detected_;
  int num_of_markers_;
  float marker_size_;
  int closest_camera_index_;
  int lowest_number_in_list_;
  tf2_msgs::TFMessageConstPtr tf_msg_list_;

  tf::TransformListener *listener_;
  tf::TransformBroadcaster broadcaster_;

  /** \brief Actual TF of camera with respect to world's origin */
  tf::StampedTransform world_position_transform_;

  /** \brief Container holding MarkerInfo data about all detected markers */
  std::vector<MarkerInfo> markers_;

  /** \brief Actual Pose of camera with respect to world's origin */
  geometry_msgs::Pose world_position_geometry_msg_;

  //tf::TransformListener tf_listener_;
  //Detector::Ptr detector_;

  //sensor_msgs::CameraInfo camera_info_;

  // parameters (set in yaml file)
  //std::string world_frame_;
 

  // subscriber
  // image_transport::Subscriber img_sub_;
  // ros::Subscriber             init_pos_sub_;  // remove position error!
  ros::Subscriber              input_tf_sub_; 

  // publisher
  /** \brief Publisher of visualization_msgs::Marker message to "aruco_markers" topic*/
  ros::Publisher marker_visualization_pub_;
  
  // image_transport::Publisher binary_img_pub_;
  // ros::Publisher ball_states_pub_;


  // action server
  //boost::shared_ptr<GetBallStatesActionServer> get_ball_states_as_;

  // consts:
  // constexpr required since cpp11
  static constexpr double WAIT_FOR_TRANSFORM_INTERVAL = 2.0;
  static constexpr double BROADCAST_WAIT_INTERVAL = 0.0001;

  static const int THIS_IS_FIRST_MARKER = -2;



   static constexpr double RVIZ_MARKER_HEIGHT = 0.01;
   static constexpr double RVIZ_MARKER_LIFETIME = 0.2;
   static constexpr double RVIZ_MARKER_COLOR_R = 1.0;
   static constexpr double RVIZ_MARKER_COLOR_G = 1.0;
   static constexpr double RVIZ_MARKER_COLOR_B = 1.0;
   static constexpr double RVIZ_MARKER_COLOR_A = 1.0;
   static constexpr double INIT_MIN_SIZE_VALUE = 1000000;
}; // end of class
}

#endif
