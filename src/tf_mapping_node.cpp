#include <tf_mapping/tf_mapping_node.h>
#include <tf_mapping/helper.h>

namespace tf_mapping
{

// constructor:  
TfMappingNode::TfMappingNode(ros::NodeHandle& nh):
  first_tf_detected_(false),
  tf_counter_(0),
  first_tf_id_(-1)
{
 
  // Node Handle stuff:


  // // initialize detector
  // detector_.reset(new Detector(camera_info_, ball_diameter_, nh.param("tf_mapping/max_age", 10.0)
  //                              , nh.param("tf_mapping/min_radius", 10.0)
  //                              , nh.param("tf_mapping/fill_ratio", 0.6)
  //                              , use_simulation_
  //                              , nh.param("tf_mapping/kf_states", 9)
  //                              ));

  // subscriber
  // image_transport::ImageTransport it(nh);
  // img_sub_     = it.subscribe(input_img_topic_, 1, &BallTrackingNode::imageCb, this);
  // init_pos_sub_= nh.subscribe("tf_mapping/initialize_position", 1, &BallTrackingNode::initializeCb, this);
  input_tf_sub_ = nh.subscribe("/tf", 1, &TfMappingNode::input_tfCb, this);

  // publisher
  //binary_img_pub_ = it.advertise("tf_mapping/binary_img", 1);
  //first_ball_state_point_stamped_pub_ = nh.advertise<geometry_msgs::PointStamped>("tf_mapping/first_ball_state_point_stamped", 1);

  // init action servers
  //get_ball_states_as_.reset(new GetBallStatesActionServer(nh, "tf_mapping/get_ball_states", false));
  //get_ball_states_as_->registerGoalCallback(boost::bind(&BallTrackingNode::ballRequestActionGoalCb, this));
  //get_ball_states_as_->start();

  ROS_INFO("tf_mapping initialization finished");
}

TfMappingNode::~TfMappingNode()
{
}

void TfMappingNode::update()
{

}



// should listen to:
// transforms:
// -
//   header:
//     seq: 0
//     stamp:
//       secs: 1551807735
//       nsecs: 439033972
//     frame_id: "usb_cam"
//   child_frame_id: "marker_id1"
//   transform:
//     translation:
//       x: 0.373289123077
//       y: -0.154011595081
//       z: 0.918938849558
//     rotation:
//       x: 0.970674993953
//       y: -0.042641629777
//       z: -0.0703542358842
//       w: -0.22588056361
void TfMappingNode::input_tfCb(const tf2_msgs::TFMessageConstPtr& msg)
{
  ROS_INFO_ONCE("Received first tf");
  tf_msg_ = msg;
  //------------------------------------------------------
  // FIRST tf DETECTED (Line 210)
  //------------------------------------------------------
  first_tf_detected_ = true;
  first_tf_id_       = getNumOfString(tf_msg_->transforms[0].child_frame_id);

  ROS_INFO_STREAM("The first tf id is" << first_tf_id_ );
}


// callbacks:
// void BallTrackingNode::imageCb(const sensor_msgs::ImageConstPtr& image)
// {
//   current_image_ = image;
//   ROS_INFO_ONCE("tf_mapping received an image");
// }


//action server callback
// void TfMappingNode::ballRequestActionGoalCb()
// {

// }
} //end of namespace






int main(int argc, char** argv)
{
  ros::init(argc, argv, "tf_mapping");

  ros::NodeHandle nh;

  tf_mapping::TfMappingNode node(nh);
  ros::Rate loop_rate(nh.param("tf_mapping/update_rate", 1.0));

  while (ros::ok())
  {
    ros::spinOnce();
    node.update();

    if (!loop_rate.sleep())
      ROS_WARN_THROTTLE(10.0, "[tf_mapping] Update rate was not met!");
  }

  return 0;
}
