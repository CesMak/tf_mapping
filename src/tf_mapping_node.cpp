#include <tf_mapping/tf_mapping_node.h>

namespace tf_mapping
{

// constructor:  
TfMappingNode::TfMappingNode(ros::NodeHandle& nh)
  //: frame_count_(0)
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
  // update_params_sub_= nh.subscribe("tf_mapping/update_params", 1, &BallTrackingNode::updateParamsCb, this);

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
