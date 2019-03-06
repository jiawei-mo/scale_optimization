// This file is part of SVO - Semi-direct Visual Odometry.
//
// Copyright (C) 2014 Christian Forster <forster at ifi dot uzh dot ch>
// (Robotics and Perception Group, University of Zurich, Switzerland).
//
// SVO is free software: you can redistribute it and/or modify it under the
// terms of the GNU General Public License as published by the Free Software
// Foundation, either version 3 of the License, or any later version.
//
// SVO is distributed in the hope that it will be useful, but WITHOUT ANY
// WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
// FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.
#include <ros/package.h>
#include <string>
#include <frame_handler_sosvo.h>
#include <svo/map.h>
#include <svo/config.h>
#include <visualizer_sosvo.h>
#include <vikit/params_helper.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/String.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <image_transport/image_transport.h>
#include <boost/thread.hpp>
#include <cv_bridge/cv_bridge.h>
#include <Eigen/Core>
#include <vikit/pinhole_camera.h>
#include <vikit/camera_loader.h>
#include <vikit/user_input_thread.h>
#include <opencv2/core/eigen.hpp>

namespace svo {

/// SVO Interface
class SOSVONode
{
public:
  svo::FrameHandlerSOSVO* vo_;
  svo::VisualizerSOSVO visualizer_;
  bool publish_markers_;                 //!< publish only the minimal amount of info (choice for embedded devices)
  bool publish_dense_input_;
  boost::shared_ptr<vk::UserInputThread> user_input_thread_;
  ros::Subscriber sub_remote_key_;
  std::string remote_input_;
  vk::PinholeCamera* cam_;
  vk::PinholeCamera* stereo_cam_;
  bool quit_;
  SOSVONode();
  ~SOSVONode();
  void imageMessageCallback(const sensor_msgs::ImageConstPtr& msg0, const sensor_msgs::ImageConstPtr& msg1);
  void processUserActions();
  void remoteKeyCb(const std_msgs::StringConstPtr& key_input);
};

SOSVONode::SOSVONode() :
  vo_(NULL),
  publish_markers_(vk::getParam<bool>("svo/publish_markers", true)),
  publish_dense_input_(vk::getParam<bool>("svo/publish_dense_input", false)),
  remote_input_(""),
  cam_(NULL),
  stereo_cam_(NULL),
  quit_(false)
{
  // Start user input thread in parallel thread that listens to console keys
  if(vk::getParam<bool>("svo/accept_console_user_input", true))
    user_input_thread_ = boost::make_shared<vk::UserInputThread>();

  // Create Cameras
  ros::NodeHandle nhPriv("~");
  std::vector<double> E0, K0, frame_size0, dist_coeff0;
  std::vector<double> E1, K1, frame_size1, dist_coeff1;
  // cameras model
  if(!nhPriv.getParam("/cam0/T_BS/data", E0)
  || !nhPriv.getParam("/cam0/intrinsics", K0)
  || !nhPriv.getParam("/cam0/resolution", frame_size0)
  || !nhPriv.getParam("/cam0/distortion_coefficients", dist_coeff0)
  || !nhPriv.getParam("/cam1/T_BS/data", E1)
  || !nhPriv.getParam("/cam1/intrinsics", K1)
  || !nhPriv.getParam("/cam1/resolution", frame_size1)
  || !nhPriv.getParam("/cam1/distortion_coefficients", dist_coeff1))
  {
    ROS_INFO("Fail to get cameras parameters, exit.");
        return;
  }

  cam_ = new vk::PinholeCamera(
    frame_size0[0], frame_size0[1], K0[0], K0[1], K0[2], K0[3],
    dist_coeff0[0], dist_coeff0[1], dist_coeff0[2], dist_coeff0[3]);
  stereo_cam_ = new vk::PinholeCamera(
    frame_size1[0], frame_size1[1], K1[0], K1[1], K1[2], K1[3],
    dist_coeff1[0], dist_coeff1[1], dist_coeff1[2], dist_coeff1[3]);

  // stereo pose
  cv::Mat cam0_E = cv::Mat(E0);
  cam0_E = cam0_E.reshape(0,4);
  cv::Mat R0(cam0_E, cv::Rect(0,0,3,3));
  cv::Mat t0 = cam0_E(cv::Rect(3,0,1,3));

  cv::Mat cam1_E = cv::Mat(E1);
  cam1_E = cam1_E.reshape(0,4);
  cv::Mat R1 = cam1_E(cv::Rect(0,0,3,3));
  cv::Mat t1 = cam1_E(cv::Rect(3,0,1,3));

  cv::Mat R1T;
  cv::transpose(R1, R1T);
  cv::Mat cvR = R1T * R0;
  cv::Mat cvt = R1T * (t0 - t1);

  Eigen::Matrix3d R;
  Eigen::Vector3d t;
  cv::cv2eigen(cvR, R);
  cv::cv2eigen(cvt, t);

  Sophus::SE3 T_stereo = Sophus::SE3(R,t);

  // Get initial position and orientation
  visualizer_.T_world_from_vision_ = Sophus::SE3(
      vk::rpy2dcm(Vector3d(vk::getParam<double>("svo/init_rx", 0.0),
                           vk::getParam<double>("svo/init_ry", 0.0),
                           vk::getParam<double>("svo/init_rz", 0.0))),
      Eigen::Vector3d(vk::getParam<double>("svo/init_tx", 0.0),
                      vk::getParam<double>("svo/init_ty", 0.0),
                      vk::getParam<double>("svo/init_tz", 0.0)));

  // Init VO and start
  vo_ = new svo::FrameHandlerSOSVO(cam_, stereo_cam_, T_stereo);
  vo_->start();
}

SOSVONode::~SOSVONode()
{
  delete vo_;
  delete cam_;
  if(user_input_thread_ != NULL)
    user_input_thread_->stop();
}

void SOSVONode::imageMessageCallback(const sensor_msgs::ImageConstPtr& msg0, const sensor_msgs::ImageConstPtr& msg1)
{
  cv::Mat img, stereo_img;
  try {
    img = cv_bridge::toCvShare(msg0, "mono8")->image;
    stereo_img = cv_bridge::toCvShare(msg1, "mono8")->image;
  } catch (cv_bridge::Exception& e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
  }
  processUserActions();
  vo_->addImage(img, stereo_img, msg0->header.stamp.toSec());
  visualizer_.publishMinimal(img, vo_->lastFrame(), *vo_, msg0->header.stamp.toSec());

  if(publish_markers_ && vo_->stage() != FrameHandlerBase::STAGE_PAUSED)
    visualizer_.visualizeMarkers(vo_->lastFrame(), vo_->coreKeyframes(), vo_->map());

  if(publish_dense_input_)
    visualizer_.exportToDense(vo_->lastFrame());

  if(vo_->stage() == FrameHandlerSOSVO::STAGE_PAUSED)
    usleep(100000);
}

void SOSVONode::processUserActions()
{
  char input = remote_input_.c_str()[0];
  remote_input_ = "";

  if(user_input_thread_ != NULL)
  {
    char console_input = user_input_thread_->getInput();
    if(console_input != 0)
      input = console_input;
  }

  switch(input)
  {
    case 'q':
      quit_ = true;
      printf("SVO user input: QUIT\n");
      break;
    case 'r':
      vo_->reset();
      printf("SVO user input: RESET\n");
      break;
    case 's':
      vo_->start();
      printf("SVO user input: START\n");
      break;
    default: ;
  }
}

void SOSVONode::remoteKeyCb(const std_msgs::StringConstPtr& key_input)
{
  remote_input_ = key_input->data;
}

} // namespace svo

int main(int argc, char **argv)
{
  ros::init(argc, argv, "svo");
  ros::NodeHandle nh;
  std::cout << "create vo_node" << std::endl;
  svo::SOSVONode vo_node;

  // subscribe to cam msgs
  ros::NodeHandle nhPriv("~");
  std::string cam0_topic, cam1_topic;
  if(!nhPriv.getParam("cam0_topic", cam0_topic)
  || !nhPriv.getParam("cam1_topic", cam1_topic) )
  {
    ROS_INFO("Fail to get sensor topics, exit.");
        return 1;
  }

  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> SOSVOSyncPolicy;
  message_filters::Subscriber<sensor_msgs::Image> *cam0_sub;
  message_filters::Subscriber<sensor_msgs::Image> *cam1_sub;
  message_filters::Synchronizer<SOSVOSyncPolicy> *sync;
  cam0_sub = new message_filters::Subscriber<sensor_msgs::Image>(nh, cam0_topic, 1000);
  cam1_sub = new message_filters::Subscriber<sensor_msgs::Image>(nh, cam1_topic, 1000);
  sync = new message_filters::Synchronizer<SOSVOSyncPolicy>(SOSVOSyncPolicy(10), *cam0_sub, *cam1_sub);
  sync->registerCallback(boost::bind(&svo::SOSVONode::imageMessageCallback, &vo_node, _1, _2));

  // subscribe to remote input
  vo_node.sub_remote_key_ = nh.subscribe("svo/remote_key", 5, &svo::SOSVONode::remoteKeyCb, &vo_node);

  // start processing callbacks
  while(ros::ok() && !vo_node.quit_)
  {
    ros::spinOnce();
    // TODO check when last image was processed. when too long ago. publish warning that no msgs are received!
  }

  printf("SVO terminated.\n");
  return 0;
}
