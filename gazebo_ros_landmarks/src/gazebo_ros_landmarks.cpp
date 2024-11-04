// Copyright 2023 Andrea Ostuni - PIC4SeR
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
// THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.


#include "gazebo_ros_landmarks/gazebo_ros_landmarks.hpp"

namespace gazebo_ros_landmarks
{

class GazeboLandmarksPrivate
{
  using LandmarkMeasureArray = landmark_msgs::msg::LandmarkMeasureArray;
  using LandmarkMeasure = gazebo_landmark_msgs::msg::LandmarkMeasure;
  using Landmark = std::pair<std::string,ignition::math::Pose3d>;
public:
  /**
   * @brief Callback to be called at every simulation iteration
   * @param[in] info Updated simulation info
   */
  void OnUpdate(const gazebo::common::UpdateInfo & info);

  /**
   * @brief The robot model being controlled.
  */
  gazebo::physics::ModelPtr robot_model_{nullptr};

  /**
   * @brief The reference link of the robot.
  */
  gazebo::physics::LinkPtr robot_reference_link_{nullptr};

  /**
   * @brief Connection that maintains a link between the contact sensor's
   *  updated signal and the OnUpdate callback.
   */
  gazebo::event::ConnectionPtr update_connection_;

  /**
   * @brief A pointer to the world.
   */
  gazebo::physics::WorldPtr world_{nullptr};

  /**
   * @brief pointer to the GazeboROS node.
   */
  gazebo_ros::Node::SharedPtr ros_node_{nullptr};

  /**
   * @brief Odom topic name
   */
  std::string topic_name_{"landmarks"};

    /**
   * @brief Frame transform name, should match name of reference link, or be world.
   */
  std::string robot_frame_name_{"base_link"};

  /**
    * @brief The landmarks names and poses in the world frame
  */
  std::vector<Landmark> landmarks_;

  /**
   * @brief Keep track of the last update time.
   */
  gazebo::common::Time last_time_;

  /**
   * @brief  Publish rate in Hz.
   */
  double update_rate_{0.0};

  /**
   * @brief Gaussian noise to be added to the range measurements.
   */
  double gaussian_noise_range_{0.0};

  /**
   * @brief Gaussian noise to be added to the bearing measurements.
   */
  double gaussian_noise_bearing_{0.0};

  /**
    * @brief  min range of the sensor (defaults to 0m)
  */
  double min_range_{0.0};
  
  /**
   * @brief  max range of the sensor (defaults to 10m)
  */
  double max_range_{10.0};

  /**
   * @brief  Publish landmark poses as ranges and bearings.
   */
  std::shared_ptr<rclcpp::Publisher<LandmarkMeasureArray>> pub_;
};

double wrapAngle(double angle)
{
  while (angle > M_PI) {
    angle -= 2 * M_PI;
  }
  while (angle < -M_PI) {
    angle += 2 * M_PI;
  }
  return angle;
}

double distance2D(ignition::math::Vector3d v1, ignition::math::Vector3d v2)
{
  return sqrt(pow(v1.X() - v2.X(), 2) + pow(v1.Y() - v2.Y(), 2));
}

GazeboLandmarks::GazeboLandmarks()
: impl_(std::make_unique<GazeboLandmarksPrivate>())
{
}

GazeboLandmarks::~GazeboLandmarks()
{
  impl_->ros_node_.reset();
}

void GazeboLandmarks::Load(gazebo::physics::WorldPtr _world, sdf::ElementPtr sdf)
{
  // Get world pointer
  impl_->world_ = _world;

  // Configure the plugin from the SDF file
  impl_->ros_node_ = gazebo_ros::Node::Get(sdf);

  // Get QoS profiles
  const gazebo_ros::QoS & qos = impl_->ros_node_->get_qos();

  if (!sdf->HasElement("update_rate")) {
    RCLCPP_DEBUG(
      impl_->ros_node_->get_logger(), "landmarks plugin missing <update_rate>, defaults to 0.0"
      " (as fast as possible)");
  } else {
    impl_->update_rate_ = sdf->GetElement("update_rate")->Get<double>();
  }

  std::string robot_name;
  if (!sdf->HasElement("robot_name")) {
    RCLCPP_ERROR(impl_->ros_node_->get_logger(), "Missing <robot_name>, cannot proceed");
    return;
  } else {
    robot_name = sdf->GetElement("robot_name")->Get<std::string>();
  }

  impl_->robot_model_ = impl_->world_->ModelByName(robot_name);
  if (!impl_->robot_model_) {
    RCLCPP_ERROR(
      impl_->ros_node_->get_logger(), "robot_name: %s does not exist\n",
      robot_name.c_str());
    return;
  }

  if(!sdf->HasElement("min_range")) {
    RCLCPP_DEBUG(impl_->ros_node_->get_logger(), "Missing <min_range>, defaults to 0.0");
  } else {
    impl_->min_range_ = sdf->GetElement("min_range")->Get<double>();
  }

  if(!sdf->HasElement("max_range")) {
    RCLCPP_DEBUG(impl_->ros_node_->get_logger(), "Missing <max_range>, defaults to 10.0");
  } else {
    impl_->max_range_ = sdf->GetElement("max_range")->Get<double>();
  }

  if(!sdf->HasElement("robot_frame_name")) {
    RCLCPP_WARN(impl_->ros_node_->get_logger(), "Missing <robot_frame_name>, defaults to base_link");
    return;
  } else {
    impl_->robot_frame_name_ = sdf->GetElement("robot_frame_name")->Get<std::string>();
  }
  impl_->robot_reference_link_ = impl_->robot_model_->GetLink(impl_->robot_frame_name_);
  if (!impl_->robot_reference_link_) {
    RCLCPP_ERROR(
      impl_->ros_node_->get_logger(), "robot_frame_name: %s does not exist\n",
      impl_->robot_frame_name_.c_str());
    return;
  }

  std::vector<std::string> landmark_names;
  if (!sdf->HasElement("landmark_names")) {
    RCLCPP_ERROR(impl_->ros_node_->get_logger(), "Missing <landmark_names>, cannot proceed");
    return;
  } else {

    auto landmark = sdf->GetElement("landmark_names")->GetElement("landmark");
    while(landmark)
    {
      landmark_names.push_back(landmark->Get<std::string>());
      landmark = landmark->GetNextElement("landmark");
    }
  }

  // Load the landmark poses
  for (auto landmark_name : landmark_names)
  {
    auto landmark_model = impl_->world_->ModelByName(landmark_name);
    if (!landmark_model) {
      RCLCPP_ERROR(
        impl_->ros_node_->get_logger(), "landmark_name: %s does not exist\n",
        landmark_name.c_str());
      return;
    }
    impl_->landmarks_.push_back(std::make_pair(landmark_name, landmark_model->WorldPose()));
  }

  impl_->pub_ = impl_->ros_node_->create_publisher<gazebo_landmark_msgs::msg::LandmarkMeasureArray>(
    impl_->topic_name_, qos.get_publisher_qos(
      impl_->topic_name_, rclcpp::SensorDataQoS().reliable()));
  impl_->topic_name_ = impl_->pub_->get_topic_name();
  RCLCPP_DEBUG(
    impl_->ros_node_->get_logger(), "Publishing on topic [%s]", impl_->topic_name_.c_str());


  if(!sdf->HasElement("gaussian_noise_range")) {
    RCLCPP_DEBUG(impl_->ros_node_->get_logger(), "Missing <gaussian_noise_range>, defaults to 0.0");
    impl_->gaussian_noise_range_ = 0.0;
  } else {
    impl_->gaussian_noise_range_ = sdf->GetElement("gaussian_noise_range")->Get<double>();
  }

  if(!sdf->HasElement("gaussian_noise_bearing")) {
    RCLCPP_DEBUG(impl_->ros_node_->get_logger(), "Missing <gaussian_noise_bearing>, defaults to 0.0");
    impl_->gaussian_noise_bearing_ = 0.0;
  } else {
    impl_->gaussian_noise_bearing_ = sdf->GetElement("gaussian_noise_bearing")->Get<double>();
  }

  impl_->last_time_ = impl_->world_->SimTime();

  // Listen to the update event. This event is broadcast every simulation iteration
  impl_->update_connection_ = gazebo::event::Events::ConnectWorldUpdateBegin(
    std::bind(&GazeboLandmarksPrivate::OnUpdate, impl_.get(), std::placeholders::_1));
}

// Update the controller
void GazeboLandmarksPrivate::OnUpdate(const gazebo::common::UpdateInfo & info)
{
  if (!robot_model_) {
    return;
  }

  gazebo::common::Time current_time = info.simTime;

  if (current_time < last_time_) {
    RCLCPP_WARN(ros_node_->get_logger(), "Negative update time difference detected.");
    last_time_ = current_time;
  }

  // Rate control
  if (update_rate_ > 0 &&
    (current_time - last_time_).Double() < (1.0 / update_rate_))
  {
    return;
  }

  // If we don't have any subscribers, don't bother composing and sending the message
  if (ros_node_->count_subscribers(topic_name_) == 0) {
    return;
  }

  // Create message
  LandmarkMeasureArray measure_array_msg;
  LandmarkMeasure measure_msg;
  

  // Fill in header
  measure_array_msg.header.stamp = gazebo_ros::Convert<builtin_interfaces::msg::Time>(current_time);
  measure_array_msg.header.frame_id = robot_frame_name_;

  // Get pose/orientation
  auto robot_pose = robot_reference_link_->WorldPose();

  // Get landmark distances and bearings
  for (const auto & [landmark_name, landmark_pose] : landmarks_)
  {
    // Get the landmark pose in the robot frame
    auto landmark_pose_robot_frame = robot_pose.Inverse() * landmark_pose;

    // Get the landmark distance and bearing
    measure_msg.id = landmark_name;
    measure_msg.range = distance2D(landmark_pose_robot_frame.Pos(), ignition::math::Vector3d(0,0,0));
    if(measure_msg.range < min_range_ || measure_msg.range > max_range_)
    {
      continue;
    }
    measure_msg.bearing = atan2(landmark_pose_robot_frame.Pos().Y(), landmark_pose_robot_frame.Pos().X());

    // Add noise
    measure_msg.range += ignition::math::Rand::DblNormal(0, gaussian_noise_range_);
    measure_msg.bearing += ignition::math::Rand::DblNormal(0, gaussian_noise_bearing_);
    measure_msg.bearing = wrapAngle(measure_msg.bearing);
    // Add to the array
    measure_array_msg.landmarks.push_back(measure_msg);

    measure_msg.hamming = 1;
    measure_msg.goodness = 1.0;
    measure_msg.decision_margin = 1.0;
    
  }

  // Publish to ROS
  pub_->publish(measure_array_msg);
  // Save last time stamp
  last_time_ = current_time;
}



GZ_REGISTER_WORLD_PLUGIN(GazeboLandmarks)
} // namespace gazebo_ros_landmarks
