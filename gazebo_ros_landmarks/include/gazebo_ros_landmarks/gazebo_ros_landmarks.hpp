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


#ifndef GAZEBO_ROS_LANDMARKS_PLUGIN_HPP
#define GAZEBO_ROS_LANDMARKS_PLUGIN_HPP

#include <string>

// Gazebo
#include <gazebo/gazebo.hh>
#include <gazebo/util/system.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo_ros/node.hpp>
#include <gazebo_ros/utils.hpp>
#include <gazebo_ros/conversions/geometry_msgs.hpp>
#include <gazebo_ros/conversions/builtin_interfaces.hpp>
#include <geometry_msgs/msg/wrench.hpp>

// ROS
#include <landmark_msgs/msg/landmark_measure_array.hpp>
#include <landmark_msgs/msg/landmark_measure.hpp>

#include <memory>
#include <string>

namespace gazebo_ros_landmarks
{

class GazeboLandmarksPrivate;

/// Broadcasts the inertial pose of an model's link via a nav_msgs/Odometry message on a ROS topic.
/**
  Example Usage:
  \code{.xml}
    <plugin name="gazebo_ros_landmarks" filename="libgazebo_ros_landmarks.so">

      <ros>

        <!-- Add a namespace -->
        <namespace>/demo</namespace>

        <!-- Remap the default topic -->
        <remapping>landmarks:=landmars_measures</remapping>

      </ros>

      <!-- Names of the landmarks -->
      <landmark_names>
        <landmark>box_link</landmark>
        <landmark>box_link_2</landmark>
      </landmark_names>

      <!-- Name of the robot model -->
      <robot_name>sphere_link</robot_name>
      <!-- Name of the reference link -->
      <robot_frame_name>sphere_link</robot_frame_name>

      <!-- Update rate in Hz, defaults to 0.0, which means as fast as possible -->
      <update_rate>1</update_rate>

      <!-- Noise parameters -->
      <!-- Standard deviation of the noise to be added to the landmark measurements. -->
      <gaussian_noise>0.01</gaussian_noise>

    </plugin>
  \endcode
*/

class GazeboLandmarks : public gazebo::WorldPlugin
{

public:
  /**
   * @brief Constructor for the GazeboLandmarksPlugin class.
   */
  GazeboLandmarks();

  /**
   * @brief Destructor for the GazeboLandmarks class.
   */
  virtual ~GazeboLandmarks();

  /**
   * @brief Load the plugin.
   * @param _sensor Pointer to the sensor that loaded this plugin.
   * @param _sdf SDF element that describes the plugin.
   */
  virtual void Load(gazebo::physics::WorldPtr _world, sdf::ElementPtr sdf);

private:
  /**
   * @brief Pointer to the implementation.
   */
  std::unique_ptr<GazeboLandmarksPrivate> impl_;
};

} // namespace gazebo_ros_landmarks

#endif // GAZEBO_ROS_LANDMARKS_PLUGIN_HPP
