/*
 * Copyright 2013 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

/*
   Desc: GazeboRosGpuLaser plugin for simulating ray sensors in Gazebo
   Author: Mihai Emanuel Dolha
   Date: 29 March 2012
 */

#include <algorithm>
#include <string>
#include <assert.h>

#include <sdf/sdf.hh>

#include <gazebo/physics/World.hh>
#include <gazebo/physics/HingeJoint.hh>
#include <gazebo/sensors/Sensor.hh>
#include <gazebo/common/Exception.hh>
#include "../gazebo/GpuSonarSensor.hh"
#include <gazebo/sensors/SensorTypes.hh>
#include <gazebo/transport/transport.hh>

#include <tf/tf.h>
#include <tf/transform_listener.h>

#include <gazebo_plugins/gazebo_ros_utils.h>

#include "gazebo_ros_gpu_sonar.h"

namespace gazebo
{
// Register this plugin with the simulator
GZ_REGISTER_SENSOR_PLUGIN(GazeboRosGpuSonar)

////////////////////////////////////////////////////////////////////////////////
// Constructor
GazeboRosGpuSonar::GazeboRosGpuSonar()
{
  this->seed = 0;
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
GazeboRosGpuSonar::~GazeboRosGpuSonar()
{
  ROS_DEBUG_STREAM_NAMED("gpu_sonar","Shutting down Gpu Sonar");
  this->rosnode_->shutdown();
  delete this->rosnode_;
  ROS_DEBUG_STREAM_NAMED("gpu_sonar","Unloaded");
}

////////////////////////////////////////////////////////////////////////////////
// Load the controller
void GazeboRosGpuSonar::Load(sensors::SensorPtr _parent, sdf::ElementPtr _sdf)
{
std::cout << "GazeboRosGpuSonar Load\n";

  // load plugin
  GpuSonarPlugin::Load(_parent, this->sdf);
  // Get the world name.
  std::string worldName = _parent->WorldName();
  this->world_ = physics::get_world(worldName);
  // save pointers
  this->sdf = _sdf;

  GAZEBO_SENSORS_USING_DYNAMIC_POINTER_CAST;
  this->parent_ray_sensor_ =
    dynamic_pointer_cast<sensors::GpuSonarSensor>(_parent);

  if (!this->parent_ray_sensor_)
    gzthrow("GazeboRosGpuSonar controller requires a GpuSonar Sensor as its parent");

  this->robot_namespace_ =  GetRobotNamespace(_parent, _sdf, "GpuSonar");

  if (!this->sdf->HasElement("frameName"))
  {
    ROS_INFO_NAMED("gpu_sonar", "GazeboRosGpuSonar plugin missing <frameName>, defaults to /world");
    this->frame_name_ = "/world";
  }
  else
    this->frame_name_ = this->sdf->Get<std::string>("frameName");

  if (!this->sdf->HasElement("topicName"))
  {
    ROS_INFO_NAMED("gpu_sonar", "GazeboRosGpuSonar plugin missing <topicName>, defaults to /world");
    this->topic_name_ = "/world";
  }
  else
    this->topic_name_ = this->sdf->Get<std::string>("topicName");

  this->sonar_connect_count_ = 0;


  // Make sure the ROS node for Gazebo has already been initialized
  if (!ros::isInitialized())
  {
    ROS_FATAL_STREAM_NAMED("gpu_sonar", "A ROS node for Gazebo has not been initialized, unable to load plugin. "
      << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
    return;
  }

  ROS_INFO_NAMED("gpu_sonar", "Starting GazeboRosGpuSonar Plugin (ns = %s)", this->robot_namespace_.c_str() );
  // ros callback queue for processing subscription
  this->deferred_load_thread_ = boost::thread(
    boost::bind(&GazeboRosGpuSonar::LoadThread, this));

}

////////////////////////////////////////////////////////////////////////////////
// Load the controller
void GazeboRosGpuSonar::LoadThread()
{
  this->gazebo_node_ = gazebo::transport::NodePtr(new gazebo::transport::Node());
  this->gazebo_node_->Init(this->world_name_);

  this->pmq.startServiceThread();

  this->rosnode_ = new ros::NodeHandle(this->robot_namespace_);

  this->tf_prefix_ = tf::getPrefixParam(*this->rosnode_);
  if(this->tf_prefix_.empty()) {
      this->tf_prefix_ = this->robot_namespace_;
      boost::trim_right_if(this->tf_prefix_,boost::is_any_of("/"));
  }
  ROS_INFO_NAMED("gpu_sonar", "NPS Beam Plugin (ns = %s) <tf_prefix_>, set to \"%s\"",
             this->robot_namespace_.c_str(), this->tf_prefix_.c_str());

  // resolve tf prefix
  this->frame_name_ = tf::resolve(this->tf_prefix_, this->frame_name_);

  if (this->topic_name_ != "")
  {
    ros::AdvertiseOptions ao =
      ros::AdvertiseOptions::create<sensor_msgs::LaserScan>(
      this->topic_name_, 1,
      boost::bind(&GazeboRosGpuSonar::SonarConnect, this),
      boost::bind(&GazeboRosGpuSonar::SonarDisconnect, this),
      ros::VoidPtr(), NULL);
    this->pub_ = this->rosnode_->advertise(ao);
    this->pub_queue_ = this->pmq.addPub<sensor_msgs::LaserScan>();
  }

  // Initialize the controller

  // sensor generation off by default
  this->parent_ray_sensor_->SetActive(false);

  ROS_INFO_STREAM_NAMED("gpu_sonar","LoadThread function completed");
}

////////////////////////////////////////////////////////////////////////////////
// Increment count
void GazeboRosGpuSonar::SonarConnect()
{
  this->sonar_connect_count_++;
  if (this->sonar_connect_count_ == 1)
    this->sonar_scan_sub_ =
      this->gazebo_node_->Subscribe(this->parent_ray_sensor_->Topic(),
                                    &GazeboRosGpuSonar::OnScan, this);
}

////////////////////////////////////////////////////////////////////////////////
// Decrement count
void GazeboRosGpuSonar::SonarDisconnect()
{
  this->sonar_connect_count_--;
  if (this->sonar_connect_count_ == 0)
    this->sonar_scan_sub_.reset();
}

/* original code copies Gazebo _msg to ROS sonar_msg and publishes 
////////////////////////////////////////////////////////////////////////////////
// Convert new Gazebo message to ROS message and publish it
void GazeboRosGpuSonar::OnScan(ConstLaserScanStampedPtr &_msg)
{
  // We got a new message from the Gazebo sensor.  Stuff a
  // corresponding ROS message and publish it.
  sensor_msgs::SonarScan sonar_msg;
  sonar_msg.header.stamp = ros::Time(_msg->time().sec(), _msg->time().nsec());
  sonar_msg.header.frame_id = this->frame_name_;
  sonar_msg.angle_min = _msg->scan().angle_min();
  sonar_msg.angle_max = _msg->scan().angle_max();
  sonar_msg.angle_increment = _msg->scan().angle_step();
  sonar_msg.time_increment = 0;  // instantaneous simulator scan
  sonar_msg.scan_time = 0;  // not sure whether this is correct
  sonar_msg.range_min = _msg->scan().range_min();
  sonar_msg.range_max = _msg->scan().range_max();
  sonar_msg.ranges.resize(_msg->scan().ranges_size());
  std::copy(_msg->scan().ranges().begin(),
            _msg->scan().ranges().end(),
            sonar_msg.ranges.begin());
  sonar_msg.intensities.resize(_msg->scan().intensities_size());
  std::copy(_msg->scan().intensities().begin(),
            _msg->scan().intensities().end(),
            sonar_msg.intensities.begin());
  this->pub_queue_->push(sonar_msg, this->pub_);
}
*/

/* NPS code copies much of Gazebo _msg to ROS sonar_msg but only returns
   one processed range and intensity value.
*/
////////////////////////////////////////////////////////////////////////////////
// Convert new Gazebo message to ROS message and publish it
void GazeboRosGpuSonar::OnScan(ConstLaserScanStampedPtr &_msg)
{
  // We got a new message from the Gazebo sensor.  Stuff a
  // corresponding ROS message and publish it.
  sensor_msgs::LaserScan sonar_msg;
  sonar_msg.header.stamp = ros::Time(_msg->time().sec(), _msg->time().nsec());
  sonar_msg.header.frame_id = this->frame_name_;
  sonar_msg.angle_min = _msg->scan().angle_min();
  sonar_msg.angle_max = _msg->scan().angle_max();
  sonar_msg.angle_increment = _msg->scan().angle_step();
  sonar_msg.time_increment = 0;  // instantaneous simulator scan
  sonar_msg.scan_time = 0;  // not sure whether this is correct
  sonar_msg.range_min = _msg->scan().range_min();
  sonar_msg.range_max = _msg->scan().range_max();

  // ROS output message size is 1
  sonar_msg.ranges.resize(0);
  sonar_msg.intensities.resize(0);

  // calculate range and intensity from Gazebo array
  float angle = sonar_msg.angle_min;
  float increment = sonar_msg.angle_increment;
  float intensity = 0.0;
  float intensity_ref = 2.6e-16;
  float absorption = 5e-5;
  float echo_level = 0.0;
  float source_level = 120.0;
  float transmission_loss = 0.0;
  float target_strength = 10.0;
  float range = sonar_msg.range_max - sonar_msg.range_min;
  auto range_it = _msg->scan().ranges().begin();
  auto intensity_it = _msg->scan().intensities().begin();
  while (range_it != _msg->scan().ranges().end())
  {
    // Andi is going to put some code here to implement sonar eqn's.
    // sum of f(intensity, angle)
    intensity += *intensity_it * cos(10.0 * angle);
    // min of range
    range = *range_it < range ? *range_it : range;

    /*
    // calculate target strength

    target_strength = 10 * log(intensity/intensity_ref)
 

    // calculate transmission loss
    transmission_loss = 20 * log(range) + absorption*range;


    // echo level will eventually be used to display intensities in dB on a sonar viewer
    // calculate echo level
    echo_level = source_level - 2 * (transmission_loss) + target_strength;
    */

    // next
    ++range_it;
    ++intensity_it;
    angle = angle + increment;
  }

  // store calculated range and intensity
  sonar_msg.ranges.push_back(range);
  sonar_msg.intensities.push_back(intensity);

  // publish
  this->pub_queue_->push(sonar_msg, this->pub_);
}
}

