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
 * Desc: Ros Laser controller.
 * Author: Nathan Koenig
 * Date: 01 Feb 2007
 */

#ifndef _CUSTOM_GAZEBO_ROS_LASER_HH_
#define _CUSTOM_GAZEBO_ROS_LASER_HH_


#include <thread>

#include <ros/ros.h>
#include <ros/advertise_options.h>
#include <sensor_msgs/LaserScan.h>

#include <sdf/Param.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/TransportTypes.hh>
#include <gazebo/msgs/MessageTypes.hh>
#include <gazebo/common/Time.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Events.hh>
#include <gazebo/sensors/SensorTypes.hh>
#include <gazebo/plugins/RayPlugin.hh>
#include <gazebo_plugins/gazebo_ros_utils.h>

#include <gazebo_plugins/PubQueue.h>

#include <algorithm>
#include <string>
#include <assert.h>

#include <gazebo/physics/World.hh>
#include <gazebo/physics/HingeJoint.hh>
#include <gazebo/sensors/Sensor.hh>
#include <sdf/sdf.hh>
#include <sdf/Param.hh>
#include <gazebo/common/Exception.hh>
#include <gazebo/sensors/RaySensor.hh>
#include <gazebo/sensors/SensorTypes.hh>
#include <gazebo/transport/transport.hh>

#include <tf/tf.h>
#include <tf/transform_listener.h>

namespace gazebo
{
	


class GAZEBO_VISIBLE CustomGazeboRosLaserPlugin : public RayPlugin
  {



    private: int laser_connect_count_;


    // Pointer to the model
    GazeboRosPtr gazebo_ros_;
    private: std::string world_name_;
    private: physics::WorldPtr world_;
    private: sensors::RaySensorPtr parent_ray_sensor_;

    private: ros::NodeHandle* rosnode_;
    private: ros::Publisher pub_;
    private: PubQueue<sensor_msgs::LaserScan>::Ptr pub_queue_;

    private: std::string topic_name_;

    private: std::string frame_name_;
    
    private: std::string tf_prefix_;

    private: std::string robot_namespace_;
    
    private: double updateRate_;

    // deferred load in case ros is blocking
    private: sdf::ElementPtr sdf;
    private: std::thread deferred_load_thread_;
    private: unsigned int seed;

    private: gazebo::transport::NodePtr gazebo_node_;
    private: gazebo::transport::SubscriberPtr laser_scan_sub_;

    private: PubMultiQueue pmq;

// Constructor
public: CustomGazeboRosLaserPlugin()
{
  this->seed = 0;
}
/*
// Destructor
public: virtual ~CustomGazeboRosLaserPlugin()
{
  this->rosnode_->shutdown();
  delete this->rosnode_;
}
*/
// Load the controller
public: void Load(sensors::SensorPtr _parent, sdf::ElementPtr _sdf )
{

  // load plugin
  ROS_INFO("Trying to load RayPlugin");
  RayPlugin::Load(_parent, this->sdf);
  // Get the world name.
# if GAZEBO_MAJOR_VERSION >= 7
  std::string worldName = _parent->WorldName();
# else
  std::string worldName = _parent->GetWorldName();
# endif

  this->world_ = physics::get_world(worldName);
  // save pointers
  this->sdf = _sdf;

  GAZEBO_SENSORS_USING_DYNAMIC_POINTER_CAST;
  this->parent_ray_sensor_ =
    dynamic_pointer_cast<sensors::RaySensor>(_parent);

  if (!this->parent_ray_sensor_)
    gzthrow("CustomGazeboRosLaserPlugin controller requires a Ray Sensor as its parent");

  this->robot_namespace_ =  GetRobotNamespace(_parent, _sdf, "Laser");

  if (!this->sdf->HasElement("frameName"))
  {
    ROS_INFO("Laser plugin missing <frameName>, defaults to /world");
    this->frame_name_ = "/world";
  }
  else
    this->frame_name_ = this->sdf->Get<std::string>("frameName");


  if (!this->sdf->HasElement("topicName"))
  {
    ROS_INFO("Laser plugin missing <topicName>, defaults to /world");
    this->topic_name_ = "/world";
  }
  else
    this->topic_name_ = this->sdf->Get<std::string>("topicName");
    
  if (!this->sdf->HasElement("updateRate"))
  {
    ROS_INFO("Laser plugin missing <updateRate>, defaults to 10.0");
    this->updateRate_ = 10.0;
  }
  else
    this->updateRate_ = this->sdf->Get<double>("updateRate");    

  this->laser_connect_count_ = 0;

    // Make sure the ROS node for Gazebo has already been initialized
  if (!ros::isInitialized())
  {
    ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
      << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
    return;
  }

  //ROS_INFO ( "Starting Laser Plugin (ns = %s)!", this->robot_namespace_.c_str() );
  //// ros callback queue for processing subscription
  //this->deferred_load_thread_ = std::thread(
    //std::bind(&CustomGazeboRosLaserPlugin::LoadThread, this));

}

/*
// Load the controller
public: void LoadThread()
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
  ROS_INFO("Laser Plugin (ns = %s)  <tf_prefix_>, set to \"%s\"",
             this->robot_namespace_.c_str(), this->tf_prefix_.c_str());

  // resolve tf prefix
  this->frame_name_ = tf::resolve(this->tf_prefix_, this->frame_name_);

  if (this->topic_name_ != "")
  {
    ros::AdvertiseOptions ao =
      ros::AdvertiseOptions::create<sensor_msgs::LaserScan>(
      this->topic_name_, 1,
      boost::bind(&CustomGazeboRosLaserPlugin::LaserConnect, this),
      boost::bind(&CustomGazeboRosLaserPlugin::LaserDisconnect, this),
      ros::VoidPtr(), NULL);
    this->pub_ = this->rosnode_->advertise(ao);
    this->pub_queue_ = this->pmq.addPub<sensor_msgs::LaserScan>();
  }

  // Initialize the controller

  // sensor generation on by default
  this->parent_ray_sensor_->SetActive(true);

  
}

// Increment count
public: void LaserConnect()
{
  this->laser_connect_count_++;
  if (this->laser_connect_count_ == 1)
    this->laser_scan_sub_ =
# if GAZEBO_MAJOR_VERSION >= 7
      this->gazebo_node_->Subscribe(this->parent_ray_sensor_->Topic(),
# else
      this->gazebo_node_->Subscribe(this->parent_ray_sensor_->GetTopic(),
# endif
                                    &CustomGazeboRosLaserPlugin::OnScan, this);
}

// Decrement count
public: void LaserDisconnect()
{
  this->laser_connect_count_--;
  if (this->laser_connect_count_ == 0)
    this->laser_scan_sub_.reset();
}

// Convert new Gazebo message to ROS message and publish it
public: void OnScan(ConstLaserScanStampedPtr &_msg)
{
  // We got a new message from the Gazebo sensor.  Stuff a
  // corresponding ROS message and publish it.
  sensor_msgs::LaserScan laser_msg;
  laser_msg.header.stamp = ros::Time(_msg->time().sec(), _msg->time().nsec());
  laser_msg.header.frame_id = this->frame_name_;
  laser_msg.angle_min = _msg->scan().angle_min();
  laser_msg.angle_max = _msg->scan().angle_max();
  laser_msg.angle_increment = _msg->scan().angle_step();
  laser_msg.time_increment = 0;  // instantaneous simulator scan
  laser_msg.scan_time = 0;  // not sure whether this is correct
  laser_msg.range_min = _msg->scan().range_min();
  laser_msg.range_max = _msg->scan().range_max();
  laser_msg.ranges.resize(_msg->scan().ranges_size());
  std::copy(_msg->scan().ranges().begin(),
            _msg->scan().ranges().end(),
            laser_msg.ranges.begin());
  laser_msg.intensities.resize(_msg->scan().intensities_size());
  std::copy(_msg->scan().intensities().begin(),
            _msg->scan().intensities().end(),
            laser_msg.intensities.begin());
  this->pub_queue_->push(laser_msg, this->pub_);
}
* */
  };
// Register this plugin with the simulator
GZ_REGISTER_SENSOR_PLUGIN(CustomGazeboRosLaserPlugin)  

}
# endif
