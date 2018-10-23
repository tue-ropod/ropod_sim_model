#ifndef _ODOMETRY_PLUGIN_HH_
#define _ODOMETRY_PLUGIN_HH_



#include <tue/profiling/timer.h>
#include <geolib/ros/msg_conversions.h>

#include <geolib/Box.h>

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <thread>
#include "ros/ros.h"
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"
#include <tf/tf.h>
#include <nav_msgs/Odometry.h>
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"



namespace gazebo
{
/// \brief A plugin to control a Drive sensor.
class OdometryPlugin : public ModelPlugin
{
ros::Duration updateRate;
ros::Duration actualupdateRate;
ros::Duration commandRate;
ros::Time prevCommandTime;
ros::Time prevUpdateTime;
math::Vector3 cmd_linvel;
math::Vector3 cmd_angvel;
// parameters
double updateRate_in;
double updateRate_def = 20; // default
std::string veltopicName;
std::string odomtopicName;
std::string robotBaseFrame;
std::string odomFrame;
/// \brief Constructor
public: OdometryPlugin() {		
	this->updateRate_in  = updateRate_def;
	this->veltopicName = std::string ("cmd_vel");
	this->odomtopicName = std::string ("odom");
	this->robotBaseFrame = std::string ("base_link");
	this->odomFrame = std::string ("odom");
	this->cmd_linvel = math::Vector3(0.0,0.0,0.0);
	this->cmd_angvel = math::Vector3(0.0,0.0,0.0);
	this->commandRate = ros::Duration(0,(int32_t)1e9/2); // commands should be received at least at 2Hz
	
}


public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
	// Load parameters
	

	ROS_INFO("Loading Odometry Gazebo Plugin");
	
	if (_sdf->HasElement("updateRate"))
		this->updateRate_in = _sdf->Get<double>("updateRate");
	if (updateRate_in<1){
		 this->updateRate_in = updateRate_def;
		 ROS_INFO("Update rate <1, reset to default %f Hz",updateRate_def);
	 }
	 
	 

	if (_sdf->HasElement("veltopicName"))
		this->veltopicName = std::string(_sdf->Get<std::string>("veltopicName")); 
	
	if (_sdf->HasElement("odomtopicName"))
		this->odomtopicName = std::string(_sdf->Get<std::string>("odomtopicName")); 	 
	
	if (_sdf->HasElement("robotBaseFrame"))
		this->robotBaseFrame = std::string(_sdf->Get<std::string>("robotBaseFrame")); 
			
	if (_sdf->HasElement("odomFrame"))
		this->odomFrame = std::string(_sdf->Get<std::string>("odomFrame")); 
		
					 
	// Store the model pointer for convenience.
	this->model = _model;
	
	// Initialize ros, if it has not already bee initialized.
	if (!ros::isInitialized())
	{
		int argc = 0;
		char **argv = NULL;
		ros::init(argc, argv, "gazebo_client",
		ros::init_options::NoSigintHandler);
	}
	
	// Create our ROS node. This acts in a similar manner to
	// the Gazebo node
	this->rosNode.reset(new ros::NodeHandle("gazebo_client"));
	
	
	
	// Create a named topic, and subscribe to it.
	ros::SubscribeOptions so=
	ros::SubscribeOptions::create<geometry_msgs::Twist>
	(this->veltopicName, 1,
	boost::bind(&OdometryPlugin::ROSCallback_Vel, this, _1),
	
	ros::VoidPtr(), &this->rosQueue);
	this->rosSub = this->rosNode->subscribe(so);
	
	
	ros::NodeHandle n("gazebo_client");
	
	this->odom_pub = n.advertise<nav_msgs::Odometry>(this->odomtopicName, 1);
	
	// set the update rate (you can also set it as a parameter from the sdf file)
	this->updateRate = ros::Duration(0,(int32_t)1e9/this->updateRate_in);
	ROS_INFO("Odometry update rate: %f Hz",1/this->updateRate.toSec());
	// initialize the prevUpdateTime
	this->prevUpdateTime = ros::Time::now();
	
	this->prevCommandTime = ros::Time::now();
	
	
	
	this->updateConnection = event::Events::ConnectWorldUpdateBegin(
	boost::bind(&OdometryPlugin::OnUpdate, this));
	
	// Spin up the queue helper thread.
	this->rosQueueThread =
	std::thread(std::bind(&OdometryPlugin::QueueThread, this));

}

public: void OnUpdate()
{
	math::Vector3 p;
	math::Quaternion r;
	math::Vector3 AngPos;
	math::Vector3 Angvel;
	math::Vector3 Linvel;	
	math::Quaternion rpi(0.0,0.0,0.0);
	double velx_cmd, vely_cmd, velangz_cmd;	
	double vlinx=0.0,vliny=0.0,vlinz=0.0, vangx=0.0, vangy=0.0, vangz=0.0;
	
	// read pose
	p = rpi.RotateVector(model->GetWorldPose().pos); 
	r = model->GetWorldPose().rot;				
	AngPos = r.GetAsEuler()+rpi.GetAsEuler();
	
	// Always set velocities
	if (ros::Time::now() - this->prevCommandTime < this->commandRate){
		// rotate vector to emulate robot coordinates vel command
		velx_cmd = this->cmd_linvel.x*cos(AngPos.z) - this->cmd_linvel.y*sin(AngPos.z);
		vely_cmd = this->cmd_linvel.x*sin(AngPos.z) + this->cmd_linvel.y*cos(AngPos.z);	
		velangz_cmd = this->cmd_angvel.z;
                model->SetLinearVel( math::Vector3(velx_cmd,vely_cmd,0.0));  
                model->SetAngularVel( math::Vector3(0.0,0.0,velangz_cmd));
	}
/*	else{
		velx_cmd = 0.0;
		vely_cmd = 0.0;
		velangz_cmd = 0.0;	
	}*/		
	
	this -> actualupdateRate = ros::Time::now() - this->prevUpdateTime;
	
	/* ******* publish odom data at desired rate **** */
	if (this -> actualupdateRate < this->updateRate)
    return;
	

	r = math::Quaternion(AngPos);
	       
	Linvel = model->GetWorldLinearVel();
	Angvel = model->GetWorldAngularVel();
	vlinx =   Linvel.x*cos(AngPos.z) + Linvel.y*sin(AngPos.z);
	vliny = - Linvel.x*sin(AngPos.z) + Linvel.y*cos(AngPos.z);
		 
	
	vlinz=0.0;
	vangx=0.0;
	vangy=0.0;
	vangz=Angvel.z;
	
	
	// *****************************************************
	// *********** ODOMETRY DATA ************************************ 
	geometry_msgs::Quaternion odom_quat;
	odom_quat.x=0.0; // the robot moves in a 2D plane
	odom_quat.y=0.0;
	odom_quat.z=r.z;
	odom_quat.w=r.w;
	
	//publish the odometry message overtoROS
	nav_msgs::Odometry odom;
	odom.header.stamp =ros::Time::now(); 
	odom.header.frame_id = this->odomFrame;
	odom.child_frame_id = this->robotBaseFrame;
	
	//set the position for odom message
	odom.pose.pose.position.x = p.x;
	odom.pose.pose.position.y = p.y;
	odom.pose.pose.position.z = 0.0; // the robot moves in a 2D plane
	odom.pose.pose.orientation = odom_quat;
	
	//set the velocity for odom message
	odom.twist.twist.linear.x = vlinx;
	odom.twist.twist.linear.y = vliny;
	odom.twist.twist.linear.z = vlinz;
	odom.twist.twist.angular.x = vangx;
	odom.twist.twist.angular.y = vangy;
	odom.twist.twist.angular.z = vangz;
		
	//publish the message
	this->odom_pub.publish(odom);
	this->prevUpdateTime = ros::Time::now();
	
	
}



void ROSCallback_Vel(const geometry_msgs::Twist::ConstPtr &msg)
{
	// ROS_INFO("I received odom: [%lf,%lf,%lf,%lf,%lf, %lf]",msg->linear.x,msg->linear.y,msg->linear.z,msg->angular.x,msg->angular.y,msg->angular.z);
	this->prevCommandTime = ros::Time::now();
	this->cmd_linvel.x = msg->linear.x;
	this->cmd_linvel.y = msg->linear.y;	
	this->cmd_angvel.z = msg->angular.z;

}



/// \brief ROS helper function that processes messages
private: void QueueThread()
{
	static const double timeout = 0.01;
	while (this->rosNode->ok())
	{
		this->rosQueue.callAvailable(ros::WallDuration(timeout));
	}
}




private: event::ConnectionPtr updateConnection;


/// \brief Pointer to the model.
private: physics::ModelPtr model;

/// \brief Pointer to the link.
private: physics::LinkPtr link;

/// \brief A node use for ROS transport
private: std::unique_ptr<ros::NodeHandle> rosNode;

/// \brief A ROS subscriber
private: ros::Subscriber rosSub;

/// \brief A ROS callbackqueue that helps process messages
private: ros::CallbackQueue rosQueue;

/// \brief A thread the keeps running the rosQueue
private: std::thread rosQueueThread;
/// ROS Publisher



private: ros::Publisher odom_pub;

};

// Tell Gazebo about this plugin, so that Gazebo can call Load on this plugin.
GZ_REGISTER_MODEL_PLUGIN(OdometryPlugin)
}
#endif

