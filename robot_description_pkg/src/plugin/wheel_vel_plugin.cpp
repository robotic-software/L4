#include <ros/ros.h>
#include "std_msgs/Float32MultiArray.h"

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>

using namespace std;
	
namespace gazebo
{
  class WheelsVelPlugin : public ModelPlugin
  {

	//---ROS
	private: ros::NodeHandle* _node_handle;
	private: ros::Publisher _w_v_pub;
	private: std_msgs::Float32MultiArray _w_vel;
	
	//---Gazebo
	private: physics::ModelPtr _model;
	private: physics::JointPtr _front_left_wheel_joint;
	private: physics::JointPtr _front_right_wheel_joint;
 
	private: event::ConnectionPtr updateConnection;

	
	public: void Load(physics::ModelPtr parent_, sdf::ElementPtr sdf_) {	
	
		_node_handle = new ros::NodeHandle();	
		_model = parent_;

		_front_left_wheel_joint = this->_model->GetJoint("front_left_wheel_joint");
		_front_right_wheel_joint = this->_model->GetJoint("front_right_wheel_joint");
		_w_v_pub = _node_handle->advertise< std_msgs::Float32MultiArray >("/diff_wheels/vel", 1);
		_w_vel.data.resize(2);


		this->updateConnection = event::Events::ConnectWorldUpdateBegin(std::bind(&WheelsVelPlugin::OnUpdate, this));
    
	}

    // Called by the world update start event
    public: void OnUpdate()  {
			_w_vel.data[0] = _front_left_wheel_joint->GetVelocity(0);
			_w_vel.data[1] = _front_right_wheel_joint->GetVelocity(0);
			_w_v_pub.publish( _w_vel );
		}
  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(WheelsVelPlugin)
}



