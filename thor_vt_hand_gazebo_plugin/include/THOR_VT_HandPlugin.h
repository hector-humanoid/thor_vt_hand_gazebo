#ifndef GAZEBO_THOR_VT_HAND_PLUGIN_HH
#define GAZEBO_THOR_VT_HAND_PLUGIN_HH

#include <string>
#include <vector>
#include <list>

#include <boost/thread/mutex.hpp>
//#include <boost/unordered/unordered_map.hpp>
#include <boost/thread.hpp>

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/advertise_options.h>
#include <ros/subscribe_options.h>

#include <std_msgs/String.h>

#include <gazebo/math/Vector3.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/physics/PhysicsTypes.hh>
#include <gazebo/transport/TransportTypes.hh>
#include <gazebo/common/Time.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Events.hh>
#include <gazebo/common/PID.hh>

#include <vigir_grasp_msgs/JointCommands.h>
#include <sensor_msgs/JointState.h>

#include <gazebo_plugins/PubQueue.h>

namespace gazebo
{
	namespace physics {
		class Collision;
	}

	class THOR_VT_HandPlugin : public ModelPlugin
	{
		/// \brief Constructor
	public: THOR_VT_HandPlugin();

			/// \brief Destructor
	public: virtual ~THOR_VT_HandPlugin();

			/// \brief Load the controller
	public: void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);

			/// \brief Update the controller
	private: void UpdateStates();
        private: int GetObjectDetection(
  const gazebo::physics::JointPtr &_joint, int _index, double _command,
  double _prevCommand);

			 /// \brief ROS callback queue thread
	private: void RosQueueThread();

			 /// \brief: thread out Load function with
			 /// with anything that might be blocking.
	private: void DeferredLoad();
	private: void CopyVectorIfValid(const std::vector<double> &from,
		std::vector<double> &to);

			 /// \brief Callback for contact messages
			 /// \param[in] _msg Gazebo contact message
	private: void OnContacts(ConstContactsPtr &_msg);

	typedef std::list<boost::shared_ptr<msgs::Contacts const> > ContactMsgs_L;

	private: physics::WorldPtr world;
	private: physics::ModelPtr model;

			 /// Which hand (left(l)/right(r))
	private: std::string side;
	private: std::string prefix;

  			/// \brief Velocity tolerance. Below this value we assume that the joint is
  			/// stopped (rad/s).
  	private: static const double VelTolerance = 0.002;

  			/// \brief Position tolerance. If the difference between target position and
  			/// current position is within this value we'll conclude that the joint
 			 /// reached its target (rad).
 	private: static const double PoseTolerance = 0.002;


			 /// Pointer to the update event connections
	private: event::ConnectionPtr updateConnection;

			 /// Throttle update rate
	private: double lastStatusTime;
	private: double updateRate;

			 // deferred loading in case ros is blocking
	private: sdf::ElementPtr sdf;
	private: boost::thread deferredLoadThread;

			 // ROS stuff
	private: ros::NodeHandle* rosNode;
	private: ros::CallbackQueue rosQueue;
	private: boost::thread callbackQueeuThread;
	private: ros::Publisher pubJointStates;
	private: PubQueue<sensor_msgs::JointState>::Ptr pubJointStatesQueue;

	private: ros::Subscriber subJointCommands;
	private: void SetJointCommands(
        const vigir_grasp_msgs::JointCommands::ConstPtr &_msg);

	private: std::vector<std::string> jointNames;
	private: physics::Joint_V joints;
	private: class ErrorTerms
	{
		double q_p;
		double d_q_p_dt;
		double q_i;
		double qd_p;
		friend class THOR_VT_HandPlugin;
	};
	private: std::vector<ErrorTerms> errorTerms;

        private: vigir_grasp_msgs::JointCommands jointCommands;

	private: sensor_msgs::JointState jointStates;

			 // Controls stuff
	private: common::Time lastControllerUpdateTime;

			 // ros publish multi queue, prevents publish() blocking
	private: PubMultiQueue* pmq;

			 /// \breif prevent overwriting commadns
	private: boost::mutex mutex;
	};
}
#endif
