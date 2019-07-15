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
namespace physics
{
class Collision;
}

class THOR_VT_HandPlugin : public ModelPlugin
{
public:
  /// \brief Constructor
  THOR_VT_HandPlugin();

  /// \brief Destructor
  virtual ~THOR_VT_HandPlugin();

  /// \brief Load the controller
  void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf) override;

  /// \brief Update the controller
private:
  void UpdateStates();

  int GetObjectDetection(const gazebo::physics::JointPtr& _joint, int _index, double _command, double _prevCommand);

  /// \brief ROS callback queue thread
  void RosQueueThread();

  /// \brief: thread out Load function with
  /// with anything that might be blocking.
  void DeferredLoad();

  void CopyVectorIfValid(const std::vector<double>& from, std::vector<double>& to);

  /// \brief Callback for contact messages
  /// \param[in] _msg Gazebo contact message
  void OnContacts(ConstContactsPtr& _msg);

  typedef std::list<boost::shared_ptr<msgs::Contacts const> > ContactMsgs_L;

  physics::WorldPtr world;
  physics::ModelPtr model;

  /// Which hand (left(l)/right(r))
  std::string side;

  std::string prefix;

  /// \brief Velocity tolerance. Below this value we assume that the joint is
  /// stopped (rad/s).
  static constexpr double VelTolerance = 0.002;

  /// \brief Position tolerance. If the difference between target position and
  /// current position is within this value we'll conclude that the joint
  /// reached its target (rad).
  static constexpr double PoseTolerance = 0.002;

  /// Pointer to the update event connections
  event::ConnectionPtr updateConnection;

  /// Throttle update rate
  double lastStatusTime;

  double updateRate;

  // deferred loading in case ros is blocking
  sdf::ElementPtr sdf;

  boost::thread deferredLoadThread;

  // ROS stuff
  ros::NodeHandle* rosNode;

  ros::CallbackQueue rosQueue;

  boost::thread callbackQueeuThread;

  ros::Publisher pubJointStates;

  PubQueue<sensor_msgs::JointState>::Ptr pubJointStatesQueue;

  ros::Subscriber subJointCommands;

  void SetJointCommands(const vigir_grasp_msgs::JointCommands::ConstPtr& _msg);

  std::vector<std::string> jointNames;

  physics::Joint_V joints;

  class ErrorTerms
  {
    double q_p;
    double d_q_p_dt;
    double q_i;
    double qd_p;
    friend class THOR_VT_HandPlugin;
  };

  std::vector<ErrorTerms> errorTerms;

  vigir_grasp_msgs::JointCommands jointCommands;

  sensor_msgs::JointState jointStates;

  // Controls stuff
  common::Time lastControllerUpdateTime;

  // ros publish multi queue, prevents publish() blocking
  PubMultiQueue* pmq;

  /// \brief prevent overwriting commadns
  boost::mutex mutex;
};
}  // namespace gazebo
#endif
