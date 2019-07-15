#include <string>
#include <vector>

#include <gazebo/transport/Node.hh>
#include "THOR_VT_HandPlugin.h"

static bool wasMoving[2] = { false, false };
static bool underactuating[2] = { false, false };
static double prev_jointCommands[4] = { 0, 0, 0, 0 };
static double tmp_jointCommands[4] = { 0, 0, 0, 0 };

using std::string;

namespace gazebo
{
GZ_REGISTER_MODEL_PLUGIN(THOR_VT_HandPlugin)

////////////////////////////////////////////////////////////////////////////////
// Constructor
THOR_VT_HandPlugin::THOR_VT_HandPlugin()
{
  this->pmq = new PubMultiQueue();
  this->rosNode = NULL;
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
THOR_VT_HandPlugin::~THOR_VT_HandPlugin()
{
  delete this->pmq;
  this->rosNode->shutdown();
  this->rosQueue.clear();
  this->rosQueue.disable();
  this->callbackQueeuThread.join();
  delete this->rosNode;
}

////////////////////////////////////////////////////////////////////////////////
int THOR_VT_HandPlugin::GetObjectDetection(const gazebo::physics::JointPtr& _joint, int _index, double _command, double _prevCommand)
{
  // Check finger's speed.
  bool isMoving = _joint->GetVelocity(0) > 0.002;

  // Check if the finger reached its target positions. We look at the error in
  // the position PID to decide if reached the target.
  bool reachPosition = (fabs(_command - _joint->Position(0)) < 0.02);

  if (isMoving)
  {
    // Finger is in motion.
    return 0;
  }
  else
  {
    if (reachPosition)
    {
      // Finger is at the requestedPosition.
      return 3;
    }
    else if (_command - _prevCommand > 0)
    {
      // Finger has stopped due to a contact while closing.
      return 2;
    }
    else if (_command - _prevCommand < 0)
    {
      // Finger has stopped due to a contact while opening.
      return 1;
    }
    else
    {
      // Finger stuck.
      return -1;
    }
  }
  wasMoving[_index] = isMoving;
}

////////////////////////////////////////////////////////////////////////////////
// Load the controller
void THOR_VT_HandPlugin::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
{
  this->model = _parent;

  // Get the world name.
  this->world = this->model->GetWorld();
  this->sdf = _sdf;
  this->lastControllerUpdateTime = this->world->SimTime();

  // determine which hand (left/right)
  if (!this->sdf->HasElement("prefix") || !this->sdf->GetElement("prefix")->GetValue()->Get(this->prefix) || ((this->prefix != "l") && (this->prefix != "r")))
  {
    gzerr << "Failed to determine which hand we're controlling; "
             "aborting plugin load. <prefix> should be either 'l' or 'r'."
          << std::endl;
    return;
  }

  gzlog << "THOR_VT_HandPlugin loading for " << this->prefix << " hand." << std::endl;

  // get joints
  this->jointNames.push_back(this->prefix + "_f0_j0");
  this->jointNames.push_back(this->prefix + "_f1_j0");
  this->jointNames.push_back(this->prefix + "_f0_j1");
  this->jointNames.push_back(this->prefix + "_f1_j1");

  this->joints.resize(this->jointNames.size());

  // Get hand joints
  {
    unsigned int jointCount = 0;
    for (unsigned int i = 0; i < this->joints.size(); ++i)
    {
      this->joints[i] = this->model->GetJoint(this->jointNames[i]);
      if (this->joints[i])
        ++jointCount;
    }
    if (jointCount != this->joints.size())
    {
      ROS_ERROR("Error loading thor_vt hand joints, plugin not loaded");
      return;
    }
  }

  this->errorTerms.resize(this->joints.size());

  this->jointStates.name.resize(this->joints.size());
  this->jointStates.position.resize(this->joints.size());
  this->jointStates.velocity.resize(this->joints.size());
  this->jointStates.effort.resize(this->joints.size());

  for (unsigned int i = 0; i < this->joints.size(); ++i)
  {
    this->jointStates.name[i] = this->jointNames[i];
    this->jointStates.position[i] = 0;
    this->jointStates.velocity[i] = 0;
    this->jointStates.effort[i] = 0;
  }

  this->jointCommands.name.resize(this->joints.size());
  this->jointCommands.position.resize(this->joints.size());
  this->jointCommands.velocity.resize(this->joints.size());
  this->jointCommands.effort.resize(this->joints.size());
  this->jointCommands.kp_position.resize(this->joints.size());
  this->jointCommands.ki_position.resize(this->joints.size());
  this->jointCommands.kd_position.resize(this->joints.size());
  this->jointCommands.kp_velocity.resize(this->joints.size());
  this->jointCommands.i_effort_min.resize(this->joints.size());
  this->jointCommands.i_effort_max.resize(this->joints.size());

  for (unsigned i = 0; i < this->joints.size(); ++i)
  {
    this->errorTerms[i].q_p = 0;
    this->errorTerms[i].d_q_p_dt = 0;
    this->errorTerms[i].q_i = 0;
    this->errorTerms[i].qd_p = 0;

    this->jointCommands.name[i] = this->joints[i]->GetScopedName();
    this->jointCommands.position[i] = 0;
    this->jointCommands.velocity[i] = 0;
    this->jointCommands.effort[i] = 0;
    this->jointCommands.kp_position[i] = 0;
    this->jointCommands.ki_position[i] = 0;
    this->jointCommands.kd_position[i] = 0;
    this->jointCommands.kp_velocity[i] = 0;
    this->jointCommands.i_effort_min[i] = 0;
    this->jointCommands.i_effort_max[i] = 0;
  }

  // ros callback queue for processing subscription
  this->deferredLoadThread = boost::thread(boost::bind(&THOR_VT_HandPlugin::DeferredLoad, this));
}

////////////////////////////////////////////////////////////////////////////////
// Load the controller
void THOR_VT_HandPlugin::DeferredLoad()
{
  // initialize ros
  if (!ros::isInitialized())
  {
    gzerr << "Not loading plugin since ROS hasn't been "
          << "properly initialized.  Try starting gazebo with ros plugin:\n"
          << "  gazebo -s libgazebo_ros_api_plugin.so\n";
    return;
  }

  // ros stuff
  this->rosNode = new ros::NodeHandle("");

  // publish multi queue
  this->pmq->startServiceThread();

  // pull down controller parameters; they should be on the param server by now
  const int NUM_FINGERS = 2, NUM_FINGER_JOINTS = 2;
  for (int finger = 0; finger < NUM_FINGERS; finger++)
  {
    for (int joint = 0; joint < NUM_FINGER_JOINTS; joint++)
    {
      char joint_ns[200] = "";
      snprintf(joint_ns, sizeof(joint_ns), "thor_vt_hands/gains/%s_f%d_j%d/", this->prefix.c_str(), finger, joint);

      double p_val = 0, i_val = 0, d_val = 0, i_clamp_val = 0;
      string p_str = string(joint_ns) + "p";
      string i_str = string(joint_ns) + "i";
      string d_str = string(joint_ns) + "d";
      string i_clamp_str = string(joint_ns) + "i_clamp";
      if (!this->rosNode->getParam(p_str, p_val) || !this->rosNode->getParam(i_str, i_val) || !this->rosNode->getParam(d_str, d_val) || !this->rosNode->getParam(i_clamp_str, i_clamp_val))
      {
        ROS_ERROR("couldn't find a param for %s", joint_ns);
        continue;
      }
      int joint_idx = finger * NUM_FINGER_JOINTS + joint;
      this->jointCommands.kp_position[joint_idx] = p_val;
      this->jointCommands.ki_position[joint_idx] = i_val;
      this->jointCommands.kd_position[joint_idx] = d_val;
      this->jointCommands.i_effort_min[joint_idx] = -i_clamp_val;
      this->jointCommands.i_effort_max[joint_idx] = i_clamp_val;
    }
  }

  // ROS Controller API

  // ros publication / subscription
  /// brief broadcasts the robot states
  std::string topic_base = std::string("thor_vt_hands/") + this->prefix;
  this->pubJointStatesQueue = this->pmq->addPub<sensor_msgs::JointState>();
  this->pubJointStates = this->rosNode->advertise<sensor_msgs::JointState>(topic_base + std::string("_hand/joint_states"), 10);

  // ros topic subscriptions
  ros::SubscribeOptions jointCommandsSo =
      ros::SubscribeOptions::create<vigir_grasp_msgs::JointCommands>(topic_base + std::string("_hand/joint_commands"), 100, boost::bind(&THOR_VT_HandPlugin::SetJointCommands, this, _1), ros::VoidPtr(), &this->rosQueue);
  this->subJointCommands = this->rosNode->subscribe(jointCommandsSo);

  // initialize status pub time
  this->lastStatusTime = this->world->SimTime().Double();
  this->updateRate = 1.0;

  // ros callback queue for processing subscription
  this->callbackQueeuThread = boost::thread(boost::bind(&THOR_VT_HandPlugin::RosQueueThread, this));

  this->updateConnection = event::Events::ConnectWorldUpdateBegin(boost::bind(&THOR_VT_HandPlugin::UpdateStates, this));
}

////////////////////////////////////////////////////////////////////////////////

void THOR_VT_HandPlugin::UpdateStates()
{
  common::Time curTime = this->world->SimTime();

  if (curTime > this->lastControllerUpdateTime)
  {
    // populate FromRobot from robot
    this->jointStates.header.stamp = ros::Time(curTime.sec, curTime.nsec);
    for (unsigned int i = 0; i < this->joints.size(); ++i)
    {
      this->jointStates.position[i] = this->joints[i]->Position(0);
      this->jointStates.velocity[i] = this->joints[i]->GetVelocity(0);
      // better to use GetForceTorque dot joint axis
      this->jointStates.effort[i] = this->joints[i]->GetForce(0u);
    }
    this->pubJointStatesQueue->push(this->jointStates, this->pubJointStates);

    double dt = (curTime - this->lastControllerUpdateTime).Double();

    /// update pid with feedforward force
    double position;
    double velocity;
    for (unsigned int i = 0; i < this->joints.size(); ++i)
    {
      position = this->jointStates.position[i];
      velocity = this->jointStates.velocity[i];
      int state;

      double force;
      {
        boost::mutex::scoped_lock lock(this->mutex);

        double q_p;
        if (i <= 1)
        {
          q_p = 3 - position;

          if (underactuating[i] && (q_p - this->errorTerms[i].q_p < 0))
          {
            this->errorTerms[i].d_q_p_dt = 0;
          }
          else if (!ignition::math::equal(dt, 0.0))
          {
            this->errorTerms[i].d_q_p_dt = (q_p - this->errorTerms[i].q_p) / dt;
          }
        }

        // if we control finger-joint 1 -> hold in zero position
        else
        {
          state = this->GetObjectDetection(this->joints[i - 2], i - 2, this->jointCommands.position[i - 2], prev_jointCommands[i - 2]);

          if (underactuating[i - 2] && (this->jointStates.position[i] < 0.05) && (this->jointCommands.position[i - 2] < (this->jointStates.position[i - 2] + this->jointStates.position[i])))
          {
            underactuating[i - 2] = false;
          }
          if ((state == 2) && (tmp_jointCommands[i] == this->jointCommands.position[i]))
          {
            underactuating[i - 2] = true;
          }

          if (underactuating[i - 2])
          {
            // ROS_ERROR("STATE: %d WasMoving: %d Command: %f PrevCommand: %f", state, wasMoving[i-2],this->jointCommands.position[i-2], prev_jointCommands[i-2]);
            q_p = this->jointCommands.position[i - 2] - (this->jointStates.position[i - 2] + this->jointStates.position[i]);
          }
          else
          {
            q_p = -position * 10;
          }

          if (!ignition::math::equal(dt, 0.0))
          {
            this->errorTerms[i].d_q_p_dt = (q_p - this->errorTerms[i].q_p) / dt;
          }
        }

        this->errorTerms[i].q_p = q_p;

        this->errorTerms[i].qd_p = this->jointCommands.velocity[i] - velocity;

        this->errorTerms[i].q_i = ignition::math::clamp(this->errorTerms[i].q_i + dt * this->errorTerms[i].q_p, static_cast<double>(this->jointCommands.i_effort_min[i]), static_cast<double>(this->jointCommands.i_effort_max[i]));

        // use gain params to compute force cmd
        force = this->jointCommands.kp_position[i] * this->errorTerms[i].q_p + this->jointCommands.kp_velocity[i] * this->errorTerms[i].qd_p + this->jointCommands.ki_position[i] * this->errorTerms[i].q_i +
                this->jointCommands.kd_position[i] * this->errorTerms[i].d_q_p_dt + this->jointCommands.effort[i];
      }
      // ROS_ERROR("%s ; Velocity %f ; CommandPosition %f ; JointPosition %f ; force %f", this->jointCommands.name[i].c_str() , this->joints[i]->GetVelocity(0), this->jointCommands.position[i] , this->jointStates.position[i], force);
      // ROS_ERROR("%s ; q_p %f ; qd_p %f ; q_i %f d_q_p_dt %f dt %f ; force %f", this->jointCommands.name[i].c_str(), this->errorTerms[i].q_p, this->errorTerms[i].qd_p, this->errorTerms[i].q_i, this->errorTerms[i].d_q_p_dt, dt, force);

      this->joints[i]->SetForce(0, force);
      this->lastControllerUpdateTime = curTime;

      if (tmp_jointCommands[i] != this->jointCommands.position[i])
      {
        if (tmp_jointCommands[i] != prev_jointCommands[i])
        {
          prev_jointCommands[i] = tmp_jointCommands[i];
        }
        tmp_jointCommands[i] = this->jointCommands.position[i];
      }
    }
  }
}

////////////////////////////////////////////////////////////////////////////////

void THOR_VT_HandPlugin::RosQueueThread()
{
  static const double timeout = 0.01;

  while (this->rosNode->ok())
  {
    this->rosQueue.callAvailable(ros::WallDuration(timeout));
  }
}

////////////////////////////////////////////////////////////////////////////////

// helper function to save some typing
void THOR_VT_HandPlugin::CopyVectorIfValid(const std::vector<double>& from, std::vector<double>& to)
{
  if (!from.size() || from.size() != to.size())
    return;
  for (size_t i = 0; i < from.size(); i++)
    to[i] = from[i];
}

////////////////////////////////////////////////////////////////////////////////
// Set Joint Commands
void THOR_VT_HandPlugin::SetJointCommands(const vigir_grasp_msgs::JointCommands::ConstPtr& _msg)
{
  boost::mutex::scoped_lock lock(this->mutex);
  // this implementation does not check the ordering of the joints. they must
  // agree with the structure initialized above!
  CopyVectorIfValid(_msg->position, this->jointCommands.position);
  CopyVectorIfValid(_msg->velocity, this->jointCommands.velocity);
  CopyVectorIfValid(_msg->effort, this->jointCommands.effort);
  CopyVectorIfValid(_msg->kp_position, this->jointCommands.kp_position);
  CopyVectorIfValid(_msg->ki_position, this->jointCommands.ki_position);
  CopyVectorIfValid(_msg->kd_position, this->jointCommands.kd_position);
  CopyVectorIfValid(_msg->kp_velocity, this->jointCommands.kp_velocity);
  CopyVectorIfValid(_msg->i_effort_min, this->jointCommands.i_effort_min);
  CopyVectorIfValid(_msg->i_effort_max, this->jointCommands.i_effort_max);
}
}  // namespace gazebo
