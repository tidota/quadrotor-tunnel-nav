#include <cstdlib>
#include <cstring>
#include <ctime>
#include <fstream>
#include <sstream>
#include <unordered_set>

#include <openssl/sha.h>

#include "adhoc/AdHocClientPlugin.hh"

using namespace gazebo;

GZ_REGISTER_MODEL_PLUGIN(AdHocClientPlugin)

//////////////////////////////////////////////////
void AdHocClientPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr /*_sdf*/)
{
  this->model = _model;

  // TODO: Get specific address from rosparam
  // assuming the model name has a number as suffix.
  std::istringstream(this->model->GetName().substr(5)) >> this->id;

  gzmsg << "Starting Ad Hoc Net client " << this->id
        << " for " << this->model->GetScopedName() << std::endl;

  this->running = false;
  this->stopping = false;
  this->delayTime = 0;

  this->node = transport::NodePtr(new transport::Node());
  this->node->Init();

  this->simCommSub
    = this->node->Subscribe<adhoc::msgs::SimInfo>(
      "/sim_cmd", &AdHocClientPlugin::OnSimCmd, this);

  this->simCommResPub
    = this->node->Advertise<adhoc::msgs::SimInfo>(
      "/sim_cmd_res");

  this->pub = this->node->Advertise<adhoc::msgs::Datagram>(
    this->model->GetName() + "/comm_out");
  this->sub = this->node->Subscribe<adhoc::msgs::Datagram>(
    this->model->GetName() + "/comm_in",
    &AdHocClientPlugin::OnMessage, this);

  this->sentMessageCount = 0;

  this->msg_req.set_robot_name(this->model->GetName());
  this->msg_req.set_src_address(this->id);
  this->msg_req.set_hops(0);
  this->msg_req.set_data("request");

  this->msg_res.set_robot_name(this->model->GetName());
  this->msg_res.set_src_address(this->id);
  this->msg_res.set_hops(0);
  this->msg_res.set_data("response");

  this->lastSentTime = this->model->GetWorld()->SimTime();

  this->updateConnection = event::Events::ConnectWorldUpdateBegin(
    std::bind(&AdHocClientPlugin::OnUpdate, this));

  this->recvMessageCount = 0;
  this->totalHops = 0;
  this->totalRoundTripTime = 0.0;

  this->totalDistComm = 0;
  this->totalDistMotion = 0;

  std::srand(std::time(nullptr));

  this->sendStoppedResponse = false;
  ros::NodeHandle n;
  this->cmdVelMonitorSub
    = n.subscribe(
      "/" + this->model->GetName() + "/cmd_vel", 1,
      &AdHocClientPlugin::OnCmdVel, this);
}

//////////////////////////////////////////////////
void AdHocClientPlugin::OnUpdate()
{
  std::lock_guard<std::mutex> lk1(this->simInfoMutex);
  std::lock_guard<std::mutex> lk2(this->messageMutex);

  // at some interval, initiate a communication.
  common::Time current = this->model->GetWorld()->SimTime();
  if (this->running && !this->stopping
    && current.Double() - this->lastSentTime.Double() > 0.5)
  {
    unsigned int dst = std::rand() % 9;
    if (dst >= this->id - 1)
      dst = (dst + 1) % 10;
    dst++;
    this->msg_req.set_dst_address(dst);
    this->msg_req.set_index(this->sentMessageCount);
    this->msg_req.set_hops(1);
    this->msg_req.set_time(current.Double());

    this->msg_req.set_dist_comm(0);
    this->msg_req.set_dist_motion(0);

    ignition::math::Pose3d currentPose = this->model->WorldPose();
    gazebo::msgs::Vector3d* prevLocMsg = this->msg_req.mutable_prev_loc();
    prevLocMsg->set_x(currentPose.pos.x);
    prevLocMsg->set_y(currentPose.pos.y);
    prevLocMsg->set_z(currentPose.pos.z);

    unsigned char hash[SHA256_DIGEST_LENGTH];
    this->CalcHash(this->msg_req, hash);
    this->RegistHash(hash);

    this->pub->Publish(this->msg_req);
    this->lastSentTime = current;
    this->sentMessageCount++;
  }

  ProcessincomingMsgsStamped();
}

//////////////////////////////////////////////////
void AdHocClientPlugin::OnMessage(
  const boost::shared_ptr<adhoc::msgs::Datagram const> &_msg)
{
  std::lock_guard<std::mutex> lk(this->messageMutex);

  adhoc::msgs::Datagram tempMsg(*_msg);
  // For tracking purposes.
  ignition::math::Pose3d currentPose = this->model->WorldPose();
  gazebo::msgs::Vector3d* prevLocMsg = tempMsg.mutable_prev_loc();
  double dx = currentPose.pos.x - prevLocMsg->x();
  double dy = currentPose.pos.y - prevLocMsg->y();
  double dz = currentPose.pos.z - prevLocMsg->z();
  double dist = std::sqrt(dx*dx + dy*dy + dz*dz);
  tempMsg.set_dist_comm(_msg->dist_comm() + dist);
  prevLocMsg->set_x(currentPose.pos.x);
  prevLocMsg->set_y(currentPose.pos.y);
  prevLocMsg->set_z(currentPose.pos.z);

  // Just save the message, it will be processed later.
  common::Time t = this->model->GetWorld()->SimTime();
  this->incomingMsgsStamped.push_back(
    std::pair<adhoc::msgs::Datagram, common::Time>(tempMsg, t));
}

//////////////////////////////////////////////////
void AdHocClientPlugin::OnSimCmd(
  const boost::shared_ptr<adhoc::msgs::SimInfo const> &_req)
{
  std::lock_guard<std::mutex> lk1(this->simInfoMutex);
  std::lock_guard<std::mutex> lk2(this->messageMutex);

  if (!this->running && _req->state() == "start")
  {
    this->running = true;
    this->stopping = false;
    this->delayTime = _req->delay_time();

    gzmsg << "Delay Time(" << this->model->GetName() << "): "
          << this->delayTime << std::endl;

    this->sentMessageCount = 0;
    this->recvMessageCount = 0;
    this->totalHops = 0;
    this->totalRoundTripTime = 0;
    this->totalDistComm = 0;
    this->totalDistMotion = 0;

    this->hashSet.clear();
    this->incomingMsgsStamped.clear();

    // start recording
    adhoc::msgs::SimInfo msg;
    msg.set_state("started");
    msg.set_robot_name(this->model->GetName());
    this->simCommResPub->Publish(msg);
  }
  else if (this->running && !this->stopping && _req->state() == "stop_sending")
  {
    this->stopping = true;
  }
  else if (this->running && this->stopping && _req->state() == "stop")
  {
    this->sendStoppedResponse = true;
    this->running = false;
    this->stopping = false;
  }
  else
  {
    if (this->running)
    {
      gzerr << "Invalid command received while running("
            << this->model->GetName()
            << "): "
            << _req->state() << std::endl;
    }
    else
    {
      gzerr << "Invalid command received while not running("
            << this->model->GetName()
            << "): "
            << _req->state() << std::endl;
    }
  }
}

//////////////////////////////////////////////////
void AdHocClientPlugin::OnCmdVel(
  const ros::MessageEvent<geometry_msgs::Twist const>& _event)
{
  auto cmd = _event.getMessage();

  if (this->sendStoppedResponse
    && cmd->linear.x == 0 && cmd->linear.y == 0 && cmd->linear.z == 0
    && cmd->angular.x == 0 && cmd->angular.y == 0 && cmd->angular.z == 0)
  {
    adhoc::msgs::SimInfo msg;
    msg.set_state("stopped");
    msg.set_robot_name(this->model->GetName());
    msg.set_delay_time(this->delayTime);
    msg.set_sent_message_count(this->sentMessageCount);
    msg.set_recv_message_count(this->recvMessageCount);
    msg.set_total_hops(this->totalHops);
    msg.set_total_round_trip_time(this->totalRoundTripTime);
    msg.set_total_dist_comm(this->totalDistComm);
    msg.set_total_dist_motion(this->totalDistMotion);
    this->simCommResPub->Publish(msg);
    this->sendStoppedResponse = false;
  }
}

/////////////////////////////////////////////////
void AdHocClientPlugin::ProcessincomingMsgsStamped()
{
  common::Time current = this->model->GetWorld()->SimTime();

  while (!this->incomingMsgsStamped.empty())
  {
    auto &p = this->incomingMsgsStamped.front();
    auto &t = p.second;
    if (current.Double() - t.Double() < this->delayTime)
      break;

    auto const &msg = p.first;

    unsigned char hash[SHA256_DIGEST_LENGTH];

    this->CalcHash(msg, hash);

    if (!this->HasHash(hash))
    {
      this->RegistHash(hash);

      if (this->id == msg.dst_address())
      {
        if (msg.data() == "request")
        {
          this->msg_res.set_dst_address(msg.src_address());
          this->msg_res.set_index(msg.index());
          this->msg_res.set_hops(msg.hops() + 1);
          this->msg_res.set_time(msg.time());

          // For tracking purposes.
          ignition::math::Pose3d currentPose = this->model->WorldPose();
          gazebo::msgs::Vector3d* prevLocMsg = this->msg_res.mutable_prev_loc();
          double dx = currentPose.pos.x - prevLocMsg->x();
          double dy = currentPose.pos.y - prevLocMsg->y();
          double dz = currentPose.pos.z - prevLocMsg->z();
          double dist = std::sqrt(dx*dx + dy*dy + dz*dz);
          this->msg_res.set_dist_comm(msg.dist_comm());
          this->msg_res.set_dist_motion(msg.dist_motion() + dist);
          prevLocMsg->set_x(currentPose.pos.x);
          prevLocMsg->set_y(currentPose.pos.y);
          prevLocMsg->set_z(currentPose.pos.z);

          unsigned char hash[SHA256_DIGEST_LENGTH];
          this->CalcHash(this->msg_res, hash);
          this->RegistHash(hash);

          this->pub->Publish(this->msg_res);
        }
        else if (msg.data() == "response")
        {
          common::Time current = this->model->GetWorld()->SimTime();
          this->recvMessageCount++;
          this->totalHops += msg.hops();
          this->totalRoundTripTime += current.Double() - msg.time();

          this->totalDistComm += msg.dist_comm();
          this->totalDistMotion += msg.dist_motion();
        }
        else
        {
          gzerr << this->model->GetName() << " got invalid data." << std::endl;
        }
      }
      else if (msg.hops() <= 20)
      {
        adhoc::msgs::Datagram forwardMsg(msg);
        forwardMsg.set_robot_name(this->model->GetName());
        forwardMsg.set_hops(msg.hops() + 1);

        // For tracking purposes.
        ignition::math::Pose3d currentPose = this->model->WorldPose();
        gazebo::msgs::Vector3d* prevLocMsg = forwardMsg.mutable_prev_loc();
        double dx = currentPose.pos.x - prevLocMsg->x();
        double dy = currentPose.pos.y - prevLocMsg->y();
        double dz = currentPose.pos.z - prevLocMsg->z();
        double dist = std::sqrt(dx*dx + dy*dy + dz*dz);
        forwardMsg.set_dist_comm(msg.dist_comm());
        forwardMsg.set_dist_motion(msg.dist_motion() + dist);
        prevLocMsg->set_x(currentPose.pos.x);
        prevLocMsg->set_y(currentPose.pos.y);
        prevLocMsg->set_z(currentPose.pos.z);

        this->pub->Publish(forwardMsg);
      }
    }

    this->incomingMsgsStamped.pop_front();
  }
}

//////////////////////////////////////////////////
void AdHocClientPlugin::CalcHash(
  const adhoc::msgs::Datagram &_msg, unsigned char *_hash)
{
  std::string input
    = std::to_string(_msg.src_address()) + std::to_string(_msg.dst_address())
    + std::to_string(_msg.index()) + _msg.data();

  SHA256((unsigned char*)input.c_str(), input.length(), _hash);
}

//////////////////////////////////////////////////
bool AdHocClientPlugin::HasHash(const unsigned char *_hash)
{
  std::string buff;
  buff.assign((const char*)_hash, SHA256_DIGEST_LENGTH);

  if (this->hashSet.count(buff) > 0)
      return true;
  else
      return false;
}

//////////////////////////////////////////////////
void AdHocClientPlugin::RegistHash(const unsigned char *_hash)
{
  std::string str;
  str.assign((const char*)_hash, SHA256_DIGEST_LENGTH);
  this->hashSet.insert(str);
}
