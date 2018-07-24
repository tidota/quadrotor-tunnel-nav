#include <sstream>

#include <gazebo/common/Assert.hh>
#include <gazebo/common/Events.hh>

#include <openssl/sha.h>

#include "adhoc/CommonTypes.hh"
#include "adhoc/AdHocNetPlugin.hh"
#include "quadrotor_tunnel_nav/protobuf/datagram.pb.h"

using namespace gazebo;

GZ_REGISTER_WORLD_PLUGIN(AdHocNetPlugin)

/////////////////////////////////////////////////
void AdHocNetPlugin::Load(physics::WorldPtr _world, sdf::ElementPtr _sdf)
{
  gzmsg << "Starting Ad Hoc Net server" << std::endl;

  if (_sdf->HasElement("range"))
    this->commRange = _sdf->Get<double>("range");
  else
    this->commRange = 10.0;

  gzmsg << "Communication range: " << this->commRange << std::endl;

  GZ_ASSERT(_world, "AdHocNetPlugin world pointer is NULL");
  this->world = _world;

  this->started = true;
  this->finished = false;
  if (_sdf->HasElement("enable"))
  {
    this->started = _sdf->Get<bool>("enable");
  }

  this->enableSub
    = this->n.subscribe(
        "/start_comm", 1, &AdHocNetPlugin::OnStartStopMessage, this);


  this->node = transport::NodePtr(new transport::Node());
  this->node->Init();
  for (int i = 1; i <= 10; ++i)
  {
    this->pubMap["robot" + std::to_string(i)]
      = this->node->Advertise<adhoc::msgs::Datagram>(
        "robot" + std::to_string(i) + "/comm_in");
    this->subMap["robot" + std::to_string(i)]
      = this->node->Subscribe<adhoc::msgs::Datagram>(
        "robot" + std::to_string(i) + "/comm_out",
        &AdHocNetPlugin::OnMessage, this);
  }
  this->clientOutputSub
    = this->node->Subscribe<gazebo::msgs::GzString>(
      "/print_by_net",
      &AdHocNetPlugin::OnClientMessage, this);

  this->updateConnection = event::Events::ConnectWorldUpdateBegin(
    std::bind(&AdHocNetPlugin::OnUpdate, this));

  this->lastDisplayed = this->world->GetSimTime();

  this->totalPackets = 0;

  this->topoChangeCount = 0;
  this->InitTopoList();


  gzmsg << "Net init done" << std::endl;
}

//////////////////////////////////////////////////
void AdHocNetPlugin::OnStartStopMessage(const ros::MessageEvent<std_msgs::Bool const>& event)
{
  const std_msgs::Bool::ConstPtr& flag = event.getMessage();

  std::lock_guard<std::mutex> lk(this->mutexStartStop);
  if (!this->started && !this->finished && flag->data)
  {
    this->started = true;
    this->InitTopoList();
    this->CheckTopoChange();
    this->startTime = this->world->GetSimTime();

    // start recording
    gzmsg << "Network started" << std::endl;
  }
  else if (this->started && !this->finished && !flag->data)
  {
    this->finished = true;

    common::Time current = this->world->GetSimTime();
    double elapsed = current.Double() - this->startTime.Double();

    // finish recording
    std::stringstream ss;
    ss << "--- Network ---" << std::endl;
    ss << "Time: " << elapsed << std::endl;
    ss << "Total # of Packets: " << this->totalPackets << std::endl;
    ss << "Total # of Message: " << this->hashList.size() << std::endl;
    ss << "Ave # of Packets per Message: " << ((double)this->totalPackets)/this->hashList.size() << std::endl;
    ss << "Total # of Topology Changes: " << this->topoChangeCount << std::endl;
    ss << "Frequency of Topology Change: " << this->topoChangeCount / elapsed << std::endl;
    gzmsg << ss.str();
  }
}

/////////////////////////////////////////////////
void AdHocNetPlugin::OnUpdate()
{
  {
    std::lock_guard<std::mutex> lk(this->mutexStartStop);
    auto current = this->world->GetSimTime();
    if (current.Double() - this->lastDisplayed.Double() > 3.0)
    {
      gzmsg << "===== locations =====" << std::endl;
      for (auto model : this->world->GetModels())
      {
        auto pose = model->GetWorldPose();
        gzmsg << model->GetName() << ": " << pose << std::endl;
      }
      gzmsg << "=====================" << std::endl;
      this->lastDisplayed = current;
    }
  }

  if (this->started && !this->finished && this->CheckTopoChange())
  {
    this->topoChangeCount++;
  }

  std::lock_guard<std::mutex> lk(this->mutex);
  this->ProcessIncomingMsgs();
}

/////////////////////////////////////////////////
void AdHocNetPlugin::ProcessIncomingMsgs()
{
  while (!this->incomingMsgs.empty())
  {
    // Forward the messages.
    auto const &msg = this->incomingMsgs.front();

    physics::ModelPtr sender = this->world->GetModel(msg.model_name());

    if (sender)
    {
      for (int i = 1; i <= 10; ++i)
      {
        std::string name = "robot" + std::to_string(i);

        if (name == sender->GetName())
          continue;

        physics::ModelPtr robot = this->world->GetModel(name);

        if (robot)
        {
          // forward the message if the robot is within the range of the sender.
          auto diffVec = robot->GetWorldPose().CoordPositionSub(sender->GetWorldPose());
          double length = fabs(diffVec.GetLength());

          if (length <= this->commRange)
          {
            this->pubMap[robot->GetName()]->Publish(msg);
            unsigned char hash[SHA256_DIGEST_LENGTH];
            this->CalcHash(msg, hash);
            if (!this->HasHash(hash))
              this->RegistHash(hash);
            this->totalPackets++;
          }
        }
      }
    }

    this->incomingMsgs.pop();
  }
}

/////////////////////////////////////////////////
void AdHocNetPlugin::OnMessage(const boost::shared_ptr<adhoc::msgs::Datagram const> &_req)
{
  // Just save the message, it will be processed later.
  std::lock_guard<std::mutex> lk(this->mutex);
  this->incomingMsgs.push(*_req);
}

/////////////////////////////////////////////////
void AdHocNetPlugin::OnClientMessage(const boost::shared_ptr<gazebo::msgs::GzString const> &_data)
{
  std::lock_guard<std::mutex> lk(this->mutexStartStop);
  gzmsg << _data->data() << std::endl;
}

//////////////////////////////////////////////////
void AdHocNetPlugin::CalcHash(const adhoc::msgs::Datagram &_msg, unsigned char *_hash)
{
  std::string input
    = std::to_string(_msg.src_address()) + std::to_string(_msg.dst_address())
    + std::to_string(_msg.index()) + _msg.data();

  SHA256((unsigned char*)input.c_str(), input.length(), _hash);
}

//////////////////////////////////////////////////
bool AdHocNetPlugin::HasHash(const unsigned char *_hash)
{
  std::string buff;
  buff.assign((const char*)_hash, SHA256_DIGEST_LENGTH);
  for (auto h : this->hashList)
  {
    if (h == buff)
      return true;
  }
  return false;
}

//////////////////////////////////////////////////
void AdHocNetPlugin::RegistHash(const unsigned char *_hash)
{
  std::string str;
  str.assign((const char*)_hash, SHA256_DIGEST_LENGTH);
  this->hashList.push_back(str);
}

//////////////////////////////////////////////////
void AdHocNetPlugin::InitTopoList()
{
  for (int i = 1; i <= 9; ++i)
  {
    for (int j = i + 1; j <= 10; ++j)
    {
      this->topoList[std::to_string(i)+":"+std::to_string(j)] = false;
    }
  }
}

//////////////////////////////////////////////////
bool AdHocNetPlugin::CheckTopoChange()
{
  bool changed = false;
  for (int i = 1; i <= 9; ++i)
  {
    for (int j = i + 1; j <= 10; ++j)
    {
      physics::ModelPtr robot1 = this->world->GetModel("robot" + std::to_string(i));
      physics::ModelPtr robot2 = this->world->GetModel("robot" + std::to_string(j));

      if (!robot1 || !robot2)
        continue;

      auto diffVec = robot1->GetWorldPose().CoordPositionSub(robot2->GetWorldPose());
      double length = fabs(diffVec.GetLength());

      bool inRange = (length <= this->commRange);
      if (inRange != this->topoList[std::to_string(i)+":"+std::to_string(j)])
      {
        changed = true;
        this->topoList[std::to_string(i)+":"+std::to_string(j)] = inRange;
        if (inRange)
          gzmsg << i << ":" << j << ", connected" << std::endl;
        else
          gzmsg << i << ":" << j << ", disconnected" << std::endl;
      }
    }
  }

  return changed;
}
