#include <algorithm>
#include <fstream>
#include <sstream>

#include <dynamic_reconfigure/Reconfigure.h>
#include <openssl/sha.h>
#include <ros/master.h>

#include <gazebo/common/Assert.hh>
#include <gazebo/common/Events.hh>

#include "adhoc/AdHocNetPlugin.hh"

using namespace gazebo;

GZ_REGISTER_WORLD_PLUGIN(AdHocNetPlugin)

/////////////////////////////////////////////////
void AdHocNetPlugin::Load(physics::WorldPtr _world, sdf::ElementPtr /*_sdf*/)
{
  gzmsg << "Starting Ad Hoc Net server" << std::endl;

  GZ_ASSERT(_world, "AdHocNetPlugin world pointer is NULL");
  this->world = _world;

  this->n.param("robots", this->robotList, this->robotList);

  std::vector<std::string> tempVec;
  this->n.param("setting_list", tempVec, tempVec);
  for (auto item: tempVec)
  {
    gzmsg << "Setting: " << item << std::endl;
    this->settingList.push(item);
  }

  this->node = transport::NodePtr(new transport::Node());
  this->node->Init();

  this->simCmdPub = this->node->Advertise<adhoc::msgs::SimInfo>("/sim_cmd");
  this->simCmdResSub = this->node->Subscribe<adhoc::msgs::SimInfo>(
    "/sim_cmd_res", &AdHocNetPlugin::OnSimCmdResponse, this);

  for (auto robot: this->robotList)
  {
    this->pubMap[robot]
      = this->node->Advertise<adhoc::msgs::Datagram>(
        robot + "/comm_in");
    this->subMap[robot]
      = this->node->Subscribe<adhoc::msgs::Datagram>(
        robot + "/comm_out", &AdHocNetPlugin::OnMessage, this);
  }

  this->updateConnection = event::Events::ConnectWorldUpdateBegin(
    std::bind(&AdHocNetPlugin::OnUpdate, this));

  this->robotsReadyToComm = false;

  this->startFlyingPub
    = this->n.advertise<std_msgs::Bool>("/start_flying", 1, true);

  this->navVelUpdatePub
    = this->n.advertise<std_msgs::Float32>("/nav_vel_update", 1);

  this->robotCheckThread
    = std::thread(&AdHocNetPlugin::CheckRobotsReadyTh, this);
}

/////////////////////////////////////////////////
void AdHocNetPlugin::OnUpdate()
{
  std::lock_guard<std::mutex> lk(this->simInfoMutex);

  if (!this->robotsReadyToComm)
  {
    bool flag = true;
    for (std::string &robot: this->robotList)
    {
      auto model = this->world->ModelByName(robot);
      if (!model || model->WorldPose().POS_Z < 4.9)
        flag = false;
    }
    if (flag)
    {
      this->robotsReadyToComm = true;

      this->StartNewTrial();
    }
  }
  else
  {
    if (this->started && !this->finished)
    {
      auto current = this->world->SimTime();

      if (current.Double() - this->lastStatPrintTime.Double() >= 10.0)
      {
        // gzmsg << "===== locations =====" << std::endl;
        // for (auto model : this->world->GetModels())
        // {
        //   auto pose = model->WorldPose();
        //   gzmsg << model->GetName() << ": " << pose << std::endl;
        // }
        // gzmsg << "=====================" << std::endl;
        this->lastStatPrintTime = current;
      }

      if (this->stopping)
      {
        std::lock_guard<std::mutex> lk(this->messageMutex);

        // check if no more message is not coming.
        if (current.Double() - this->lastRecvTime.Double()
              > this->currentDelayTime + 1.0)
        {
          gzmsg << "*** Simulation period passed. Stopping ***" << std::endl;
          this->listStopResponses.clear();
          adhoc::msgs::SimInfo start;
          start.set_state("stop");
          start.set_robot_name("");
          this->simCmdPub->Publish(start);
          this->finished = true;

          // Set the robot's speed to 0.
          std_msgs::Float32 vel;
          vel.data = 0;
          this->navVelUpdatePub.publish(vel);
        }
      }
      else if(current.Double() - this->startTime.Double()
        >= this->simPeriod)
      {
        gzmsg << "*** All communication done. Stopped. ***" << std::endl;
        this->listStopResponses.clear();
        adhoc::msgs::SimInfo start;
        start.set_state("stop_sending");
        start.set_robot_name("");
        this->simCmdPub->Publish(start);
        this->stopping = true;
      }

      this->topoChangeCount += this->CheckTopoChange();
    }
  }
}

//////////////////////////////////////////////////
void AdHocNetPlugin::CheckRobotsReadyTh()
{
  while (ros::ok())
  {
    ros::master::V_TopicInfo master_topics;
    ros::master::getTopics(master_topics);

    bool allReady = true;
    for (auto name: this->robotList)
    {
      int counts = 0;
      for (auto it = master_topics.begin(); it != master_topics.end(); it++)
      {
        const ros::master::TopicInfo& info = *it;
        if (
          info.name == "/" + name + "/controller/velocity/x/state" ||
          info.name == "/" + name + "/controller/velocity/y/state" ||
          info.name == "/" + name + "/controller/velocity/z/state" ||
          info.name == "/" + name + "/controller/attitude/roll/state" ||
          info.name == "/" + name + "/controller/attitude/pitch/state" ||
          info.name == "/" + name + "/controller/attitude/yawrate/state")
        {
          counts++;
        }
      }
      if (counts < 6)
      {
        allReady = false;
      }
    }
    if (allReady)
    {
      gzmsg << "ALL READY!" << std::endl;
      for (auto robotName: this->robotList)
      {
        this->initPoseList[robotName]
          = this->world->ModelByName(robotName)->WorldPose();
      }
      break;
    }
    ros::Duration(0.1).sleep();
  }

  if (ros::ok())
  {
    ros::Duration(2).sleep();
    std_msgs::Bool start;
    start.data = true;
    this->startFlyingPub.publish(start);
  }
}

/////////////////////////////////////////////////
void AdHocNetPlugin::OnMessage(
  const boost::shared_ptr<adhoc::msgs::Datagram const> &_req)
{
  // Once a packet is received, it is instantaneously processed.
  std::lock_guard<std::mutex> lk(this->messageMutex);
  this->lastRecvTime = this->world->SimTime();

  physics::ModelPtr sender = this->world->ModelByName(_req->robot_name());
  this->totalSentPackets++;

  if (sender)
  {
    for (int i = 1; i <= 10; ++i)
    {
      std::string name = "robot" + std::to_string(i);

      if (name == sender->GetName())
        continue;

      physics::ModelPtr robot = this->world->ModelByName(name);

      if (robot)
      {
        // forward the message if the robot is within the range of the sender.
        auto diffVec
          = robot->WorldPose().CoordPositionSub(sender->WorldPose());
        double length = fabs(diffVec.Length());

        if (length <= this->commRange)
        {
          this->pubMap[robot->GetName()]->Publish(*_req);
          unsigned char hash[SHA256_DIGEST_LENGTH];
          this->CalcHash(*_req, hash);
          if (!this->HasHash(hash))
            this->RegistHash(hash);
          this->totalRecvPackets++;
        }
      }
    }
  }
}

//////////////////////////////////////////////////
void AdHocNetPlugin::OnSimCmdResponse(
  const boost::shared_ptr<adhoc::msgs::SimInfo const> &_res)
{
  std::lock_guard<std::mutex> lk(this->simInfoMutex);

  if (_res->state() == "started")
  {
    gzmsg << "response (started) from " << _res->robot_name() << std::endl;

    if (std::find(
        this->listStartResponses.begin(), this->listStartResponses.end(), _res)
          == this->listStartResponses.end())
      this->listStartResponses.push_back(_res);

    // if all started.
    if (this->listStartResponses.size() == this->robotList.size())
    {
      this->CheckTopoChange();
      this->startTime = this->world->SimTime();
      this->lastStatPrintTime = this->world->SimTime();
      this->started = true;

      // start recording
      gzmsg << "Network started" << std::endl;
    }
  }
  else if (_res->state() == "stopped")
  {
    gzmsg << "response (stopped) from " << _res->robot_name() << std::endl;

    if (std::find(
        this->listStopResponses.begin(), this->listStopResponses.end(), _res)
          == this->listStopResponses.end())
      this->listStopResponses.push_back(_res);

    // if all are done.
    if (this->listStopResponses.size() == this->robotList.size())
    {
      common::Time current = this->world->SimTime();
      double elapsed = current.Double() - this->startTime.Double();

      int sentMessageCount = 0;
      int recvMessageCount = 0;
      int totalHops = 0;
      double totalRoundTripTime = 0;

      double totalDistComm = 0;
      double totalDistMotion = 0;

      for (auto res: this->listStopResponses)
      {
        sentMessageCount += res->sent_message_count();
        recvMessageCount += res->recv_message_count();
        totalHops += res->total_hops();
        totalRoundTripTime += res->total_round_trip_time();

        totalDistComm += res->total_dist_comm();
        totalDistMotion += res->total_dist_motion();
      }

      // finish recording
      std::stringstream ss;
      ss << "--- Network ---" << std::endl;
      ss << "Time," << elapsed << std::endl;
      ss << "Total # of Sent Packets," << this->totalSentPackets << std::endl;
      ss << "Total # of Recv Packets," << this->totalRecvPackets << std::endl;
      ss << "Total # of Message," << this->hashSet.size() << std::endl;
      ss << "Avg # of Packets per Sent Message,"
         << ((double)this->totalSentPackets)/this->hashSet.size() << std::endl;
      ss << "Avg # of Packets per Recv Message,"
         << ((double)this->totalRecvPackets)/this->hashSet.size() << std::endl;
      ss << "Total # of Topology Changes,"
         << this->topoChangeCount << std::endl;
      ss << "Frequency of Topology Change,"
         << this->topoChangeCount / elapsed << std::endl;

      ss << "--- Client ---" << std::endl;
      ss << "Robot Speed," << this->currentRobotSpeed << std::endl;
      ss << "Time of hop to delay," << this->currentDelayTime << std::endl;
      ss << "Total # of Sent Messages," << sentMessageCount << std::endl;
      ss << "Total # of Received Messages," << recvMessageCount << std::endl;
      ss << "Total # of Hops," << totalHops << std::endl;
      ss << "Avg # of hops per Message,"
         << ((double)totalHops)/recvMessageCount << std::endl;
      ss << "Total Round Trip Time," << totalRoundTripTime << std::endl;
      ss << "Avg Round Trip Time per Message,"
         << totalRoundTripTime/recvMessageCount << std::endl;
      ss << "Total Distance of Communication," << totalDistComm << std::endl;
      ss << "Avg Distance of Communicaiton Taken by a Packet,"
         << totalDistComm/recvMessageCount << std::endl;
      ss << "Total Distance of Motion," << totalDistMotion << std::endl;
      ss << "Avg Distance of Motion Taken by a Packet,"
         << totalDistMotion/recvMessageCount << std::endl;

      gzmsg << ss.str();

      std::stringstream filename;
      filename << "Simulation_d" << this->currentDelayTime
               << "_s" << this->currentRobotSpeed
               << "_" << current.FormattedString() << ".csv";
      std::fstream fs;
      fs.open(filename.str(), std::fstream::out);
      fs << ss.str();
      fs.close();

      // Reset the robot's pose.
      for (auto robotName: this->robotList)
      {
        auto model = this->world->ModelByName(robotName);
        for (auto link: model->GetLinks())
        {
          link->ResetPhysicsStates();
        }
        model->SetWorldPose(this->initPoseList[robotName]);
        //model->Reset();
      }
      // Once the flag is set to false, the plugin starts to check the robot's
      // pose and start the next simulation trial when they reach the specific
      // altitude.
      this->robotsReadyToComm = false;
    }
  }
}

//////////////////////////////////////////////////
void AdHocNetPlugin::CalcHash(
  const adhoc::msgs::Datagram &_msg, unsigned char *_hash)
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

  if (this->hashSet.count(buff) != 0)
      return true;
  else
      return false;
}

//////////////////////////////////////////////////
void AdHocNetPlugin::RegistHash(const unsigned char *_hash)
{
  std::string str;
  str.assign((const char*)_hash, SHA256_DIGEST_LENGTH);
  this->hashSet.insert(str);
}

//////////////////////////////////////////////////
int AdHocNetPlugin::CheckTopoChange()
{
  int count = 0;
  for (int i = 1; i <= 9; ++i)
  {
    for (int j = i + 1; j <= 10; ++j)
    {
      physics::ModelPtr robot1
        = this->world->ModelByName("robot" + std::to_string(i));
      physics::ModelPtr robot2
        = this->world->ModelByName("robot" + std::to_string(j));

      if (!robot1 || !robot2)
        continue;

      auto diffVec
        = robot1->WorldPose().CoordPositionSub(robot2->WorldPose());
      double length = fabs(diffVec.Length());

      bool inRange = (length <= this->commRange);
      if (inRange != this->topoList[std::to_string(i)+":"+std::to_string(j)])
      {
        count++;
        this->topoList[std::to_string(i)+":"+std::to_string(j)] = inRange;
        //if (inRange)
        //  gzdbg << i << ":" << j << ", connected" << std::endl;
        //else
        //  gzdbg << i << ":" << j << ", disconnected" << std::endl;
      }
    }
  }

  return count;
}

//////////////////////////////////////////////////
void AdHocNetPlugin::StartNewTrial()
{
  if (this->settingList.size() > 0)
  {
    gzmsg << "StartNewTrial" << std::endl;
    std::string settingName = this->settingList.front();
    this->settingList.pop();

    gzmsg << "getting setting: " << settingName << std::endl;
    std::map<std::string, double> settingMap;
    this->n.param(settingName, settingMap, settingMap);

    this->started = false;
    this->stopping = false;
    this->finished = false;

    this->totalSentPackets = 0;
    this->totalRecvPackets = 0;
    this->topoChangeCount = 0;

    gzmsg << "Net: clearing hashSet" << std::endl;
    this->hashSet.clear();
    gzmsg << "Net: done" << std::endl;

    this->n.getParam("simulation_period", this->simPeriod);
    this->n.getParam("communication_range", this->commRange);

    if (settingMap.count("simulation_period") > 0)
      this->simPeriod = settingMap["simulation_period"];
    if (settingMap.count("communication_range") > 0)
      this->commRange = settingMap["communication_range"];

    gzmsg << "publishing new velocity" << std::endl;

    std_msgs::Float32 vel;
    this->currentRobotSpeed = settingMap["robot_speed"];
    vel.data = this->currentRobotSpeed;
    this->navVelUpdatePub.publish(vel);

    gzmsg << std::endl
          << "===================================================" << std::endl
          << "===================================================" << std::endl
          << "Trial: " << settingName << std::endl
          << "simulation period: " << this->simPeriod << std::endl
          << "communicaiton range: " << this->commRange << std::endl
          << "robot speed: " << vel.data << std::endl
          << "===================================================" << std::endl
          << "===================================================" << std::endl;

    for (int i = 1; i <= 9; ++i)
    {
      for (int j = i + 1; j <= 10; ++j)
      {
        this->topoList[std::to_string(i)+":"+std::to_string(j)] = false;
      }
    }

    this->currentDelayTime = settingMap["delay_time"];

    this->listStartResponses.clear();
    adhoc::msgs::SimInfo start;
    start.set_state("start");
    start.set_robot_name("");
    start.set_delay_time(this->currentDelayTime);
    this->simCmdPub->Publish(start);
  }
  else
  {
    gzmsg << "Simulation done: " << std::endl
          << this->world->SimTime().FormattedString() << " (Sim Time)"
          << std::endl
          << this->world->RealTime().FormattedString() << " (Real Time)"
          << std::endl;
  }
}
