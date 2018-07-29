/*
 * Copyright (C) 2018 Open Source Robotics Foundation
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

#include <cstdlib>
#include <cstring>
#include <ctime>
#include <fstream>
#include <sstream>

#include <openssl/sha.h>

#include "adhoc/AdHocClientPlugin.hh"
#include "adhoc/CommonTypes.hh"
#include "quadrotor_tunnel_nav/protobuf/datagram.pb.h"

using namespace gazebo;

GZ_REGISTER_MODEL_PLUGIN(AdHocClientPlugin)

//////////////////////////////////////////////////
void AdHocClientPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  this->model = _model;
  // assuming the model name has a number as suffix.
  std::istringstream(this->model->GetName().substr(5)) >> this->id;

  gzmsg << "Starting Ad Hoc Net client " << this->id
        << " for " << this->model->GetScopedName() << std::endl;

  this->started = true;
  this->finished = false;
  if (_sdf->HasElement("enable"))
  {
    this->started = _sdf->Get<bool>("enable");
  }
  this->delayedTime = 0;
  if (_sdf->HasElement("delay"))
  {
    this->delayedTime = _sdf->Get<double>("delay");
  }
  gzmsg << "Delayed Time: " << this->delayedTime << std::endl;

  this->enableSub
    = this->n.subscribe(
        "/start_comm", 1, &AdHocClientPlugin::OnStartStopMessage, this);


  this->node = transport::NodePtr(new transport::Node());
  this->node->Init();
  this->pub = this->node->Advertise<adhoc::msgs::Datagram>(this->model->GetName() + "/comm_out");
  this->sub = this->node->Subscribe<adhoc::msgs::Datagram>(this->model->GetName() + "/comm_in", &AdHocClientPlugin::OnNetworkMessage, this);

  this->clientOutputPub
    = this->node->Advertise<gazebo::msgs::GzString>(
      "/print_by_net");

  this->messageCount = 0;

  this->msg_req.set_model_name(this->model->GetName());
  this->msg_req.set_src_address(this->id);
  this->msg_req.set_hops(0);
  this->msg_req.set_data("request");

  this->msg_res.set_model_name(this->model->GetName());
  this->msg_res.set_src_address(this->id);
  this->msg_res.set_hops(0);
  this->msg_res.set_data("response");

  this->lastSent = this->model->GetWorld()->GetSimTime();

  this->updateConnection = event::Events::ConnectWorldUpdateBegin(
    std::bind(&AdHocClientPlugin::OnUpdate, this));

  this->totalMessages = 0;
  this->totalHops = 0;
  this->totalRoundTripTime = 0.0;

  std::srand(std::time(nullptr));
}

//////////////////////////////////////////////////
void AdHocClientPlugin::OnStartStopMessage(const ros::MessageEvent<std_msgs::Bool const>& event)
{
  const std_msgs::Bool::ConstPtr& flag = event.getMessage();

  std::lock_guard<std::mutex> lk(this->mutexStartStop);
  if (!this->started && !this->finished && flag->data)
  {
    this->started = true;

    if (this->id == 1)
    {
      // start recording
      msgs::GzString msg;
      msg.set_data("Client started!");
      this->clientOutputPub->Publish(msg);
    }
  }
  else if (this->started && !this->finished && !flag->data)
  {
    this->finished = true;

    if (this->id == 1)
    {
      // finish recording
      msgs::GzString msg;
      std::stringstream ss;
      ss << "--- Client ---" << std::endl;
      ss << "Time of hop to delay: " << this->delayedTime << std::endl;
      ss << "Total # of Sent Messages: " << this->messageCount << std::endl;
      ss << "Total # of Received Messages: " << this->totalMessages << std::endl;
      ss << "Total # of Hops: " << this->totalHops << std::endl;
      ss << "Ave # of hops per Message: " << ((double)this->totalHops)/this->totalMessages << std::endl;
      ss << "Total Round Trip Time: " << this->totalRoundTripTime << std::endl;
      ss << "Ave Round Trip Time per Message: " << this->totalRoundTripTime/this->totalMessages << std::endl;
      msg.set_data(ss.str());
      this->clientOutputPub->Publish(msg);

      common::Time current = this->model->GetWorld()->GetSimTime();
      std::fstream fs;
      fs.open("Client-" + current.FormattedString() + ".log", std::fstream::out);
      fs << ss.str();
      fs.close();
    }
  }
}

//////////////////////////////////////////////////
void AdHocClientPlugin::OnUpdate()
{
  std::lock_guard<std::mutex> lk(this->mutex);
  // at some interval, initiate a communication.
  {
    common::Time current = this->model->GetWorld()->GetSimTime();
    std::lock_guard<std::mutex> lk(this->mutexStartStop);
    if (this->started && !this->finished
      && current.Double() - this->lastSent.Double() > 0.5)
    {
      unsigned int dst = std::rand() % 9;
      if (dst >= this->id - 1)
        dst = (dst + 1) % 10;
      dst++;
      this->msg_req.set_dst_address(dst);
      this->msg_req.set_index(this->messageCount);
      this->msg_req.set_hops(1);
      this->msg_req.set_time(current.Double());
      this->pub->Publish(this->msg_req);
      this->lastSent = current;
      this->messageCount++;
    }
  }

  ProcessIncomingMsgs();
}

/////////////////////////////////////////////////
void AdHocClientPlugin::ProcessIncomingMsgs()
{
  common::Time current = this->model->GetWorld()->GetSimTime();

  while (!this->incomingMsgs.empty())
  {
    auto &p = this->incomingMsgs.front();
    auto &t = p.second;
    if (current.Double() - t.Double() < this->delayedTime)
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
          this->pub->Publish(this->msg_res);
        }
        else if (msg.data() == "response")
        {
          common::Time current = this->model->GetWorld()->GetSimTime();
          msgs::GzString m;
          std::string str = this->model->GetName();
          str = str + ": response from src " + std::to_string(msg.src_address())
            + " (" + std::to_string(msg.hops()) + " hops and "
            + std::to_string(current.Double() - msg.time()) + " sec in total)";
          m.set_data(str);
          this->clientOutputPub->Publish(m);
          this->totalMessages++;
          this->totalHops += msg.hops();
          this->totalRoundTripTime += current.Double() - msg.time();
        }
        else
        {
          gzerr << this->model->GetName() << " got invalid data." << std::endl;
        }
      }
      else if (msg.hops() <= 20)
      {
        adhoc::msgs::Datagram forwardMsg(msg);
        forwardMsg.set_model_name(this->model->GetName());
        forwardMsg.set_hops(msg.hops() + 1);
        this->pub->Publish(forwardMsg);
      }
    }

    this->incomingMsgs.pop();
  }
}

//////////////////////////////////////////////////
void AdHocClientPlugin::OnNetworkMessage(const boost::shared_ptr<adhoc::msgs::Datagram const> &_msg)
{
  // Just save the message, it will be processed later.
  std::lock_guard<std::mutex> lk(this->mutex);
  std::pair<adhoc::msgs::Datagram, common::Time> p;
  common::Time t = this->model->GetWorld()->GetSimTime();
  p.first = *_msg;
  p.second = t;
  this->incomingMsgs.push(p);
}

//////////////////////////////////////////////////
void AdHocClientPlugin::CalcHash(const adhoc::msgs::Datagram &_msg, unsigned char *_hash)
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
  for (auto h : this->hashList)
  {
    if (h == buff)
      return true;
  }
  return false;
}

//////////////////////////////////////////////////
void AdHocClientPlugin::RegistHash(const unsigned char *_hash)
{
  std::string str;
  str.assign((const char*)_hash, SHA256_DIGEST_LENGTH);
  this->hashList.push_back(str);
}
