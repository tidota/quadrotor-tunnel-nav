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

#include <iostream>
#include <sstream>

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

  this->node = transport::NodePtr(new transport::Node());
  this->node->Init();
  this->pub = this->node->Advertise<adhoc::msgs::Datagram>(this->model->GetName() + "/comm_out");
  this->sub = this->node->Subscribe<adhoc::msgs::Datagram>(this->model->GetName() + "/comm_in", &AdHocClientPlugin::OnMessage, this);

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
}


//////////////////////////////////////////////////
void AdHocClientPlugin::OnUpdate()
{
  // at some interval, initiate a communication.
  common::Time current = this->model->GetWorld()->GetSimTime();
  if (current.Double() - this->lastSent.Double() > 1.0)
  {
    this->msg_req.set_dst_address((this->id - 1 + 3) % 10 + 1);
    this->pub->Publish(this->msg_req);
    this->lastSent = current;
  }
}

//////////////////////////////////////////////////
void AdHocClientPlugin::OnMessage(const boost::shared_ptr<adhoc::msgs::Datagram const> &_msg)
{
  if (this->id == _msg->dst_address())
  {
    if (_msg->data() == "request")
    {
      gzmsg << this->model->GetName() << " got a request from " << _msg->model_name() << ". Replying..." << std::endl;
      this->msg_res.set_dst_address(_msg->src_address());
      this->pub->Publish(this->msg_res);
    }
    else if (_msg->data() == "response")
    {
      gzmsg << this->model->GetName() << " got a response from " << _msg->model_name() << std::endl;
    }
    else
    {
      gzmsg << this->model->GetName() << " got invalid data." << std::endl;
    }
  }
  else if (_msg->hops() < 10)
  {
    adhoc::msgs::Datagram forwardMsg = *_msg;
    forwardMsg.set_hops(_msg->hops() + 1);
    this->pub->Publish(forwardMsg);
  }
}
