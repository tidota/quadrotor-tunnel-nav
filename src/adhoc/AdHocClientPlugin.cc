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

  this->msg.set_model_name(this->model->GetScopedName());
  this->msg.set_src_address(this->id);
  this->msg.set_hops(0);
  this->msg.set_data("req");
  this->lastSent = this->model->GetWorld()->GetSimTime();
}


//////////////////////////////////////////////////
void AdHocClientPlugin::OnUpdate()
{
  // at some interval, initiate a communication.
  common::Time current = this->model->GetWorld()->GetSimTime();
  if (current.Double() - this->lastSent.Double() > 1.0)
  {
    this->msg.set_dst_address(100);
    this->pub->Publish(this->msg);
    this->lastSent = current;
  }
}

//////////////////////////////////////////////////
void AdHocClientPlugin::OnMessage(const adhoc::msgs::Datagram &_msg)
{
  gzmsg << "Message received" << std::endl;
}
