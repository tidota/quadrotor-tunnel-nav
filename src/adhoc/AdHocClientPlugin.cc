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
#include <string>

#include "AdHocClientPlugin.hh"
#include "CommonTypes.hh"
#include "quadrotor_tunnel_nav/protobuf/datagram.pb.h"

//////////////////////////////////////////////////
void AdHocClientPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  this->model = _model;

  // Sanity check: Verity that local address is not empty.
  if (_sdf->HasElement("local_address"))
  {
    this->localAddress = _sdf->Get<std::string>("local_address");
  }
  else
  {
    std::cerr << "AdHocClientPlugin::AdHocClientPlugin() error: Local address shouldn't "
              << "be empty" << std::endl;
  }
}


//////////////////////////////////////////////////
void OnUpdate()
{
  // at some interval, initiate a communication.
}

//////////////////////////////////////////////////
std::string AdHocClientPlugin::Host() const
{
  return this->localAddress;
}

//////////////////////////////////////////////////
bool AdHocClientPlugin::SendTo(const std::string &_data,
    const std::string &_dstAddress, const uint32_t _port)
{
  // Sanity check: Make sure that we're using a valid address.
  if (this->Host().empty())
    return false;

  // Restrict the maximum size of a message.
  if (_data.size() > this->kMtu)
  {
    std::cerr << "[" << this->Host() << "] AdHocClientPlugin::SendTo() error: "
              << "Payload size (" << _data.size() << ") is greater than the "
              << "maximum allowed (" << this->kMtu << ")" << std::endl;
    return false;
  }

  msgs::Datagram msg;
  msg.set_src_address(this->Host());
  msg.set_dst_address(_dstAddress);
  msg.set_dst_port(_port);
  msg.set_data(_data);

  return this->node.Request(kBrokerService, msg);
}

//////////////////////////////////////////////////
void AdHocClientPlugin::OnMessage(const msgs::Datagram &_msg)
{
  auto endPoint = _msg.dst_address() + ":" + std::to_string(_msg.dst_port());

  for (auto cb : this->callbacks)
  {
    if (cb.first == endPoint && cb.second)
    {
      cb.second(_msg.src_address(), _msg.dst_address(),
                _msg.dst_port(), _msg.data());
    }
  }
}
