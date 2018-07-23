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
#ifndef ADHOCCLIENTPLUGIN_HH_
#define ADHOCCLIENTPLUGIN_HH_

#include <cstdint>
#include <functional>
#include <iostream>
#include <map>
#include <memory>

#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>

#include "adhoc/CommonTypes.hh"
#include "quadrotor_tunnel_nav/protobuf/datagram.pb.h"

namespace gazebo
{
  /// \brief ToDo.
  class AdHocClientPlugin : public ModelPlugin
  {
    // Documentation inherited
    public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);

    /// \brief Callback for World Update events.
    private: void OnUpdate();

    /// \brief Function called each time a new datagram message is received.
    /// \param[in] _msg The incoming message.
    private: void OnMessage(const boost::shared_ptr<adhoc::msgs::Datagram const> &_msg);

    /// \brief An Ignition Transport node for communications.
    private: transport::NodePtr node;

    /// \brief publisher to send out data.
    private: transport::PublisherPtr pub;

    /// \brief subscriber to receive data.
    private: transport::SubscriberPtr sub;

    /// \brief Model pointer.
    private: physics::ModelPtr model;

    /// \brief id in the network
    private: int id;

    /// \brief message to send
    private: adhoc::msgs::Datagram msg_req;

    /// \brief message to send
    private: adhoc::msgs::Datagram msg_res;

    /// \brief the time when the node last sent a message.
    private: common::Time lastSent;

    /// \brief pointer to the update even connection.
    private: event::ConnectionPtr updateConnection;
  };
}
#endif
