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
#include <queue>

#include <ros/ros.h>
#include <std_msgs/Bool.h>

#include <gazebo/common/Plugin.hh>
//#include <gazebo/msgs/GzString.hh>
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

    // to receive a message to start operation.
    public: void OnStartStopMessage(const ros::MessageEvent<std_msgs::Bool const>& event);

    /// \brief Callback for World Update events.
    private: void OnUpdate();

    /// \brief Process all incoming messages.
    private: void ProcessIncomingMsgs();

    /// \brief Function called each time a new datagram message is received.
    /// \param[in] _msg The incoming message.
    private: void OnNetworkMessage(const boost::shared_ptr<adhoc::msgs::Datagram const> &_msg);

    /// \brief Make a hash string based on the message.
    private: void CalcHash(const adhoc::msgs::Datagram &_msg, unsigned char *_hash);

    /// \brief Check if the given hash value is already registered.
    private: bool HasHash(const unsigned char *_hash);

    /// \brief Register a hash value.
    private: void RegistHash(const unsigned char *_hash);

    /// \brief An Ignition Transport node for communications.
    private: transport::NodePtr node;

    /// \brief publisher to send out data.
    private: transport::PublisherPtr pub;

    /// \brief subscriber to receive data.
    private: transport::SubscriberPtr sub;

    /// \brief Model pointer.
    private: physics::ModelPtr model;

    /// \brief id in the network
    private: unsigned int id;

    /// \brief index of message
    private: unsigned int messageCount;

    /// \brief list of hash values
    private: std::vector<std::string> hashList;

    /// \brief message to send
    private: adhoc::msgs::Datagram msg_req;

    /// \brief message to send
    private: adhoc::msgs::Datagram msg_res;

    /// \brief the time when the node last sent a message.
    private: common::Time lastSent;

    /// \brief pointer to the update even connection.
    private: event::ConnectionPtr updateConnection;

    /// \brief Collection of incoming messages received during the last
    /// simulation step.
    private: std::queue<adhoc::msgs::Datagram> incomingMsgs;

    /// \brief Protect data from races.
    private: std::mutex mutex;

    private: ros::NodeHandle n;

    private: bool started;
    private: bool finished;

    private: ros::Subscriber enableSub;

    private: std::mutex mutexStartStop;

    private: transport::PublisherPtr clientOutputPub;

    // statistics
    private: int totalPackets;
    private: int totalHops;
    private: double totalTravelTime;
  };
}
#endif
