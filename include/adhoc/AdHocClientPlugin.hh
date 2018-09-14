#ifndef ADHOCCLIENTPLUGIN_HH_
#define ADHOCCLIENTPLUGIN_HH_

#include <cstdint>
#include <functional>
#include <iostream>
#include <map>
#include <memory>
#include <queue>
#include <utility>

#include <ros/ros.h>
#include <std_msgs/Bool.h>

#include <gazebo/common/Plugin.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>

#include "adhoc/CommonTypes.hh"
#include "quadrotor_tunnel_nav/protobuf/datagram.pb.h"
#include "quadrotor_tunnel_nav/protobuf/siminfo.pb.h"

namespace gazebo
{
  /// \brief ToDo.
  class AdHocClientPlugin : public ModelPlugin
  {
    // Documentation inherited
    public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);

    /// \brief Callback for World Update events.
    private: void OnUpdate();

    /// \brief Callback to receive a request from the world plugin.
    public: void OnSimCmd(const boost::shared_ptr<adhoc::msgs::SimInfo const> &_req);

    /// \brief Process all incoming messages.
    private: void ProcessincomingMsgsStamped();

    /// \brief Function called each time a new datagram message is received.
    /// \param[in] _msg The incoming message.
    private: void OnMessage(const boost::shared_ptr<adhoc::msgs::Datagram const> &_msg);

    /// \brief Make a hash string based on the message.
    private: void CalcHash(const adhoc::msgs::Datagram &_msg, unsigned char *_hash);

    /// \brief Check if the given hash value is already registered.
    private: bool HasHash(const unsigned char *_hash);

    /// \brief Register a hash value.
    private: void RegistHash(const unsigned char *_hash);

    /// \brief Model pointer.
    private: physics::ModelPtr model;

    /// \brief An Ignition Transport node for communications.
    private: transport::NodePtr node;

    /// \brief id in the network
    private: unsigned int id;

    /// \brief Subscriber to the simulation command.
    private: transport::SubscriberPtr simCommSub;

    /// \brief Publisher to respond to the simulation command.
    private: transport::PublisherPtr simCommResPub;

    /// \brief True if the communication running.
    private: bool running;

    /// \brief True if the communication stopped.
    private: bool finished;

    // statistics
    private: int totalMessages;
    private: int totalHops;
    private: double totalRoundTripTime;

    /// \brief Mutex for simulation information.
    private: std::mutex simInfoMutex;

    /// \brief index of message
    private: unsigned int messageCount;

    /// \brief the time when the node last sent a message.
    private: common::Time lastSentTime;

    /// \brief the time when the node last processed messages.
    private: double delayTime;

    /// \brief publisher to send out data.
    private: transport::PublisherPtr pub;

    /// \brief subscriber to receive data.
    private: transport::SubscriberPtr sub;

    /// \brief list of hash values
    private: std::vector<std::string> hashList;

    /// \brief message to send
    private: adhoc::msgs::Datagram msg_req;

    /// \brief message to send
    private: adhoc::msgs::Datagram msg_res;

    /// \brief pointer to the update even connection.
    private: event::ConnectionPtr updateConnection;

    /// \brief Collection of incoming messages received during the last
    /// simulation step.
    private: std::queue<
      std::pair<adhoc::msgs::Datagram, common::Time> > incomingMsgsStamped;

    /// \brief Protect data from races.
    private: std::mutex messageMutex;
  };
}
#endif
