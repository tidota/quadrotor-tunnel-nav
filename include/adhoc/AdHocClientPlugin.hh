#ifndef ADHOCCLIENTPLUGIN_HH_
#define ADHOCCLIENTPLUGIN_HH_

#include <cstdint>
#include <functional>
#include <iostream>
#include <list>
#include <map>
#include <memory>
#include <utility>

#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>

#include <gazebo/common/Plugin.hh>
#include <gazebo/math/Pose.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>

#include "adhoc/CommonTypes.hh"
#include "quadrotor_tunnel_nav/protobuf/datagram.pb.h"
#include "quadrotor_tunnel_nav/protobuf/siminfo.pb.h"

namespace gazebo
{
  /// \brief A model plugin to send/receive packets and also responds to
  /// commands from the world plugin AdHocNetPlugin.
  class AdHocClientPlugin : public ModelPlugin
  {
    // Documentation inherited
    public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);

    /// \brief Callback for World Update events.
    private: void OnUpdate();

    /// \brief Callback to receive a request from the world plugin.
    /// \param[in] _req A message containing a command from the world plugin.
    public: void OnSimCmd(
      const boost::shared_ptr<adhoc::msgs::SimInfo const> &_req);

    /// \brief Callback function to monitor cmd_vel.
    /// \param[in] _event A message containing a velocity vector.
    public: void OnCmdVel(
      const ros::MessageEvent<geometry_msgs::Twist const>& _event);

    /// \brief Process all incoming messages.
    private: void ProcessincomingMsgsStamped();

    /// \brief Function called each time a new datagram message is received.
    /// \param[in] _msg The incoming message.
    private: void OnMessage(
        const boost::shared_ptr<adhoc::msgs::Datagram const> &_msg);

    /// \brief Make a hash string based on the message.
    /// \param[in] _msg A message to caculate a hash value.
    /// \param[out] _hash A hash value of the message.
    private: void CalcHash(
      const adhoc::msgs::Datagram &_msg, unsigned char *_hash);

    /// \brief Check if the given hash value is already registered.
    /// \param[in] _hash A hash value to check.
    /// \return True if the hash value has already been registered. Otherwise,
    ///         False.
    private: bool HasHash(const unsigned char *_hash);

    /// \brief Register a hash value.
    /// \param[in] _hash A hash value to register.
    private: void RegistHash(const unsigned char *_hash);

    /// \brief Model pointer.
    private: physics::ModelPtr model;

    /// \brief pointer to the update even connection.
    private: event::ConnectionPtr updateConnection;

    /// \brief An Ignition Transport node for communications.
    private: transport::NodePtr node;

    /// \brief subscriber to receive data.
    private: transport::SubscriberPtr sub;

    /// \brief publisher to send out data.
    private: transport::PublisherPtr pub;

    /// \brief Collection of incoming messages received during the last
    /// simulation step.
    private: std::list<std::pair<adhoc::msgs::Datagram, common::Time> >
        incomingMsgsStamped;

    /// \brief Subscriber to the simulation command.
    private: transport::SubscriberPtr simCommSub;

    /// \brief Publisher to respond to the simulation command.
    private: transport::PublisherPtr simCommResPub;

    /// \brief Subscriber to monior the cmd_vel.
    private: ros::Subscriber cmdVelMonitorSub;

    /// \brief id in the network
    private: unsigned int id;

    /// \brief A request message to send.
    private: adhoc::msgs::Datagram msg_req;

    /// \brief A response message to send.
    private: adhoc::msgs::Datagram msg_res;

    /// \brief the time when the node last processed messages.
    private: double delayTime;

    /// \brief the time when the node last sent a message.
    private: common::Time lastSentTime;

    /// \brief True if the communication running.
    private: bool running;

    /// \brief True if the client is no longer creating new messages.
    private: bool stopping;

    /// \brief True if the communication stopped.
    private: bool finished;

    /// \brief Set this to true so cmd_vel is zero before responding.
    private: bool sendStoppedResponse;

    /// \brief list of hash values
    private: std::vector<std::string> hashList;

    /// \brief The number of received messages.
    private: int recvMessageCount;

    /// \brief The number of sent messages. Used as an index of a new message.
    private: unsigned int sentMessageCount;

    /// \brief The total number of hops taken cumulated for all packets.
    private: int totalHops;

    /// \brief The total distance taken to get the destination cumulated for all
    /// packets.
    private: double totalRoundTripTime;

    /// \brief (For tracking purposes) total communication distance taken by
    /// packets.
    private: double totalDistComm;

    /// \brief (For tracking purposes) total motion distance taken by packets.
    private: double totalDistMotion;

    /// \brief Protect data from races.
    private: std::mutex messageMutex;

    /// \brief Mutex for simulation information.
    private: std::mutex simInfoMutex;
  };
}
#endif
