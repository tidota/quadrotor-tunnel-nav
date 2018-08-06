#ifndef ADHOCNETPLUGIN_HH_
#define ADHOCNETPLUGIN_HH_

#include <mutex>
#include <memory>
#include <queue>
#include <thread>

#include <ros/ros.h>
#include <std_msgs/Bool.h>

#include <gazebo/common/Event.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <hector_uav_msgs/EnableMotors.h>
#include <sdf/sdf.hh>

namespace adhoc
{
  namespace msgs
  {
    // Forward declarations.
    class Datagram;
  }
}

namespace gazebo
{
  /// \brief A plugin that controls ad hoc communications.
  class AdHocNetPlugin : public WorldPlugin
  {
    /// Constructor.
    public: AdHocNetPlugin(): robotsFlying(false)
    {}

    // Documentation inherited
    public: virtual void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf);

    /// brief Checks all robots are ready to fly
    public: void CheckRobotsReadyTh();

    // to receive a message to start operation.
    public: void OnStartStopMessage(const ros::MessageEvent<std_msgs::Bool const>& event);

    /// \brief Callback for World Update events.
    private: void OnUpdate();

    /// \brief Process all incoming messages.
    private: void ProcessIncomingMsgs();

    /// \brief Callback executed when a new request is received.
    /// \param _req The datagram contained in the request.
    private: void OnMessage(const boost::shared_ptr<adhoc::msgs::Datagram const> &_req);

    private: void OnClientMessage(const boost::shared_ptr<gazebo::msgs::GzString const> &_data);

    /// \brief Make a hash string based on the message.
    private: void CalcHash(const adhoc::msgs::Datagram &_msg, unsigned char *_hash);

    /// \brief Check if the given hash value is already registered.
    private: bool HasHash(const unsigned char *_hash);

    /// \brief Register a hash value.
    private: void RegistHash(const unsigned char *_hash);

    private: void InitTopoList();
    /// \brief Check if the net topology changed.
    private: bool CheckTopoChange();

    /// \brief Thread object to check if robots are ready to fly.
    private: std::thread robotCheckThread;

    /// \brief True if the all robots started flying.
    private: bool robotsFlying;

    /// \brief Publisher to send a command to start flying.
    private: ros::Publisher pubStartFlying;

    /// \brief Mutex for the thread.
    private: std::mutex mutexRobotCheck;

    /// \brief list of robot names.
    private: std::vector<std::string> robotList;

    /// \brief World pointer.
    private: physics::WorldPtr world;

    /// \brief Connection to World Update events.
    private: event::ConnectionPtr updateConnection;

    /// \brief An Ignition Transport node for communications.
    private: transport::NodePtr node;

    /// \brief publisher map to send data.
    private: std::map< std::string, transport::PublisherPtr > pubMap;

    /// \brief subscriber map to receive data.
    private: std::map< std::string, transport::SubscriberPtr > subMap;

    /// \brief Collection of incoming messages received during the last
    /// simulation step.
    private: std::queue<adhoc::msgs::Datagram> incomingMsgs;

    /// \brief the time when the robot positions were last displayed.
    private: common::Time lastDisplayed;

    /// \brief Protect data from races.
    private: std::mutex mutex;

    private: ros::NodeHandle n;

    private: bool started;
    private: bool finished;

    private: ros::Subscriber enableSub;

    private: std::mutex mutexStartStop;

    private: transport::SubscriberPtr clientOutputSub;

    private: double commRange;

    // statistics
    private: int totalPackets;

    /// \brief list of hash values
    private: std::vector<std::string> hashList;

    private: std::map<std::string, bool> topoList;

    /// \brief # of topology changes
    private: int topoChangeCount;

    private: common::Time startTime;

    /// \brief how long time to run the simulation
    private: double simPeriod;

  };
}
#endif
