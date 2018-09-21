#ifndef ADHOCNETPLUGIN_HH_
#define ADHOCNETPLUGIN_HH_

#include <memory>
#include <mutex>
#include <queue>
#include <thread>

#include <hector_uav_msgs/EnableMotors.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>

#include <gazebo/common/Event.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/math/Pose.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <sdf/sdf.hh>

#include "adhoc/CommonTypes.hh"
#include "quadrotor_tunnel_nav/protobuf/datagram.pb.h"
#include "quadrotor_tunnel_nav/protobuf/siminfo.pb.h"

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
    public: AdHocNetPlugin()
    {}

    // Documentation inherited
    public: virtual void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf);

    /// \brief Callback for World Update events.
    private: void OnUpdate();

    /// brief Checks all robots are ready to fly
    public: void CheckRobotsReadyTh();

    /// brief Callback to receive a response from a client.
    public: void OnSimCmdResponse(const boost::shared_ptr<adhoc::msgs::SimInfo const> &_res);

    /// \brief Process all incoming messages.
    private: void ProcessIncomingMsgs();

    /// \brief Callback executed when a new request is received.
    /// \param _req The datagram contained in the request.
    private: void OnMessage(const boost::shared_ptr<adhoc::msgs::Datagram const> &_req);

    /// \brief Make a hash string based on the message.
    private: void CalcHash(const adhoc::msgs::Datagram &_msg, unsigned char *_hash);

    /// \brief Check if the given hash value is already registered.
    private: bool HasHash(const unsigned char *_hash);

    /// \brief Register a hash value.
    private: void RegistHash(const unsigned char *_hash);

    /// \brief Check if the net topology changed.
    private: bool CheckTopoChange();

    /// \brief Initialize the simulation status.
    private: void StartNewTrial();

    /// \brief World pointer.
    private: physics::WorldPtr world;

    /// \brief Connection to World Update events.
    private: event::ConnectionPtr updateConnection;

    /// \brief An Ignition Transport node for communications.
    private: transport::NodePtr node;

    /// \brief ROS node handler
    private: ros::NodeHandle n;

    /// \brief list of robot names.
    private: std::vector<std::string> robotList;

    /// \brief list of simulation settings.
    private: std::queue<std::string> settingList;

    /// \brief True if the communication started.
    private: bool started;

    /// \brief True if the communicaiton finished.
    private: bool finished;

    /// \brief Simulation start time.
    private: common::Time startTime;

    /// \brief delay time currently used.
    private: double currentDelayTime;

    /// \brief robot speed currently used.
    private: double currentRobotSpeed;

    /// \brief Communciation range.
    private: double commRange;

    /// \brief Period to run the communication (in seconds)
    private: double simPeriod;

    /// \brief Total number of packets processed.
    private: int totalRecvPackets;

    /// \brief Total number of packets submitted.
    private: int totalSentPackets;

    /// \brief # of topology changes
    private: int topoChangeCount;

    /// \brief List of connections
    private: std::map<std::string, bool> topoList;

    /// \brief Subscriber for simulation command responses.
    private: transport::SubscriberPtr simCmdResSub;

    /// \brief Publisher for simulation command.
    private: transport::PublisherPtr simCmdPub;

    /// \brief List of received start responses.
    private: std::vector<boost::shared_ptr<adhoc::msgs::SimInfo const>>
      listStartResponses;

    /// \brief List of received stop responses.
    private: std::vector<boost::shared_ptr<adhoc::msgs::SimInfo const>>
      listStopResponses;

    /// \brief Mutex for the simulation information from clients.
    private: std::mutex simInfoMutex;

    /// \brief list of hash values
    private: std::vector<std::string> hashList;

    /// \brief the last time to print the robot positions on the terminal.
    private: common::Time lastStatPrintTime;

    /// \brief Thread object to check if robots are ready to fly.
    private: std::thread robotCheckThread;

    /// \brief Publisher to send a command to start flying.
    private: ros::Publisher startFlyingPub;

    /// \brief Publisher to update the robot speed.
    private: ros::Publisher navVelUpdatePub;

    /// \brief True if the all robots are flying at the specific altitude
    /// and ready to start netowrking.
    private: bool robotsReadyToComm;

    /// \brief Collection of incoming messages received during the last
    /// simulation step.
    private: std::deque<adhoc::msgs::Datagram> incomingMsgs;

    /// \brief publisher map to send data.
    private: std::map< std::string, transport::PublisherPtr > pubMap;

    /// \brief subscriber map to receive data.
    private: std::map< std::string, transport::SubscriberPtr > subMap;

    /// \brief Mutex for communication data.
    private: std::mutex messageMutex;
  };
}
#endif
