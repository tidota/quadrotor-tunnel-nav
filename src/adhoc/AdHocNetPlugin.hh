#ifndef ADHOCNETPLUGIN_HH_
#define ADHOCNETPLUGIN_HH_

#include <mutex>
#include <queue>
#include <gazebo/common/Event.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <ignition/transport/Node.hh>
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
    // Documentation inherited
    public: virtual void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf);

    /// \brief Callback for World Update events.
    private: void OnUpdate();

    /// \brief Process all incoming messages.
    private: void ProcessIncomingMsgs();

    /// \brief Callback executed when a new request is received.
    /// \param _req The datagram contained in the request.
    private: void OnMessage(const adhoc::msgs::Datagram &_req);

    /// \brief World pointer.
    private: physics::WorldPtr world;

    /// \brief Connection to World Update events.
    private: event::ConnectionPtr updateConnection;

    /// \brief An Ignition Transport node for communications.
    private: ignition::transport::Node node;

    /// \brief Collection of incoming messages received during the last
    /// simulation step.
    private: std::queue<adhoc::msgs::Datagram> incomingMsgs;

    /// \brief Protect data from races.
    private: std::mutex mutex;
  };
}
#endif
