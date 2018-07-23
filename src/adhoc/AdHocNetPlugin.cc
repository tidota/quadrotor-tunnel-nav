#include <gazebo/common/Assert.hh>
#include <gazebo/common/Events.hh>

#include "adhoc/CommonTypes.hh"
#include "adhoc/AdHocNetPlugin.hh"
#include "quadrotor_tunnel_nav/protobuf/datagram.pb.h"

using namespace gazebo;

GZ_REGISTER_WORLD_PLUGIN(AdHocNetPlugin)

/////////////////////////////////////////////////
void AdHocNetPlugin::Load(physics::WorldPtr _world, sdf::ElementPtr /*_sdf*/)
{
  GZ_ASSERT(_world, "AdHocNetPlugin world pointer is NULL");
  this->world = _world;

  this->node = transport::NodePtr(new transport::Node());
  this->node->Init();
  for (int i = 1; i <= 10; ++i)
  {
    this->pubMap["robot" + std::to_string(i)]
      = this->node->Advertise<adhoc::msgs::Datagram>(
        "robot" + std::to_string(i) + "/comm_in");
    this->subMap["robot" + std::to_string(i)]
      = this->node->Subscribe<adhoc::msgs::Datagram>(
        "robot" + std::to_string(i) + "/comm_out",
        &AdHocNetPlugin::OnMessage, this);
  }

  this->updateConnection = event::Events::ConnectWorldUpdateBegin(
    std::bind(&AdHocNetPlugin::OnUpdate, this));

  gzmsg << "Starting Ad Hoc Net server" << std::endl;

}

/////////////////////////////////////////////////
void AdHocNetPlugin::OnUpdate()
{
  // ToDo: Step the comms model.

  std::lock_guard<std::mutex> lk(this->mutex);
  this->ProcessIncomingMsgs();
}

/////////////////////////////////////////////////
void AdHocNetPlugin::ProcessIncomingMsgs()
{
  while (!this->incomingMsgs.empty())
  {
    // Forward the messages.
    auto const &msg = this->incomingMsgs.front();

    physics::ModelPtr sender = this->world->GetModel(msg.model_name());

    if (sender)
    {
      for (int i = 1; i <= 10; ++i)
      {
        std::string name = "robot" + std::to_string(i);

        if (name == sender->GetName())
          continue;

        physics::ModelPtr robot = this->world->GetModel(name);

        if (robot)
        {
          // forward the message if the robot is within the range of the sender.
          auto diffVec = robot->GetWorldPose().CoordPositionSub(sender->GetWorldPose());
          double length = diffVec.GetLength();

          if (length <= 10.0)
          {
            this->pubMap[robot->GetName()]->Publish(msg);
          }
        }
      }
    }

    this->incomingMsgs.pop();
  }
}

/////////////////////////////////////////////////
void AdHocNetPlugin::OnMessage(const boost::shared_ptr<adhoc::msgs::Datagram const> &_req)
{
  // Just save the message, it will be processed later.
  std::lock_guard<std::mutex> lk(this->mutex);
  this->incomingMsgs.push(*_req);
}
