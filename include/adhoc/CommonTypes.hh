#ifndef COMMONTYPES_HH_
#define COMMONTYPES_HH_

#include <string>

// The following macroes are for old versions of gazebo.
#if GAZEBO_MAJOR_VERSION >= 8
#include <ignition/math/Pose3.hh>
#define POSE ignition::math::Pose3d
#define POS_X Pos().X()
#define POS_Y Pos().Y()
#define POS_Z Pos().Z()
#else
#include <gazebo/math/Pose.hh>
#define POSE math::Pose
#define POS_X pos.x
#define POS_Y pos.y
#define POS_Z pos.z
#define SimTime() GetSimTime()
#define RealTime() GetRealTime()
#define WorldPose() GetWorldPose()
#define ModelByName( NAME ) GetModel( NAME )
#define Length() GetLength()
#endif


/// \brief Address used to send a message to all the members of the team
/// listening on a specific port.
const std::string kBroadcast = "broadcast";

/// \brief Address used to bind to a multicast group. Note that we do not
/// support multiple multicast groups, only one.
const std::string kMulticast = "multicast";

/// \brief Address used to centralize all messages sent from the agents.
const std::string kBrokerService = "broker";

/// \brief Default port.
const uint32_t kDefaultPort = 4100u;

#endif
