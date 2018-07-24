#ifndef COMMONTYPES_HH_
#define COMMONTYPES_HH_

#include <string>

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
