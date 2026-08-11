# the capability event message

# id for the for the bond this event is related to
string bond_id

# id for the instance that triggered this event
string trigger_id

# event can pass over a connection
# could represent connection between capabilities
# usually via runners
# Capabilities which this event pertains to (source and target)
capabilities2_msgs/CapabilityConnection connection

# the event code
capabilities2_msgs/CapabilityEventCode code

# optional event message
string description
