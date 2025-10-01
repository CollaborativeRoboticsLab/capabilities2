# the capability event message

# identifying name for the source that triggered this event
string trigger_id

# Capabilities which this event pertains to (source and target)
# also represents connections between capabilities
# usually via runners
capabilities2_msgs/CapabilityConnection connection

# the event code
capabilities2_msgs/CapabilityEventCode code

# optional event message
string description
