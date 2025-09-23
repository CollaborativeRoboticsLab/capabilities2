# the capability event message type

# Capabilities which this event pertains to (source and target)
# also represents connections between capabilities
# usually via runners
capabilities2_msgs/CapabilityConnection connection

# PID of the related process
int32 pid

# Thread id of the capability which this event pertains to
int8 thread_id

# Node name which published this event
string node_id

# Events available
uint8 IDLE=0
# runner events
uint8 STARTED=1 # on_started
uint8 STOPPED=2 # on_stopped
uint8 FAILED=3  # on_failure
uint8 SUCCEEDED=4 # on_success
uint8 UNDEFINED=5
# system events
uint8 SERVER_READY=6 # server ready
uint8 LAUNCHED=7 # process launched
uint8 TERMINATED=8 # process terminated

# Related event
uint8 event

# optional event message
string description

# Event level available
uint8 INFO=0
uint8 DEBUG=1
uint8 ERROR=2
uint8 ERROR_ELEMENT=3
uint8 RUNNER_DEFINE=4
uint8 RUNNER_EVENT=5

# Related event level and optional message
uint8 level
string message
