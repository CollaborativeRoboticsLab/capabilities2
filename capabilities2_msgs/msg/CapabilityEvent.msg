std_msgs/Header header

# Node name which published this event
string origin_node

# Capability which this event pertains to
Capability source

# Capability which this event targets at
Capability target

# Thread id of the capability which this event pertains to
int8 thread_id

# Events available
uint8 IDLE=0
uint8 STARTED=1
uint8 STOPPED=2
uint8 FAILED=3
uint8 SUCCEEDED=4
uint8 UNDEFINED=5

# Related event
uint8 event

# Event types available
uint8 INFO=0
uint8 DEBUG=1
uint8 ERROR=2
uint8 ERROR_ELEMENT=3

# Related event type
uint8 type

# status message
string content

# PID of the related process
int32 pid
