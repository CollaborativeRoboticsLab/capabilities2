std_msgs/Header header

# Node name which published this event
string origin_node

# Capability which this event pertains to
Capability source

# Capability which this event targets at
Capability target

# Thread id of the capability which this event pertains to
int8 thread_id

# Event types available
uint8 IDLE=0
uint8 STARTED=1
uint8 STOPPED=2
uint8 FAILED=3
uint8 SUCCEEDED=4
uint8 UNDEFINED=5

# Specific event that occurred
uint8 event

# Whether this error relates to a failure
bool error

# status message
string text

# Whether output data containes xml elements
bool is_failed_element

# string contianing xml elements
string element

# PID of the related process
int32 pid
