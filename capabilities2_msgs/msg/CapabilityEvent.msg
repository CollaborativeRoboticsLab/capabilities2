std_msgs/Header header

# Capability which this event pertains to
Capability source

# Capability which this event targets at
Capability target

# Capability trigger thread id
int8 thread_id

# status message
string text

# Event types
uint8 IDLE=0
uint8 STARTED=1
uint8 STOPPED=2
uint8 TERMINATED=3
uint8 SUCCEEDED=4
uint8 UNDEFINED=5

# Event type
uint8 event

# Whether the server configuration is complete
bool server_ready

# PID of the related process
int32 pid
