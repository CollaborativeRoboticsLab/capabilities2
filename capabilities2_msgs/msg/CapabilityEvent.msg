std_msgs/Header header
# Capability which this event pertains to
string capability
# Capability provider which this event pertains to
string provider

# Event types
string LAUNCHED="launched"
string STOPPED="stopped"
string TERMINATED="terminated"
string SERVER_READY="server_ready"
# Event type
string type

# PID of the related process
int32 pid
