# available events
uint8 IDLE=0 # state and general
# runner state events
uint8 STARTED=1 # on_started
uint8 STOPPED=2 # on_stopped
uint8 FAILED=3  # on_failure
uint8 SUCCEEDED=4 # on_success
# runner configuration events
uint8 CONNECTED=5
uint8 DISCONNECTED=6
uint8 TRIGGERED=7
# system events
uint8 SERVER_READY=8 # server ready
uint8 LAUNCHED=9 # process launched
uint8 TERMINATED=10 # process terminated

uint8 ERROR=18 # defineable error event
# something out of scope went wrong code
uint8 UNDEFINED=19

# highest code
uint8 MAX_CODE=20

# the event code
uint8 code
