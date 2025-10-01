# available events
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

# the event code
uint8 code
