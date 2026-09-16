uint8 OK=0
uint8 WARN=1
uint8 ERROR=2
uint8 STALE=3

builtin_interfaces/Time timestamp
uint8 level
string code
string message
string source
uint8 target_state
diagnostic_msgs/KeyValue[] values
