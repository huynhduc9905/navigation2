uint8 STATE_UNKNOWN=0
uint8 STATE_STOPPED=1
uint8 STATE_PLAYING=2

string device
bool   active
uint32 queue_length
string current_uri
string current_name
uint8  state

builtin_interfaces/Duration position
builtin_interfaces/Duration duration

string last_error
