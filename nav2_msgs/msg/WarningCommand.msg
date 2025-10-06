# Commands
uint8 PLAY_FILE=1      # (kept for backward-compat)
uint8 STOP=2           # stop current and clear queue
uint8 CLEAR_QUEUE=3    # clear queued warnings (current keeps playing)
uint8 SET_DEVICE=4     # change ALSA device for warnings
uint8 PLAY_SIGNAL=5    # NEW: play a named signal (enum below)

# Signals (add what you need)
uint8 BATTERY_LOW=0
uint8 ROBOT_STUCK=1
uint8 OBSTACLE_NEAR=2
uint8 GOAL_REACHED=3
uint8 NAVIGATING=4
uint8 GOAL_FAIL=5

# Command to execute
uint8 cmd

# Args (used depending on cmd)
string path     # PLAY_FILE
string device   # SET_DEVICE
uint8  signal   # PLAY_SIGNAL (use one of the enums above)
bool repeat
uint8 period_s
