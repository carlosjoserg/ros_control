# This message contains the state of one realtime controller
# that was spawned in the controller manager

# the name of the controller
string name

# the type of the controller
string type

# the time at which these controller statistics were measured
builtin_interfaces/Time timestamp

# bool that indicates if the controller is currently
# in a running or a stopped state
bool running

# the maximum time the update loop of the controller ever needed to complete
builtin_interfaces/Duration max_time

# the average time the update loop of the controller needs to complete.
# the average is computed in a sliding time window.
builtin_interfaces/Duration mean_time

# the variance on the time the update loop of the controller needs to complete.
# the variance applies to a sliding time window.
builtin_interfaces/Duration variance_time

# the number of times this controller broke the realtime loop
int32 num_control_loop_overruns

# the timestamp of the last time this controller broke the realtime loop
builtin_interfaces/Time time_last_control_loop_overrun
