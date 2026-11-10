uint8 WAIT_TRAJECTORY=0
uint8 ALIGN_HEADING=1
uint8 TRACK_TRAJECTORY=2
uint8 HOLD_FINAL=3
uint8 FAULT_STOP=4

int64 traj_id
uint8 state
float64 exec_time
bool frozen
builtin_interfaces/Time stamp
string fault_reason
