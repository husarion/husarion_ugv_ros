# Battery undervoltage protection (UVP) state, published by the battery driver when the protection
# is enabled. The battery driver owns the thresholds and the reaction settings, the hardware system
# and the managers act on what this message carries.

uint8 STATE_UNKNOWN = 0      # no valid voltage read yet
uint8 STATE_OK = 1
uint8 STATE_CHARGE_ASAP = 2  # charge as soon as possible
uint8 STATE_UVP = 3          # undervoltage protection tripped, latched

uint8 MODE_STOP = 0          # e-stop at the trip, shutdown after stop_timeout_min without charging
uint8 MODE_SHUTDOWN = 1      # shutdown at the trip

std_msgs/Header header
uint8 state                       # Robot state, the most severe of the batteries
uint8[] battery_states            # State per battery (one entry, two on a dual battery Panther)
bool reactions_enabled            # False when the reactions are off, see reactions_disabled_reason
string reactions_disabled_reason
uint8 mode
float32 stop_timeout_min          # T, minutes without charging before the shutdown in MODE_STOP
float32 drive_window_s            # Drive time after an e-stop reset while UVP is latched (s)
bool aux_off_at_trip              # Turn the AUX power off at the trip
float32 shutdown_hosts_timeout_s  # Upper bound of the per-host wait of the UVP shutdown (s)
float32 uvp_threshold             # V
float32 charge_asap_threshold     # V
string threshold_source           # "param", "eeprom" or "default"
uint32 trip_id                    # Increments on each UVP trip of the robot
float32[] trip_voltage            # Raw voltage per battery at its UVP trip (V), NaN if none
float32[] trip_current            # Discharge current per battery at its UVP trip (A), NaN if none
builtin_interfaces/Time trip_stamp
float32 seconds_since_trip        # On CLOCK_BOOTTIME, survives driver and manager restarts
bool charging                     # Charging, debounced
