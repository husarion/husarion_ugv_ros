# This message defines the battery charging process status

# Charger types
uint8 UNKNOWN = 0
uint8 WIRED = 1
uint8 WIRELESS = 2

std_msgs/Header header
bool charging               # True if battery is being charged
float32 current             # Power supply total current (A)
float32 current_battery_1   # Power supply current (A) for battery 1
float32 current_battery_2   # Power supply current (A) for battery 2 (NaN for single battery configuration)
uint8 charger_type          # Determines the type of charger connection
