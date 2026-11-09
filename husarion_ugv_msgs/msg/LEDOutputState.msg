# State of the physical LED output of the bumper strips. Published latched on every change, so a
# client that connects later still learns the current state.

# False while the output is switched off with lights/enable. Frames are still received.
bool enabled

# True once the hardware has granted the driver LED control. Without it the strips stay dark even
# when the output is enabled.
bool control_granted

# Requested global brightness, in the [0.0, 1.0] range. The strips run it in 31 steps.
float32 brightness
