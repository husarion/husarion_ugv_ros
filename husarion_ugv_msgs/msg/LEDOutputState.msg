# State of the physical LED output on the bumper strips. Latched, so a client
# that connects later still learns the current state instead of guessing.

# False when the driver is receiving frames but not writing them to the strip.
bool enabled

# Global brightness applied to every frame, in the [0.0, 1.0] range.
float32 brightness
