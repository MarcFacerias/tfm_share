Header header

# Lateral command; reciprocal of desired centre-of-gravity turn radius in 1/m,
# essentially proportional to steered wheel angle with factor of wheelbase
# making it vehicle-independent
# Left positive, right negative
float32 left_steer_turn_radius_reciprocal_command

# Longitudinal commands; accelerator and brake left separate so that an
# AV needing to apply both (e.g. holding on hill to avoid roll-back during start)
# Forwards is positive [m/s]
float32 acceleration_command

# Positive number means braking is requested [m/s]
float32 brake_deceleration_command
