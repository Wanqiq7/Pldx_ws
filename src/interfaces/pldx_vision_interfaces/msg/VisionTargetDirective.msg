# Navigation/referee -> vision target selection directive.
# Topic: vision/target_directive.  Consumers drop it after a timeout
# (vision default 750 ms), so producers must republish periodically.

# Bit flags for source_mask.
uint8 SOURCE_NAVIGATION = 1
uint8 SOURCE_REFEREE = 2

std_msgs/Header header

# False means "no directive": the consumer falls back to default selection.
bool valid

# Which producers contributed to this directive (SOURCE_* bits).
uint8 source_mask

# Target ids (1-based) that must not be engaged.
uint8[] excluded_target_ids

# When true, only ids in preferred_target_ids may be engaged; an empty list
# therefore means "engage nothing".
bool restrict_to_preferred_targets

# Target ids (1-based) to prefer, highest priority first.
uint8[] preferred_target_ids
