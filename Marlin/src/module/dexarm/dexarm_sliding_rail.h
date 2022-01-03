#pragma once
void sliding_rail_home(feedRate_t home_feedrate_high, feedRate_t home_feedrate_low, int16_t homing_threshold_first, int16_t homing_threshold_second);
bool is_sliding_rail_homed();