uint64 timestamp		# time since system start (microseconds)
uint8 source_id         	# ID number of a battery. Should be unique and consistent for the lifetime of a vehicle. 1-indexed.
float32 voltage_v		# Battery voltage in volts, 0 if unknown
uint16 run_time_to_empty   	# predicted remaining battery capacity based on the present rate of discharge in min
uint8 warning    		# current battery warning

float32 total_current_a		# combined current of the vehicle for motor failure check.

uint8 BATTERY_WARNING_NONE = 0       	# no battery low voltage warning active
uint8 BATTERY_WARNING_LOW = 1        	# warning of low voltage
uint8 BATTERY_WARNING_CRITICAL = 2   	# critical voltage, return / abort immediately
uint8 BATTERY_WARNING_EMERGENCY = 3  	# immediate landing required
uint8 BATTERY_WARNING_FAILED = 4   	# the battery has failed completely


