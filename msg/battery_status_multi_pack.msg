uint64 timestamp			# time since system start (microseconds)

uint8 MAX_BATTERY_PACK_COUNT = 16   	# maximum number of battery supported

# battery source static const
uint8 BATTERY_SOURCE_POWER_MODULE = 0
uint8 BATTERY_SOURCE_EXTERNAL = 1
uint8 BATTERY_SOURCE_ESCS = 2

# battery warning static const
uint8 BATTERY_WARNING_NONE = 0       	# no battery low voltage warning active
uint8 BATTERY_WARNING_LOW = 1        	# warning of low voltage
uint8 BATTERY_WARNING_CRITICAL = 2   	# critical voltage, return / abort immediately
uint8 BATTERY_WARNING_EMERGENCY = 3   	# immediate landing required
uint8 BATTERY_WARNING_FAILED = 4   	# the battery has failed completely

float32[16] voltage_v			# Battery voltage in volts, 0 if unknown
float32[16] voltage_filtered_v		# Battery voltage in volts, filtered, 0 if unknown
float32[16] current_a			# Battery current in amperes, -1 if unknown
float32[16] current_filtered_a		# Battery current in amperes, filtered, 0 if unknown
float32[16] average_current_a		# Battery current average in amperes, -1 if unknown
float32[16] discharged_mah		# Discharged amount in mAh, -1 if unknown
float32[16] remaining			# From 1 to 0, -1 if unknown
float32[16] scale			# Power scaling factor, >= 1, or -1 if unknown
float32[16] temperature         	# temperature of the battery. NaN if unknown
int32[16] cell_count			# Number of cells
bool[16] connected			# Whether or not a battery is connected, based on a voltage threshold

uint8[16] source			# Battery source
uint8[16] priority        		# Zero based priority is the connection on the Power Controller V1..Vn AKA BrickN-1
uint16[16] capacity         		# actual capacity of the battery
uint16[16] cycle_count         		# number of discharge cycles the battery has experienced
uint16[16] run_time_to_empty   		# predicted remaining battery capacity based on the present rate of discharge in min
uint16[16] average_time_to_empty   	# predicted remaining battery capacity based on the average rate of discharge in min
uint16[16] serial_number       		# serial number of the battery pack
uint16[16] state_of_health		# state of health. FullChargeCapacity/DesignCapacity.
uint16[16] max_error 			# max error, expected margin of error in % in the state-of-charge calculation with a range of 1 to 100%

uint8[16] id                		# ID number of a battery. Should be unique and consistent for the lifetime of a vehicle. 1-indexed.
bool[16] is_powering_off		# Power off event imminent indication, false if unknown

uint8[16] warning    			# current battery warning
