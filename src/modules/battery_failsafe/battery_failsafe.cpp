/****************************************************************************
 *
 *   Copyright (c) 2018 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#include "battery_failsafe.h"

#include <px4_platform_common/getopt.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/posix.h>

#define VOLTAGE_DROP_CHECK(now, last_time) (abs((double)(now - last_time)) > VOLTAGE_DROP_DELAY_US)

int BatteryFailsafe::print_status()
{
	PX4_INFO("Running");
	// TODO: print additional runtime information about the state of the module

	return 0;
}

int BatteryFailsafe::custom_command(int argc, char *argv[])
{
	/*
	if (!is_running()) {
		print_usage("not running");
		return 1;
	}

	// additional custom commands can be handled like this:
	if (!strcmp(argv[0], "do-something")) {
		get_instance()->do_something();
		return 0;
	}
	 */

	return print_usage("unknown command");
}


int BatteryFailsafe::task_spawn(int argc, char *argv[])
{
	_task_id = px4_task_spawn_cmd("battery_failsafe",
				      SCHED_DEFAULT,
				      SCHED_PRIORITY_DEFAULT,
				      2048,
				      (px4_main_t)&run_trampoline,
				      (char *const *)argv);

	if (_task_id < 0) {
		_task_id = -1;
		return -errno;
	}

	return 0;
}

BatteryFailsafe *BatteryFailsafe::instantiate(int argc, char *argv[])
{
	int example_param = 0;
	bool example_flag = false;
	bool error_flag = false;

	int myoptind = 1;
	int ch;
	const char *myoptarg = nullptr;

	// parse CLI arguments
	while ((ch = px4_getopt(argc, argv, "p:f", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'p':
			example_param = (int)strtol(myoptarg, nullptr, 10);
			break;

		case 'f':
			example_flag = true;
			break;

		case '?':
			error_flag = true;
			break;

		default:
			PX4_WARN("unrecognized flag");
			error_flag = true;
			break;
		}
	}

	if (error_flag) {
		return nullptr;
	}

	BatteryFailsafe *instance = new BatteryFailsafe(example_param, example_flag);

	if (instance == nullptr) {
		PX4_ERR("alloc failed");
	}

	return instance;
}

BatteryFailsafe::BatteryFailsafe(int example_param, bool example_flag)
	: ModuleParams(nullptr)
{
	// Initialize variable
	failsafe_status.warning = battery_failsafe_s::BATTERY_WARNING_NONE;
	failsafe_status.source_id = 0;
	failsafe_status.voltage_v = 0.0f;
	failsafe_status.run_time_to_empty = 0;
}

void BatteryFailsafe::run()
{
	px4_pollfd_struct_t fds[6];

	int _battery_subs[ORB_MULTI_MAX_INSTANCES];
	for (unsigned i = 0; i < ORB_MULTI_MAX_INSTANCES; i++)
	{
		_battery_subs[i] = orb_subscribe_multi(ORB_ID(battery_status),i);
		orb_set_interval(_battery_subs[i], 200);

		fds[i].fd = _battery_subs[i];
		fds[i].events = POLLIN;
	}

	int _battery_multi_pack_sub = orb_subscribe(ORB_ID(battery_status_multi_pack));
	orb_set_interval(_battery_multi_pack_sub, 200);
	fds[4].fd = _battery_multi_pack_sub;
	fds[4].events = POLLIN;

	int _vehicle_status_sub = orb_subscribe(ORB_ID(vehicle_status));
	orb_set_interval(_vehicle_status_sub, 200);
	fds[5].fd = _vehicle_status_sub;
	fds[5].events = POLLIN;

	// initialize parameters
	parameters_update(true);

	while (!should_exit()) {

		// wait for up to 1000ms for data
		int pret = px4_poll(fds, (sizeof(fds) / sizeof(fds[0])), 1000);
		bool publishUpdate = false;

		if (pret == 0) {
			// Timeout: let the loop run anyway, don't do `continue` here

		} else if (pret < 0) {
			// this is undesirable but not much we can do
			PX4_ERR("poll error %d, %d", pret, errno);
			px4_usleep(50000);
			continue;

		} else {
			// Check for armed status
			if (fds[5].revents & POLLIN)
			{
				vehicle_status_s vehicle_status;
				orb_copy(ORB_ID(vehicle_status), _vehicle_status_sub, &vehicle_status);
				armed = vehicle_status.arming_state == vehicle_status_s::ARMING_STATE_ARMED;
				redundant_frame_configuration = vehicle_status.system_type == 13 // Hexacopter
								|| vehicle_status.system_type == 14; // Octacopter
			}

			// Check from battery_status
			if (fds[0].revents & POLLIN || fds[1].revents & POLLIN || fds[2].revents & POLLIN || fds[3].revents & POLLIN)
			{

				// clear total current
				total_current = 0.0f;

				for (unsigned i = 0; i < ORB_MULTI_MAX_INSTANCES; i++)
				{
					if (!(fds[i].revents & POLLIN))
					{
						// No update on this instance or not armed
						continue;
					}

					// Read from battery topics
					battery_status_s battery;
					orb_copy(ORB_ID(battery_status), _battery_subs[i], &battery);

					// sum current from this battery
					total_current += battery.current_filtered_a;

					if (!armed)
					{
						continue;
					}

					hrt_abstime now = hrt_absolute_time();

					// save time to delay for momentary voltage drop
					if (battery.remaining > _batt_crit_thr.get())
					{
						last_time_above_critical_threshold[i] = now;
					}

					if (battery.remaining > _batt_emergen_thr.get())
					{
						last_time_above_emergency_threshold[i] = now;
					}

					// Determine the warning level for this battery.
					uint8_t new_warning_state = determine_warning(warning_status[i],
										VOLTAGE_DROP_CHECK(now, last_time_above_critical_threshold[i]),
										VOLTAGE_DROP_CHECK(now, last_time_above_emergency_threshold[i]));

					// If the new warning is worst that the previous, update the warning.
					if (new_warning_state > warning_status[i])
					{
						// Update state
						warning_status[i] = new_warning_state;

						// Set as latest candidate
						update_candidate(warning_status[i],
								 battery.id,
								 battery.voltage_filtered_v,
								 battery.run_time_to_empty);
					}

				}

				publishUpdate = true;

			}

			// Check from battery_status_multi_pack
			if (fds[4].revents & POLLIN)
			{

				// Read from multi battery topics
				battery_status_multi_pack_s batteries;
				orb_copy(ORB_ID(battery_status_multi_pack), _battery_multi_pack_sub, &batteries);

				// clear total current for muti pack message
				total_current_multi_pack = 0.0f;

				for (unsigned i = 0; i < battery_status_multi_pack_s::MAX_BATTERY_PACK_COUNT; i++)
				{
					hrt_abstime now = hrt_absolute_time();

					// Check for power module connection status change
					if (!batteries.connected[i] && last_connected_state[i])
					{
						// Send power module disconnected notice
						mavlink_log_info(&_mavlink_log_pub, "Power module SN:%d disconnected.", batteries.serial_number[i]);
					}
					else if (batteries.connected[i] && !last_connected_state[i])
					{
						// Send power module connection notice
						mavlink_log_info(&_mavlink_log_pub, "Power module SN:%d connected.", batteries.serial_number[i]);
					}
					else
					{
						// No status change
					}

					// Update connection state
					last_connected_state[i] = batteries.connected[i];

					// If power module not connected, skip
					if (!batteries.connected[i])
					{
						continue;
					}

					// Check for battery connection status change
					float current_per_cell_voltage = batteries.voltage_v[i] / batteries.cell_count[i];

					if (last_battery_voltage_per_cell[i] < SINGLE_CELL_CONNECTION_VOLTAGE
					&& current_per_cell_voltage > SINGLE_CELL_CONNECTION_VOLTAGE)
					{
						// Send battery connected notice
						mavlink_log_info(&_mavlink_log_pub, "Battery on PM SN:%d connected.", batteries.serial_number[i]);
					}
					else if (last_battery_voltage_per_cell[i] > SINGLE_CELL_CONNECTION_VOLTAGE
					&& current_per_cell_voltage < SINGLE_CELL_CONNECTION_VOLTAGE)
					{
						// Send battery  disconnected notice
						mavlink_log_info(&_mavlink_log_pub, "Battery on PM SN:%d disconnected.", batteries.serial_number[i]);
					}
					else
					{
						// No status change
					}

					// Update last know voltage for battery connection report.
					last_battery_voltage_per_cell[i] = current_per_cell_voltage;

					// Add up total current
					total_current_multi_pack += batteries.current_a[i];

					// If not armed, skip
					if (!armed)
					{
						continue;
					}

					// save time to delay for momentary voltage drop
					if (batteries.voltage_v[i] > batteries.critical_voltage_v[i])
					{
						last_time_above_critical_threshold_multi_pack[i] = now;
					}

					if (batteries.voltage_v[i] > batteries.emergency_voltage_v[i])
					{
						last_time_above_emergency_threshold_multi_pack[i] = now;
					}

					// Determine the warning level for this battery.
					uint8_t new_warning_state = determine_warning(warning_status[i + ORB_MULTI_MAX_INSTANCES],
										VOLTAGE_DROP_CHECK(now, last_time_above_critical_threshold_multi_pack[i]),
										VOLTAGE_DROP_CHECK(now, last_time_above_emergency_threshold_multi_pack[i]));

					// If the new warning is worst that the previous, update the warning.
					if (new_warning_state > warning_status[i + ORB_MULTI_MAX_INSTANCES])
					{
						// Update state
						warning_status[i + ORB_MULTI_MAX_INSTANCES] = new_warning_state;

						// Set as latest candidate
						update_candidate(warning_status[i + ORB_MULTI_MAX_INSTANCES],
								 batteries.id[i],
								 batteries.voltage_v[i],
								 batteries.run_time_to_empty[i]);
					}
				}

				publishUpdate = true;

			}
		}

		// Determine and publish the information
		if (publishUpdate)
		{
			// Determine warning to publish
			advanced_failsafe_determine_warning();

			// Add timestamp and current
			failsafe_status.timestamp = hrt_absolute_time();
			failsafe_status.total_current_a = total_current + total_current_multi_pack;

			// Publish
			_battery_failsafe_pub.publish(failsafe_status);
		}

		parameters_update();
	}
}

void BatteryFailsafe::parameters_update(bool force)
{
	// check for parameter updates
	if (_parameter_update_sub.updated() || force) {
		// clear update
		parameter_update_s update;
		_parameter_update_sub.copy(&update);

		// update parameters from storage
		updateParams();
	}

	advanced_failsafe_enabled = _batt_fs_advanced_feature_enable.get();
}

int BatteryFailsafe::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
This module handles battery failsafe status advertisement.
This module does not take any argument.

### Examples
CLI usage example:
$ battery_failsafe start

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("battery_failsafe", "battery");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_PARAM_FLAG('f', "Optional example flag", true);
	PRINT_MODULE_USAGE_PARAM_INT('p', 0, 0, 1000, "Optional example parameter", true);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

int battery_failsafe_main(int argc, char *argv[])
{
	return BatteryFailsafe::main(argc, argv);
}

uint8_t BatteryFailsafe::determine_warning(uint8_t current_warning, bool should_warn_critical, bool should_warn_emergency)
{
	// propagate warning state only if the state is higher, otherwise remain in current warning state
	if (should_warn_emergency || (current_warning == battery_failsafe_s::BATTERY_WARNING_EMERGENCY)) {
		return battery_failsafe_s::BATTERY_WARNING_EMERGENCY;

	} else if (should_warn_critical || (current_warning == battery_failsafe_s::BATTERY_WARNING_CRITICAL)) {
		return battery_failsafe_s::BATTERY_WARNING_CRITICAL;

	} else {
		return battery_failsafe_s::BATTERY_WARNING_NONE;
	}
}

void BatteryFailsafe::advanced_failsafe_determine_warning()
{
	uint8_t critical_count = 0;
	uint8_t emergency_count = 0;

	// Count each warning
	for (uint8_t i = 0; i < ORB_MULTI_MAX_INSTANCES + battery_status_multi_pack_s::MAX_BATTERY_PACK_COUNT; i++)
	{
		if (warning_status[i] == battery_failsafe_s::BATTERY_WARNING_EMERGENCY)
		{
			emergency_count++;
		}
		else if (warning_status[i] == battery_failsafe_s::BATTERY_WARNING_CRITICAL)
		{
			critical_count++;
		}
	}

	if (advanced_failsafe_enabled && redundant_frame_configuration)
	{
		if (emergency_count > 1)
		{
			trigger_warning(battery_failsafe_s::BATTERY_WARNING_EMERGENCY);
		}

		else if (critical_count + emergency_count > 1)
		{
			trigger_warning(battery_failsafe_s::BATTERY_WARNING_CRITICAL);
		}
	}

	else
	{
		if (emergency_count > 0)
		{
			trigger_warning(battery_failsafe_s::BATTERY_WARNING_EMERGENCY);
		}

		else if (critical_count > 0)
		{
			trigger_warning(battery_failsafe_s::BATTERY_WARNING_CRITICAL);
		}
	}

	return;
}

void BatteryFailsafe::trigger_warning(uint8_t warning_level)
{
	// only update if the new warning is higher than current warning
	if (warning_level > failsafe_status.warning)
	{
		failsafe_status.warning = warning_level;

		switch (warning_level)
		{
			case battery_failsafe_s::BATTERY_WARNING_CRITICAL:
				failsafe_status.source_id = candidate[0].id;
				failsafe_status.voltage_v = candidate[0].voltage_v;
				failsafe_status.run_time_to_empty = candidate[0].run_time_to_empty;
				break;

			case battery_failsafe_s::BATTERY_WARNING_EMERGENCY:
				failsafe_status.source_id = candidate[1].id;
				failsafe_status.voltage_v = candidate[1].voltage_v;
				failsafe_status.run_time_to_empty = candidate[1].run_time_to_empty;
				break;

			default:
				break;
		}
	}

	return;
}

void BatteryFailsafe::update_candidate(uint8_t warning_level, uint8_t id, float voltage_v, uint16_t run_time_to_empty)
{
	// update latest candidate for each level
	switch (warning_level)
	{
		case battery_failsafe_s::BATTERY_WARNING_CRITICAL:
			candidate[0].id = id;
			candidate[0].voltage_v = voltage_v;
			candidate[0].run_time_to_empty = run_time_to_empty;
			break;

		case battery_failsafe_s::BATTERY_WARNING_EMERGENCY:
			candidate[1].id = id;
			candidate[1].voltage_v = voltage_v;
			candidate[1].run_time_to_empty = run_time_to_empty;
			break;

		default:
			break;
	}

	return;
}
