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

#pragma once

#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <systemlib/mavlink_log.h>
#include <mathlib/mathlib.h>
#include <perf/perf_counter.h>
#include <uORB/Subscription.hpp>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/battery_status.h>
#include <uORB/topics/battery_status_multi_pack.h>

#include <uORB/Publication.hpp>
#include <uORB/topics/battery_failsafe.h>

extern "C" __EXPORT int battery_failsafe_main(int argc, char *argv[]);

#define VOLTAGE_DROP_DELAY_US 5000000


class BatteryFailsafe : public ModuleBase<BatteryFailsafe>, public ModuleParams
{
public:
	BatteryFailsafe(int example_param, bool example_flag);

	virtual ~BatteryFailsafe() = default;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static BatteryFailsafe *instantiate(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	/** @see ModuleBase::run() */
	void run() override;

	/** @see ModuleBase::print_status() */
	int print_status() override;

private:

	/**
	 * Check for parameter changes and update them if needed.
	 * @param parameter_update_sub uorb subscription to parameter_update
	 * @param force for a parameter update
	 */
	void parameters_update(bool force = false);

	uint8_t determineWarning(uint8_t currentWarning, bool should_warn_critical, bool should_warn_emergency);

	battery_failsafe_s failsafe_status;

	bool last_connected_state[battery_status_multi_pack_s::MAX_BATTERY_PACK_COUNT] = {false};

	hrt_abstime last_time_above_critical_threshold[ORB_MULTI_MAX_INSTANCES];
	hrt_abstime last_time_above_emergency_threshold[ORB_MULTI_MAX_INSTANCES];

	hrt_abstime last_time_above_critical_threshold_multi_pack[battery_status_multi_pack_s::MAX_BATTERY_PACK_COUNT];
	hrt_abstime last_time_above_emergency_threshold_multi_pack[battery_status_multi_pack_s::MAX_BATTERY_PACK_COUNT];

	DEFINE_PARAMETERS(
		(ParamInt<px4::params::BFS_ENABLED>) _bfs_enabled,
		(ParamFloat<px4::params::BAT_LOW_THR>) _batt_low_thr,
		(ParamFloat<px4::params::BAT_CRIT_THR>) _batt_crit_thr,
		(ParamFloat<px4::params::BAT_EMERGEN_THR>) _batt_emergen_thr
	)

	// Subscriber
	uORB::Subscription	_parameter_update_sub{ORB_ID(parameter_update)};

	// Publisher
	uORB::Publication<battery_failsafe_s>	_battery_failsafe_pub{ORB_ID(battery_failsafe)};
	orb_advert_t _mavlink_log_pub{nullptr};

};

