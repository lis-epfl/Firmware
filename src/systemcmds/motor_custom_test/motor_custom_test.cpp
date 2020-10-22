/****************************************************************************
 *
 *   Copyright (c) 2016 PX4 Development Team. All rights reserved.
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

/**
 * @file motor_custom_test.cpp
 *
 * @author Leonardo Cencetti <cencetti.leonardo@gmail.com>
 */

#include <px4_config.h>
#include <px4_defines.h>
#include <px4_module.h>
#include <px4_tasks.h>
#include <px4_posix.h>
#include <px4_getopt.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <math.h>
#include <poll.h>

#include <arch/board/board.h>
#include <drivers/drv_hrt.h>
#include <drivers/drv_pwm_output.h>
#include <platforms/px4_defines.h>

#include "systemlib/err.h"
#include "uORB/topics/actuator_controls.h"

enum TestState {
	TEST_INIT,
	TEST_START,
	TEST_RUNNING,
	TEST_WAIT
};


static bool _thread_should_exit = false;		/**< motor_custom_test exit flag */
static bool _thread_running = false;		/**< motor_custom_test status flag */
static int _motor_custom_test_task;				/**< Handle of motor_custom_test task / thread */
static float _test_time;
static int _pwm_setpoint;
static int _max_pwm;
static int _channel;
static const char *_pwm_output_dev = "/dev/pwm_output0";

/**
 * motor_custom_test management function.
 */
extern "C" __EXPORT int motor_custom_test_main(int argc, char *argv[]);

/**
 * Mainloop of motor_custom_test.
 */
int motor_custom_test_thread_main(int argc, char *argv[]);

bool pwm_valid(int pwm_value);

int set_min_pwm_(int fd, unsigned long max_channels, int pwm_value);

int set_out(int fd, int channel, unsigned long max_channels, int pwm);

int prepare_(int fd, unsigned long *max_channels);

/**
 * Print the correct usage.
 */
static void usage(const char *reason);

static void
usage(const char *reason)
{
	if (reason) {
		PX4_ERR("%s", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Application to test motor ramp up.

Before starting, make sure to stop any running attitude controller:
$ mc_att_control stop
$ fw_att_control stop

When starting, a background task is started, runs for several seconds (as specified), then exits.

### Example
$ motor_custom_test -m 1 -p 1100 -t 0.5
)DESCR_STR");


	PRINT_MODULE_USAGE_NAME_SIMPLE("motor_custom_test", "command");
	PRINT_MODULE_USAGE_PARAM_STRING('d', "/dev/pwm_output0", nullptr, "Pwm output device", true);
	PRINT_MODULE_USAGE_PARAM_INT('m', -1, 0, 7, "Motor to test (0...7, all if not specified)", true);
	PRINT_MODULE_USAGE_PARAM_INT('p', 0, 900, 2000, "Select pwm duty cycle in usec", false);
	PRINT_MODULE_USAGE_PARAM_FLOAT('t', 1.0f, 0.0f, 65536.0f, "Select motor test duration in sec", true);

	PRINT_MODULE_USAGE_PARAM_COMMENT("WARNING: motors will spin up to speed!");

}

/**
 * The motor_custom_test app only briefly exists to start
 * the background job. The stack size assigned in the
 * Makefile does only apply to this management task.
 *
 * The actual stack size should be set in the call
 * to task_create().
 */
int motor_custom_test_main(int argc, char *argv[])
{

	int myoptind = 1;
	int ch;
	const char *myoptarg = nullptr;
	bool error_flag = false;
	bool set_pwm = false;
	_max_pwm = 2000;
	_test_time = 1.0f;
	_channel = -1;

	if (_thread_running) {
		PX4_WARN("motor_custom_test already running\n");
		/* this is not an error */
		return 0;
	}

	if (argc < 3) {
		usage("missing parameters");
		return 1;
	}

	while ((ch = px4_getopt(argc, argv, "d:m:p:t:", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {

			case 'd':
				if(!strcmp(myoptarg, "/dev/pwm_output0") || !strcmp(myoptarg, "/dev/pwm_output1")){
					_pwm_output_dev = myoptarg;
				} else {
					usage("pwm output device not found");
					error_flag = true;
				}
				break;

			case 'p':
				_pwm_setpoint = atoi(myoptarg);

				if (!pwm_valid(_pwm_setpoint)) {
					usage("PWM not in range");
					error_flag = true;
				} else {
					set_pwm = true;
				}

				break;

			case 'm':
				_channel = (int)strtol(myoptarg, NULL, 0);

				break;

			case 't':
				_test_time = atof(myoptarg);

				if (_test_time <= 0) {
					usage("test time must be greater than 0");
					error_flag = true;
				}

				break;

			default:
				PX4_WARN("unrecognized flag");
				error_flag = true;
				break;
		}
	}

	_thread_should_exit = false;

	if(!set_pwm){
		PX4_WARN("pwm not set. use -p flag");
		error_flag = true;
	}

	if(error_flag){
		return 1;
	}

	_motor_custom_test_task = px4_task_spawn_cmd("motor_custom_test",
					      SCHED_DEFAULT,
					      SCHED_PRIORITY_DEFAULT + 40,
					      2000,
					      motor_custom_test_thread_main,
					      (argv) ? (char *const *)&argv[2] : (char *const *)nullptr);

	return 0;
}

bool pwm_valid(int pwm_value)
{
	return pwm_value >= 900 && pwm_value <= 2100;
}

int set_min_pwm_(int fd, unsigned long max_channels, int pwm_value)
{
	int ret;

	struct pwm_output_values pwm_values {};

	pwm_values.channel_count = max_channels;

	for (unsigned i = 0; i < max_channels; i++) {
		pwm_values.values[i] = pwm_value;
	}

	ret = px4_ioctl(fd, PWM_SERVO_SET_MIN_PWM, (long unsigned int)&pwm_values);

	if (ret != OK) {
		PX4_ERR("failed setting min values");
		return 1;
	}

	return 0;
}

int set_out(int fd, int channel, unsigned long max_channels, int pwm)
{
	int ret;

	for (unsigned i = 0; i < max_channels; i++) {
		if (channel != -1) {
			ret = ioctl(fd, PWM_SERVO_SET(i), (i == (unsigned)channel) ? pwm : 0);
		} else {
			ret = ioctl(fd, PWM_SERVO_SET(i), pwm);
		}

		if (ret != OK) {
			PX4_ERR("PWM_SERVO_SET(%d), value: %d", i, pwm);
			return 1;
		}
	}

	return 0;
}

int prepare_(int fd, unsigned long *max_channels)
{
	/* make sure no other source is publishing control values now */
	struct actuator_controls_s actuators;
	int act_sub = orb_subscribe(ORB_ID_VEHICLE_ATTITUDE_CONTROLS);

	/* clear changed flag */
	orb_copy(ORB_ID_VEHICLE_ATTITUDE_CONTROLS, act_sub, &actuators);

	/* wait 50 ms */
	px4_usleep(50000);

	/* now expect nothing changed on that topic */
	bool orb_updated;
	orb_check(act_sub, &orb_updated);

	if (orb_updated) {
		PX4_ERR("ABORTING! Attitude control still active. Please ensure to shut down all controllers:\n"
			"\tmc_att_control stop\n"
			"\tfw_att_control stop\n");
		return 1;
	}

	/* get number of channels available on the device */
	if (px4_ioctl(fd, PWM_SERVO_GET_COUNT, (unsigned long)max_channels) != OK) {
		PX4_ERR("PWM_SERVO_GET_COUNT");
		return 1;
	}

	/* tell IO/FMU that its ok to disable its safety with the switch */
	if (px4_ioctl(fd, PWM_SERVO_SET_ARM_OK, 0) != OK) {
		PX4_ERR("PWM_SERVO_SET_ARM_OK");
		return 1;
	}

	/* tell IO/FMU that the system is armed (it will output values if safety is off) */
	if (px4_ioctl(fd, PWM_SERVO_ARM, 0) != OK) {
		PX4_ERR("PWM_SERVO_ARM");
		return 1;
	}

	/* tell IO to switch off safety without using the safety switch */
	if (px4_ioctl(fd, PWM_SERVO_SET_FORCE_SAFETY_OFF, 0) != OK) {
		PX4_ERR("PWM_SERVO_SET_FORCE_SAFETY_OFF");
		return 1;
	}

	return 0;
}

int motor_custom_test_thread_main(int argc, char *argv[])
{
	_thread_running = true;

	unsigned long max_channels = 0;
	static struct pwm_output_values last_spos;
	static struct pwm_output_values last_min;
	static unsigned servo_count;

	int fd = px4_open(_pwm_output_dev, 0);

	if (fd < 0) {
		PX4_ERR("Can't open %s", _pwm_output_dev);
		_thread_running = false;
		return 1;
	}

	/* get the number of servo channels */
	if (px4_ioctl(fd, PWM_SERVO_GET_COUNT, (unsigned long)&servo_count) < 0) {
			PX4_ERR("PWM_SERVO_GET_COUNT");
			px4_close(fd);
			_thread_running = false;
			return 1;

	}

	/* get current servo values */
	for (unsigned i = 0; i < servo_count; i++) {

		if (px4_ioctl(fd, PWM_SERVO_GET(i), (unsigned long)&last_spos.values[i]) < 0) {
			PX4_ERR("PWM_SERVO_GET(%d)", i);
			px4_close(fd);
			_thread_running = false;
			return 1;
		}
	}

	/* get current pwm min */
	if (px4_ioctl(fd, PWM_SERVO_GET_MIN_PWM, (long unsigned int)&last_min) < 0) {
		PX4_ERR("Failed to get pwm min values");
		px4_close(fd);
		_thread_running = false;
		return 1;
	}

	if (px4_ioctl(fd, PWM_SERVO_SET_MODE, PWM_SERVO_ENTER_TEST_MODE) < 0) {
		PX4_ERR("Failed to enter pwm test mode");
		px4_close(fd);
		_thread_running = false;
		return 1;
	}

	if (prepare_(fd, &max_channels) != OK) {
		_thread_should_exit = true;
	}

	set_out(fd, -1, max_channels, 0.0f);

	float timer = 0.0f;
	hrt_abstime start = 0;

	enum TestState test_state = TEST_INIT;
	int output = 0;

	start = hrt_absolute_time();

	while (!_thread_should_exit) {

		timer = hrt_elapsed_time(&start) * 1e-6; // convert to seconds

		switch (test_state) {
			case TEST_INIT: {
					PX4_INFO("setting pwm min: %d", _pwm_setpoint);
					set_min_pwm_(fd, max_channels, _pwm_setpoint);
					test_state = TEST_START;
					break;
				}

			case TEST_START: {
					if (timer > 3.0f) {
						PX4_INFO("starting test: %.2f sec", (double)_test_time);
						start = hrt_absolute_time();
						test_state = TEST_RUNNING;
					}

					set_out(fd, -1, max_channels, output);
					break;
				}

			case TEST_RUNNING: {
					output = (int)_pwm_setpoint;

					if  (timer > 3.0f * _test_time) {
						start = hrt_absolute_time();
						test_state = TEST_WAIT;
						PX4_INFO("Test finished, waiting");
					}

					set_out(fd, _channel, max_channels, output);
					break;
				}

			case TEST_WAIT: {
					if (timer > 0.5f) {
						_thread_should_exit = true;
						PX4_INFO("stopping");
						break;
					}

					set_out(fd, _channel, max_channels, output);
					break;
				}
		}

		// rate limit
		px4_usleep(2000);
	}

	if (fd >= 0) {
		/* set current pwm min */
		if (px4_ioctl(fd, PWM_SERVO_SET_MIN_PWM, (long unsigned int)&last_min) < 0) {
			PX4_ERR("failed setting pwm min values");
			px4_close(fd);
			_thread_running = false;
			return 1;
		}

		/* set previous servo values */
		for (unsigned i = 0; i < servo_count; i++) {

			if (px4_ioctl(fd, PWM_SERVO_SET(i), (unsigned long)last_spos.values[i]) < 0) {
				PX4_ERR("PWM_SERVO_SET(%d)", i);
				px4_close(fd);
				_thread_running = false;
				return 1;
			}
		}

		if (px4_ioctl(fd, PWM_SERVO_SET_MODE, PWM_SERVO_EXIT_TEST_MODE) < 0) {
			PX4_ERR("Failed to Exit pwm test mode");
			px4_close(fd);
			_thread_running = false;
			return 1;
		}

		px4_close(fd);
	}

	_thread_running = false;

	return 0;
}
