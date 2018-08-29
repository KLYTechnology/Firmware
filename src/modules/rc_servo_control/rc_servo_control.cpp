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

#include "rc_servo_control.h"

#include <px4_getopt.h>
#include <px4_log.h>
#include <px4_posix.h>

#include <uORB/topics/parameter_update.h>
#include <uORB/topics/rc_channels.h>
#include <uORB/topics/input_rc.h>
#include <uORB/topics/actuator_controls.h>


int RCServoControl::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
This module hooks rc_channels topic to actuator_controls topic. When running this module,
it enables direct control of wing servos through RC channels 11 12 15 16.

### Examples
CLI usage example:
$ rc_servo_control start

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("rc_servo_control", "controller");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

int RCServoControl::print_status()
{
	PX4_INFO("Running");
	// TODO: print additional runtime information about the state of the module

	return 0;
}

int RCServoControl::custom_command(int argc, char *argv[])
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


int RCServoControl::task_spawn(int argc, char *argv[])
{
	_task_id = px4_task_spawn_cmd("rc_servo_control",
				      SCHED_DEFAULT,
				      SCHED_PRIORITY_DEFAULT,
				      1024,
				      (px4_main_t)&run_trampoline,
				      (char *const *)argv);

	if (_task_id < 0) {
		_task_id = -1;
		return -errno;
	}

	return 0;
}

RCServoControl *RCServoControl::instantiate(int argc, char *argv[])
{
	return new RCServoControl(0, true);
}

RCServoControl::RCServoControl(int example_param, bool example_flag)
	: ModuleParams(nullptr)
{
}

void RCServoControl::run()
{
	// Example: run the loop synchronized to the rc_channels topic publication
	int rc_channels_sub = orb_subscribe(ORB_ID(rc_channels));

	/* advertise actuator controls topic */
	struct actuator_controls_s act_ctl;
	memset(&act_ctl, 0, sizeof(act_ctl));
	orb_advert_t act_ctl_pub = orb_advertise(ORB_ID(actuator_controls_1), &act_ctl);

	px4_pollfd_struct_t fds[1];
	fds[0].fd = rc_channels_sub;
	fds[0].events = POLLIN;

	// initialize parameters
	int parameter_update_sub = orb_subscribe(ORB_ID(parameter_update));
	parameters_update(parameter_update_sub, true);

	while (!should_exit()) {

		// wait for up to 1000ms for data
		int pret = px4_poll(fds, (sizeof(fds) / sizeof(fds[0])), 1000);

		if (pret == 0) {
			// Timeout: let the loop run anyway, don't do `continue` here

		} else if (pret < 0) {
			// this is undesirable but not much we can do
			PX4_ERR("poll error %d, %d", pret, errno);
			usleep(50000);
			continue;

		} else if (fds[0].revents & POLLIN) {

			/* obtained data for the first file descriptor */
			struct rc_channels_s rc_channels_raw;
			/* copy sensors raw data into local buffer */
			orb_copy(ORB_ID(rc_channels), rc_channels_sub, &rc_channels_raw);
			/*
            PX4_INFO("Accelerometer:\t%8.4f\t%8.4f\t%8.4f",
                     (double)raw.accelerometer_m_s2[0],
                     (double)raw.accelerometer_m_s2[1],
                     (double)raw.accelerometer_m_s2[2]);
            */

			/* directly pass rc_channels data to actuator_controls data */
			act_ctl.control[4] = rc_channels_raw.channels[10];  // channel 11
			act_ctl.control[5] = rc_channels_raw.channels[11];  // channel 12
			act_ctl.control[6] = rc_channels_raw.channels[14];  // channel 15
			act_ctl.control[7] = rc_channels_raw.channels[15];  // channel 16

			orb_publish(ORB_ID(actuator_controls_1), act_ctl_pub, &act_ctl);

		}


		parameters_update(parameter_update_sub);
	}

	orb_unsubscribe(rc_channels_sub);
	orb_unsubscribe(parameter_update_sub);
}

void RCServoControl::parameters_update(int parameter_update_sub, bool force)
{
	bool updated;
	struct parameter_update_s param_upd;

	orb_check(parameter_update_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(parameter_update), parameter_update_sub, &param_upd);
	}

	if (force || updated) {
		updateParams();
	}
}


int rc_servo_control_main(int argc, char *argv[])
{
	return RCServoControl::main(argc, argv);
}
