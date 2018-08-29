/****************************************************************************
 *
 *   Copyright (c) 2012-2016 PX4 Development Team. All rights reserved.
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
 * @file px4_rc_servo_control.cpp
 * Used for KLY Delta X3 drone. When running this program
 * RC channels will directly control the servos for tilting wings
 *
 * @author Hank Yang <hankyang94@gmail.com>
 */

#include <px4_log.h>
#include <px4_config.h>
#include <px4_tasks.h>
#include <px4_posix.h>
#include <unistd.h>
#include <stdio.h>
#include <poll.h>
#include <string.h>
#include <math.h>

#include <uORB/uORB.h>
#include <uORB/topics/rc_channels.h>
#include <uORB/topics/input_rc.h>
#include <uORB/topics/actuator_controls.h>

__EXPORT int px4_rc_servo_control_main(int argc, char *argv[]);

int px4_rc_servo_control_main(int argc, char *argv[])
{
    PX4_INFO("RC channels 11 12 15 16 will control the servos!");

    /* subscribe to rc_channels topic */
    int sensor_sub_fd = orb_subscribe(ORB_ID(rc_channels));
    /* limit the update rate to 5 Hz */
    orb_set_interval(sensor_sub_fd, 100);

    /* advertise actuator controls topic */
    struct actuator_controls_s act_ctl;
    memset(&act_ctl, 0, sizeof(act_ctl));
    orb_advert_t act_ctl_pub = orb_advertise(ORB_ID(actuator_controls_1), &act_ctl);

    /* one could wait for multiple topics with this technique, just using one here */
    px4_pollfd_struct_t fds[] = {
            { .fd = sensor_sub_fd,  .events = POLLIN },
    };
            /* there could be more file descriptors here, in the form like:
             * { .fd = other_sub_fd,   .events = POLLIN },
             */

    int error_counter = 0;

    while (true) {
        /* wait for sensor update of 1 file descriptor for 1000 ms (1 second) */
        int poll_ret = px4_poll(fds, 1, 1000);

        /* handle the poll result */
        if (poll_ret == 0) {
            /* this means none of our providers is giving us data */
            PX4_ERR("Got no data within a second");

        } else if (poll_ret < 0) {
            /* this is seriously bad - should be an emergency */
            if (error_counter < 10 || error_counter % 50 == 0) {
                /* use a counter to prevent flooding (and slowing us down) */
                PX4_ERR("ERROR return value from poll(): %d", poll_ret);
            }

            error_counter++;

        } else {

            // PX4_INFO("%d", poll_ret);

            if (fds[0].revents & POLLIN) {
                /* obtained data for the first file descriptor */
                struct rc_channels_s rc_channels_raw;
                /* copy sensors raw data into local buffer */
                orb_copy(ORB_ID(rc_channels), sensor_sub_fd, &rc_channels_raw);
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

            /* there could be more file descriptors here, in the form like:
             * if (fds[1..n].revents & POLLIN) {}
             */
        }
    }

    PX4_INFO("exiting");

    return 0;
}

