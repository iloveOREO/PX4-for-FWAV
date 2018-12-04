/****************************************************************************
 *
 *   Copyright (c) 2013, 2014 PX4 Development Team. All rights reserved.
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
* @file px4_uorb_adver.c
*/
#include <stdio.h>
#include <termios.h>
#include <unistd.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <errno.h>
#include <drivers/drv_hrt.h>
#include <systemlib/err.h>
#include <fcntl.h>
#include <systemlib/mavlink_log.h>
#include <uORB/uORB.h>
#include <uORB/topics/test_uorb.h>
#include <px4_defines.h>
#include <px4_config.h>
#include <px4_posix.h>
#include <px4_shutdown.h>
#include <px4_tasks.h>
#include <px4_time.h>

static bool thread_should_exit = false;         /**< daemon exit flag */
static bool thread_running = false;             /**< daemon status flag */
static int px4_uorb_adver_task;                 /**< Handle of daemon task / thread */

/**
 * daemon management function.
 */
__EXPORT int px4_uorb_adver_main(int argc, char *argv[]);

/**
 * Mainloop of daemon.
 */
int px4_uorb_adver_thread_main(int argc, char *argv[]);

/**
 * Print the correct usage.
 */
static void usage(const char *reason);

static void
usage(const char *reason)
{
    if(reason) {
        PX4_WARN("%s\n", reason);
    }

    PX4_WARN("usage: px4_uorb_adver {start|stop|status} [-p <additional param>]\n\n");
}

/**
 * 消息发布进程，会不断的发送自定义的消息
 */
int px4_uorb_adver_main(int argc, char *argv[])
{
    if (argc < 2) {
        usage("missing command");
        return 1;
    }

    if (!strcmp(argv[1], "start")) {

        if (thread_running) {
            PX4_WARN("daemon already running\n");
            return 0;
        }

        thread_should_exit = false; //定义一个守护进程
        px4_uorb_adver_task = px4_task_spawn_cmd("px4_uorb_adver",
                        SCHED_DEFAULT,
                        SCHED_PRIORITY_DEFAULT, //调度优先级
                        2000, //堆栈分配大小
                        px4_uorb_adver_thread_main,
                        (argv) ? (char *const *)&argv[2] : (char *const *)NULL);
        return 0; 
    }

    if (!strcmp(argv[1], "stop")) {
        thread_should_exit = true;
        return 0;
    }

    if (!strcmp(argv[1], "status")) {
        if (thread_running){
            PX4_WARN("\trunning\n");
        } else {
            PX4_WARN("\tnot started\n");
        }

        return 0;
    }

    usage("unrecongnized command");
    return 1;
}

int px4_uorb_adver_thread_main(int argc, char *argv[])
{
    struct test_uorb_s test_uorb_ad;
    memset(&test_uorb_ad, 0, sizeof(test_uorb_ad));
    orb_advert_t test_pub = orb_advertise(ORB_ID(test_uorb), &test_uorb_ad);
    PX4_WARN("[daemon] starting\n");

    thread_running = true;
    while (!thread_should_exit) {
        test_uorb_ad.test1 = 5;
        test_uorb_ad.test2 = 6;
        test_uorb_ad.test3 = 7;
        orb_publish(ORB_ID(test_uorb), test_pub, &test_uorb_ad);
        usleep(1000);
    }

    PX4_WARN("[daemon] exiting.\n");
    thread_running = false;

    return 0;
}