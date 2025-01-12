/*
 * Copyright (c) 2015 Carlos Pizano-Uribe  cpu@chromium.org
 *
 * Use of this source code is governed by a MIT-style
 * license that can be found in the LICENSE file or at
 * https://opensource.org/licenses/MIT
 */

#include <lk/debug.h>
#include <lk/err.h>
#include <rand.h>
#include <string.h>
#include <lk/trace.h>

#include <kernel/event.h>
#include <kernel/port.h>
#include <kernel/thread.h>

#include <platform.h>

#include <app/tests.h>

#define LOCAL_TRACE 0

void *context1 = (void *) 0x53;

static void dump_port_result(const port_result_t *result) {
    const port_packet_t *p = &result->packet;
    LTRACEF("[%02x %02x %02x %02x %02x %02x %02x %02x]\n",
            p->value[0], p->value[1], p->value[2], p->value[3],
            p->value[4], p->value[5], p->value[6], p->value[7]);
}

static int single_thread_basic(void) {
    port_t w_port;
    status_t st = port_create("sh_prt1", PORT_MODE_UNICAST, &w_port);
    if (st < 0) {
        printf("could not create port, status = %d\n", st);
        return __LINE__;
    }

    port_t r_port;
    st = port_open("sh_prt0", context1, &r_port);
    if (st != ERR_NOT_FOUND) {
        printf("expected not to find port, status = %d\n", st);
        return __LINE__;
    }

    st = port_open("sh_prt1", context1, &r_port);
    if (st < 0) {
        printf("could not open port, status = %d\n", st);
        return __LINE__;
    }

    port_packet_t packet[3] = {
        {{0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08}},
        {{0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f, 0x10, 0x11}},
        {{0x33, 0x66, 0x99, 0xcc, 0x33, 0x66, 0x99, 0xcc}},
    };

    st = port_write(w_port, &packet[0], 1);
    if (st < 0) {
        printf("could not write port, status = %d\n", st);
        return __LINE__;
    }

    printf("reading from port:\n");

    port_result_t res = {0};

    st = port_read(r_port, 0, &res);
    if (st < 0) {
        printf("could not read port, status = %d\n", st);
        return __LINE__;
    }
    if (res.ctx != context1) {
        printf("bad context! = %p\n", res.ctx);
        return __LINE__;
    }

    st = port_read(r_port, 0, &res);
    if (st != ERR_TIMED_OUT) {
        printf("expected timeout, status = %d\n", st);
        return __LINE__;
    }

    st = port_write(w_port, &packet[1], 1);
    if (st < 0) {
        printf("could not write port, status = %d\n", st);
        return __LINE__;
    }

    st = port_write(w_port, &packet[0], 1);
    if (st < 0) {
        printf("could not write port, status = %d\n", st);
        return __LINE__;
    }

    st = port_write(w_port, &packet[2], 1);
    if (st < 0) {
        printf("could not write port, status = %d\n", st);
        return __LINE__;
    }

    int expected_count = 3;
    while (true) {
        st = port_read(r_port, 0, &res);
        if (st < 0)
            break;
        dump_port_result(&res);
        --expected_count;
    }

    if (expected_count != 0) {
        printf("invalid read count = %d\n", expected_count);
        return __LINE__;
    }

    printf("\n");

    // port should be empty. should be able to write 8 packets.
    expected_count = 8;
    while (true) {
        st = port_write(w_port, &packet[1], 1);
        if (st < 0)
            break;
        --expected_count;
        st = port_write(w_port, &packet[2], 1);
        if (st < 0)
            break;
        --expected_count;
    }

    if (expected_count != 0) {
        printf("invalid write count = %d\n", expected_count);
        return __LINE__;
    }

    // tod(cpu) fix this possibly wrong error.
    if (st != ERR_PARTIAL_WRITE) {
        printf("expected buffer error, status =%d\n", st);
        return __LINE__;
    }

    // read 3 packets.
    for (int ix = 0; ix != 3; ++ix) {
        st = port_read(r_port, 0, &res);
        if (st < 0) {
            printf("could not read port, status = %d\n", st);
            return __LINE__;
        }
    }

    // there are 5 packets, now we add another 3.
    st = port_write(w_port, packet, 3);
    if (st < 0) {
        printf("could not write port, status = %d\n", st);
        return __LINE__;
    }

    expected_count = 8;
    while (true) {
        st = port_read(r_port, 0, &res);
        if (st < 0)
            break;
        dump_port_result(&res);
        --expected_count;
    }

    if (expected_count != 0) {
        printf("invalid read count = %d\n", expected_count);
        return __LINE__;
    }

    // attempt to use the wrong port.
    st = port_write(r_port, &packet[1], 1);
    if (st !=  ERR_BAD_HANDLE) {
        printf("expected bad handle error, status = %d\n", st);
        return __LINE__;
    }

    st = port_read(w_port, 0, &res);
    if (st !=  ERR_BAD_HANDLE) {
        printf("expected bad handle error, status = %d\n", st);
        return __LINE__;
    }

    st = port_close(r_port);
    if (st < 0) {
        printf("could not close read port, status = %d\n", st);
        return __LINE__;
    }

    st = port_close(w_port);
    if (st < 0) {
        printf("could not close write port, status = %d\n", st);
        return __LINE__;
    }

    st = port_close(r_port);
    if (st != ERR_BAD_HANDLE) {
        printf("expected bad handle error, status = %d\n", st);
        return __LINE__;
    }

    st = port_close(w_port);
    if (st != ERR_BAD_HANDLE) {
        printf("expected bad handle error, status = %d\n", st);
        return __LINE__;
    }

    st = port_destroy(w_port);
    if (st < 0) {
        printf("could not destroy port, status = %d\n", st);
        return __LINE__;
    }

    printf("single_thread_basic : ok\n");
    return 0;
}

static int ping_pong_thread(void *arg) {
    port_t r_port;
    status_t st = port_open("ping_port", NULL, &r_port);
    if (st < 0) {
        printf("thread: could not open port, status = %d\n", st);
        return __LINE__;
    }

    bool should_dispose_pong_port = true;
    port_t w_port;
    st = port_create("pong_port", PORT_MODE_UNICAST, &w_port);
    if (st == ERR_ALREADY_EXISTS) {
        // won the race to create the port.
        should_dispose_pong_port = false;
    } else if (st < 0) {
        printf("thread: could not open port, status = %d\n", st);
        return __LINE__;
    }

    port_result_t pr;

    // the loop is read-mutate-write until the write port
    // is closed by the master thread.
    while (true) {
        st = port_read(r_port, INFINITE_TIME, &pr);

        if (st == ERR_CANCELLED) {
            break;
        } else if (st < 0) {
            printf("thread: could not read port, status = %d\n", st);
            return __LINE__;
        }

        pr.packet.value[0]++;
        pr.packet.value[5]--;

        st = port_write(w_port, &pr.packet, 1);
        if (st < 0) {
            printf("thread: could not write port, status = %d\n", st);
            return __LINE__;
        }
    }

    port_close(r_port);

    if (should_dispose_pong_port) {
        port_close(w_port);
        port_destroy(w_port);
    }

    return 0;

bail:
    return __LINE__;
}

static const char *kStatusPortNames[] = {
  "status0",
  "status1",
};
static const char *kRacePortName = "racer_port";

static const port_packet_t kRepeat = 
        {{'R', 'E', 'P', 'E', 'A', 'T', 0, 0}};
static const port_packet_t kQuit =
        {{'Q', 'U', 'I', 'T', 0, 0, 0, 0}};

event_t race_evt;

static int race_thread(void *arg)
{
    port_t r_port;
    int tid = (int)arg;

    printf("thread %d: connecting to control port\n", tid);
    status_t st = port_open("race_ctl", NULL, &r_port);
    if (st < 0) {
        printf("thread %d: could not open control port, status = %d\n", tid, st);
        return __LINE__;
    }

    printf("thread %d: creating status port\n", tid);
    port_t w_port;
    st = port_create(kStatusPortNames[tid], PORT_MODE_UNICAST, &w_port);
    if (st < 0) {
        printf("thread %d: could not create status port, status = %d\n", tid, st);
        port_close(r_port);
        return __LINE__;
    }

    // Loop is meant to coordinate a port_create() race.
    // The event triggers the race.
    // The thread sleeps briefly then cleans up
    // The thread reports its claim to its status port.
    // It then waits for a repeat or quit message.
    int ret = -1;
    while (ret < 0) {
        LTRACEF_LEVEL(1, "thread %d: waiting at the starting line\n", tid);
        if (event_wait_timeout(&race_evt, INFINITE_TIME) != NO_ERROR) {
            ret = __LINE__;
            break;
        }

        port_t race_port;
        while(true) {
            st = port_create(kRacePortName, PORT_MODE_UNICAST, &race_port);
            if (st != ERR_BUSY)
                break;
            thread_sleep(25);
        } // EINTR all over again . . .
        LTRACEF_LEVEL(1, "thread %d: sampling chronochip (%x)\n", tid, race_port);
        if (st == ERR_ALREADY_EXISTS) {
            // lost the race to create the port.
        } else if (st < 0) {
            LTRACEF_LEVEL(1, "thread %d: could not open port, status = %d\n", tid, st);
            ret = __LINE__;
            break;
        } else { // Dispose of it now.
            thread_sleep(25);
            port_close(race_port);
            port_destroy(race_port);
        }

        // Now send the stale pointer address as a status.
        port_packet_t claimed_port = {{0}};
        int len = sizeof(claimed_port.value);
        if (sizeof(race_port) < (size_t)len)
            len = sizeof(race_port);
        for (int i = 0; i < len; ++i) {
            claimed_port.value[i] = 0xff & ((int)race_port) >> (i * 8);
        }
        LTRACEF_LEVEL(1, "thread %d: reporting status\n", tid);
        st = port_write(w_port, &claimed_port, 1);
        if (st < 0) {
            printf("thread %d: could not write port, status = %d\n", tid, st);
            ret = __LINE__;
            break;
        }

        LTRACEF_LEVEL(1, "thread %d: awaiting instructions\n", tid);
        port_result_t pr;
        st = port_read(r_port, INFINITE_TIME, &pr);
        if (st == ERR_CANCELLED) {
            printf("thread %d: could not read port, status = %d (CANCELLED)\n", tid, st);
            ret = __LINE__;
            break;
        } else if (st < 0) {
            printf("thread %d: could not read port, status = %d\n", tid, st);
            ret = __LINE__;
            break;
        }
        if (memcmp(pr.packet.value, kQuit.value, sizeof(pr.packet.value)) == 0) {
            ret = 0;
            break;
        }
        if (memcmp(pr.packet.value, kRepeat.value, sizeof(pr.packet.value)) == 0) {
            continue;
        }
        printf("thread %d: got a weird message from the control port\n", tid);
        ret = __LINE__;
    }
    thread_sleep((1+tid) * 5); // Make console output orderly.
    printf("thread %d: shutting down (ret=%d)\n", tid, ret);

    port_close(r_port);
    port_close(w_port);
    port_destroy(w_port);
    return ret;
}


int two_threads_race(void)
{
    printf("two_threads_race test . . .\n");
    // Used to tell the threads what to do.
    port_t w_port;
    status_t st = port_create("race_ctl", PORT_MODE_BROADCAST, &w_port);
    if (st < 0) {
        printf("could not create port, status = %d\n", st);
        return __LINE__;
    }

    event_init(&race_evt, false, 0);

    thread_t *t1 = thread_create(
                       "rt0", &race_thread, (void *)0, DEFAULT_PRIORITY, DEFAULT_STACK_SIZE);
    thread_t *t2 = thread_create(
                       "rt1", &race_thread, (void *)1, DEFAULT_PRIORITY, DEFAULT_STACK_SIZE);
    thread_set_real_time(t1);
    thread_set_real_time(t2);
    thread_resume(t1);
    thread_resume(t2);

    // wait for each status port to be created so we can
    // track behavior.
    port_t r_port0, r_port1;
    printf("control: connecting to thread 0 . . .\n");
    while (true) {
        status_t st = port_open(kStatusPortNames[0], NULL, &r_port0);
        if (st == NO_ERROR) {
            break;
        } else if (st == ERR_NOT_FOUND) {
            thread_sleep(100);
        } else {
            printf("could not open port, status = %d\n", st);
            // XXX: clean up...
            break;
        }
    }
    printf("control: connecting to thread 1 . . .\n");
    while (true) {
        status_t st = port_open(kStatusPortNames[1], NULL, &r_port1);
        if (st == NO_ERROR) {
            break;
        } else if (st == ERR_NOT_FOUND) {
            thread_sleep(100);
        } else {
            printf("could not open port, status = %d\n", st);
            return __LINE__;
        }
    }

    // control port says:  0 "REPEAT, or 1 "QUIT"
    int ret = 0;
    int count = 0;
    while (ret == 0) {
        LTRACEF_LEVEL(1, "Go!\n");
        printf(".");
        event_signal(&race_evt, false);
        port_result_t pr0, pr1;
        LTRACEF_LEVEL(1, "Collecting status from thread 0 . . .\n");
        st = port_read(r_port0, INFINITE_TIME, &pr0);
        if (st < 0) {
            printf("could not read port, status = %d\n", st);
            ret = __LINE__;
        }
        LTRACEF_LEVEL(1, "Collecting status from thread 1 . . .\n");
        st = port_read(r_port1, INFINITE_TIME, &pr1);
        if (st < 0) {
            printf("could not read port, status = %d\n", st);
            ret = __LINE__;
        }
        LTRACEF_LEVEL(1, "Checking responses . . .\n");
        if (memcmp(pr0.packet.value, pr1.packet.value, sizeof(pr0.packet.value)) != 0) {
            printf("Race detected on iteration %d!\n", count);
            ret = __LINE__;
        }
        event_unsignal(&race_evt);
        int repeat = (ret == 0 && count++ < 99 ? 1 : 0);
        LTRACEF_LEVEL(1, "Telling threads to %s\n", (repeat ? "repeat" : "quit"));
        st = port_write(w_port, (repeat ? &kRepeat : &kQuit), 1);
        if (st < 0) {
            printf("could not write port, status = %d\n", st);
            ret = __LINE__;
        }
        if (!repeat) {
            break;
        }
    }
    printf("\n%d passes completed with result %d\n", count, ret);

    st = port_close(r_port0);
    if (st < 0) {
        printf("could not close port, status = %d\n", st);
        ret = __LINE__;
    }

    st = port_close(r_port1);
    if (st < 0) {
        printf("could not close port, status = %d\n", st);
        ret = __LINE__;
    }

    st = port_close(w_port);
    if (st < 0) {
        printf("could not close port, status = %d\n", st);
        ret = __LINE__;
    }

    int retcode = -1;
    thread_join(t1, &retcode, INFINITE_TIME);
    if (retcode)
        ret = retcode;

    thread_join(t2,  &retcode, INFINITE_TIME);
    if (retcode)
        ret = retcode;

    st = port_destroy(w_port);
    if (st < 0) {
        printf("could not destroy port, status = %d\n", st);
        ret = __LINE__;
    }

    printf("two_thread_race: %d\n", ret);
    return ret;
}


static int two_threads_basic(void) {
    port_t w_port;
    status_t st = port_create("ping_port", PORT_MODE_BROADCAST, &w_port);
    if (st < 0) {
        printf("could not create port, status = %d\n", st);
        return __LINE__;
    }

    thread_t *t1 = thread_create(
                       "worker1", &ping_pong_thread, NULL, DEFAULT_PRIORITY, DEFAULT_STACK_SIZE);
    thread_t *t2 = thread_create(
                       "worker2", &ping_pong_thread, NULL, DEFAULT_PRIORITY, DEFAULT_STACK_SIZE);
    thread_resume(t1);
    thread_resume(t2);

    // wait for the pong port to be created, the two threads race to do it.
    port_t r_port;
    while (true) {
        st = port_open("pong_port", NULL, &r_port);
        if (st == NO_ERROR) {
            break;
        } else if (st == ERR_NOT_FOUND) {
            thread_sleep(100);
        } else {
            printf("could not open port, status = %d\n", st);
            return __LINE__;
        }
    }

    // We have two threads listening to the ping port. Which both reply
    // on the pong port, so we get two packets in per packet out.
    const int passes = 256;
    printf("two_threads_basic test, %d passes\n", passes);

    port_packet_t packet_out = {{0xaf, 0x77, 0x00, 0x01, 0x02, 0x03, 0x04, 0x05}};

    port_result_t pr;
    for (int ix = 0; ix != passes; ++ix) {
        const size_t count = 1 + ((unsigned int)rand() % 3);

        for (size_t jx = 0; jx != count; ++jx) {
            st = port_write(w_port, &packet_out, 1);
            if (st < 0) {
                printf("could not write port, status = %d\n", st);
                return __LINE__;
            }
        }

        packet_out.value[0]++;
        packet_out.value[5]--;

        for (size_t jx = 0; jx != count * 2; ++jx) {
            st = port_read(r_port, INFINITE_TIME, &pr);
            if (st < 0) {
                printf("could not read port, status = %d\n", st);
                return __LINE__;
            }

            if ((pr.packet.value[0] != packet_out.value[0]) ||
                    (pr.packet.value[5] != packet_out.value[5])) {
                printf("unexpected data in packet, loop %d", ix);
                return __LINE__;
            }
        }
    }

    thread_sleep(100);

    // there should be no more packets to read.
    st = port_read(r_port, 0, &pr);
    if (st != ERR_TIMED_OUT) {
        printf("unexpected packet, status = %d\n", st);
        return __LINE__;
    }

    printf("two_threads_basic master shutdown\n");

    st = port_close(r_port);
    if (st < 0) {
        printf("could not close port, status = %d\n", st);
        return __LINE__;
    }

    st = port_close(w_port);
    if (st < 0) {
        printf("could not close port, status = %d\n", st);
        return __LINE__;
    }

    st = port_destroy(w_port);
    if (st < 0) {
        printf("could not destroy port, status = %d\n", st);
        return __LINE__;
    }

    int retcode = -1;
    thread_join(t1, &retcode, INFINITE_TIME);
    if (retcode)
        goto fail;

    thread_join(t2,  &retcode, INFINITE_TIME);
    if (retcode)
        goto fail;

    return 0;

fail:
    printf("child thread exited with %d\n", retcode);
    return __LINE__;
}

#define CMD_PORT_CTX ((void*) 0x77)
#define TS1_PORT_CTX ((void*) 0x11)
#define TS2_PORT_CTX ((void*) 0x12)

typedef enum {
    ADD_PORT,
    QUIT
} action_t;

typedef struct {
    action_t what;
    port_t port;
} watcher_cmd;

static status_t send_watcher_cmd(port_t cmd_port, action_t action, port_t port) {
    watcher_cmd _cmd  = {action, port};
    return port_write(cmd_port, ((port_packet_t *) &_cmd), 1);;
}

static int group_watcher_thread(void *arg) {
    port_t watched[8] = {0};
    status_t st = port_open("grp_ctrl", CMD_PORT_CTX, &watched[0]);
    if (st < 0) {
        printf("could not open port, status = %d\n", st);
        return __LINE__;
    }

    size_t count = 1;
    port_t group;
    int ctx_count = -1;

    while (true) {
        st = port_group(watched, count, &group);
        if (st < 0) {
            printf("could not make group, status = %d\n", st);
            return __LINE__;
        }

        port_result_t pr;
        while (true) {
            st = port_read(group, INFINITE_TIME, &pr);
            if (st < 0) {
                printf("could not read port, status = %d\n", st);
                return __LINE__;
            }

            if (pr.ctx == CMD_PORT_CTX) {
                break;
            } else if (pr.ctx == TS1_PORT_CTX) {
                ctx_count += 1;
            } else if (pr.ctx == TS2_PORT_CTX) {
                ctx_count += 2;
            } else {
                printf("unknown context %p\n", pr.ctx);
                return __LINE__;
            }
        }

        // Either adding a port or exiting; either way close the
        // existing group port and create a new one if needed
        // at the top of the loop.

        port_close(group);
        watcher_cmd *wc = (watcher_cmd *) &pr.packet;

        if (wc->what == ADD_PORT) {
            watched[count++] = wc->port;
        }  else if (wc->what == QUIT) {
            break;
        } else {
            printf("unknown command %d\n", wc->what);
            return __LINE__;
        }
    }

    if (ctx_count !=  2) {
        printf("unexpected context count %d", ctx_count);
        return __LINE__;
    }

    printf("group watcher shutdown\n");

    for (size_t ix = 0; ix != count; ++ix) {
        st = port_close(watched[ix]);
        if (st < 0) {
            printf("failed to close read port, status = %d\n", st);
            return __LINE__;
        }
    }

    return 0;
}

static status_t make_port_pair(const char *name, void *ctx, port_t *write, port_t *read) {
    status_t st = port_create(name, PORT_MODE_UNICAST, write);
    if (st < 0)
        return st;
    return port_open(name,ctx, read);
}

static int group_basic(void) {
    // we spin a thread that connects to a well known port, then we
    // send two ports that it will add to a group port.
    port_t cmd_port;
    status_t st = port_create("grp_ctrl", PORT_MODE_UNICAST, &cmd_port);
    if (st < 0 ) {
        printf("could not create port, status = %d\n", st);
        return __LINE__;
    }

    thread_t *wt = thread_create(
                       "g_watcher", &group_watcher_thread, NULL, DEFAULT_PRIORITY, DEFAULT_STACK_SIZE);
    thread_resume(wt);

    port_t w_test_port1, r_test_port1;
    st = make_port_pair("tst_port1", TS1_PORT_CTX, &w_test_port1, &r_test_port1);
    if (st < 0)
        return __LINE__;

    port_t w_test_port2, r_test_port2;
    st = make_port_pair("tst_port2", TS2_PORT_CTX, &w_test_port2, &r_test_port2);
    if (st < 0)
        return __LINE__;

    st = send_watcher_cmd(cmd_port, ADD_PORT, r_test_port1);
    if (st < 0)
        return __LINE__;

    st = send_watcher_cmd(cmd_port, ADD_PORT, r_test_port2);
    if (st < 0)
        return __LINE__;

    thread_sleep(50);

    port_packet_t pp = {{0}};
    st = port_write(w_test_port1, &pp, 1);
    if (st < 0)
        return __LINE__;

    st = port_write(w_test_port2, &pp, 1);
    if (st < 0)
        return __LINE__;

    st = send_watcher_cmd(cmd_port, QUIT, 0);
    if (st < 0)
        return __LINE__;

    int retcode = -1;
    thread_join(wt, &retcode, INFINITE_TIME);
    if (retcode) {
        printf("child thread exited with %d\n", retcode);
        return __LINE__;
    }

    st = port_close(w_test_port1);
    if (st < 0)
        return __LINE__;
    st = port_close(w_test_port2);
    if (st < 0)
        return __LINE__;
    st = port_close(cmd_port);
    if (st < 0)
        return __LINE__;
    st = port_destroy(w_test_port1);
    if (st < 0)
        return __LINE__;
    st = port_destroy(w_test_port2);
    if (st < 0)
        return __LINE__;
    st = port_destroy(cmd_port);
    if (st < 0)
        return __LINE__;

    return 0;
}

static int group_dynamic(void) {
    status_t st;

    port_t w_test_port1, r_test_port1;
    st = make_port_pair("tst_port1", TS1_PORT_CTX, &w_test_port1, &r_test_port1);
    if (st < 0)
        return __LINE__;

    port_t w_test_port2, r_test_port2;
    st = make_port_pair("tst_port2", TS2_PORT_CTX, &w_test_port2, &r_test_port2);
    if (st < 0)
        return __LINE__;

    port_t pg;
    st = port_group(&r_test_port1, 1, &pg);
    if (st < 0)
        return __LINE__;

    port_packet_t pkt = { { 0 } };
    st = port_write(w_test_port2, &pkt, 1);
    if (st < 0)
        return __LINE__;

    port_result_t rslt;
    st = port_read(pg, 0, &rslt);
    if (st != ERR_TIMED_OUT)
        return __LINE__;

    // Attach the port that has been written to to the port group and ensure
    // that we can read from it.
    st = port_group_add(pg, r_test_port2);
    if (st < 0)
        return __LINE__;

    st = port_read(pg, 0, &rslt);
    if (st < 0)
        return __LINE__;

    // Write some data to a port then remove it from the port group and ensure
    // that we can't read from it.
    st = port_write(w_test_port1, &pkt, 1);
    if (st < 0)
        return __LINE__;

    st = port_group_remove(pg, r_test_port1);
    if (st < 0)
        return __LINE__;

    st = port_read(pg, 0, &rslt);
    if (st != ERR_TIMED_OUT)
        return __LINE__;

    st = port_close(w_test_port1);
    if (st < 0)
        return __LINE__;
    st = port_close(w_test_port2);
    if (st < 0)
        return __LINE__;
    st = port_destroy(w_test_port1);
    if (st < 0)
        return __LINE__;
    st = port_destroy(w_test_port2);
    if (st < 0)
        return __LINE__;

    return 0;
}

static event_t group_waiting_sync_evt;

static int receive_thread(void *arg) {
    port_t pg = (port_t)arg;

    // Try to read from an empty port group. When the other thread adds a port
    // to this port group, we should wake up and
    port_result_t rslt;
    status_t st = port_read(pg, 500, &rslt);
    if (st == ERR_TIMED_OUT)
        return __LINE__;

    event_signal(&group_waiting_sync_evt, true);

    return 0;
}

/* Test the edge case where a read port with data available is added to a port
 * group that has a read-blocked receiver.
 */
static int group_waiting(void) {
    status_t st;

    event_init(&group_waiting_sync_evt, false, EVENT_FLAG_AUTOUNSIGNAL);

    port_t w_test_port1, r_test_port1;
    st = make_port_pair("tst_port1", TS1_PORT_CTX, &w_test_port1, &r_test_port1);
    if (st < 0)
        return __LINE__;

    // Write something to this port group that currently has no receivers.
    port_packet_t pkt = { { 0 } };
    st = port_write(w_test_port1, &pkt, 1);
    if (st < 0)
        return __LINE__;

    // Create an empty port group.
    port_t pg;
    st = port_group(NULL, 0, &pg);
    if (st < 0)
        return __LINE__;


    thread_t *t1 = thread_create(
                       "receiver",
                       &receive_thread,
                       (void *)pg,
                       DEFAULT_PRIORITY,
                       DEFAULT_STACK_SIZE
                   );

    thread_resume(t1);

    // Wait for the other thread to block on the read.
    thread_sleep(20);

    // Adding a port that has data available to the port group should wake any
    // threads waiting on that port group.
    port_group_add(pg, r_test_port1);

    if (event_wait_timeout(&group_waiting_sync_evt, 500) != NO_ERROR)
        return __LINE__;

    st = port_close(w_test_port1);
    if (st < 0)
        return __LINE__;

    st = port_destroy(w_test_port1);
    if (st < 0)
        return __LINE__;

    return 0;
}

#define RUN_TEST(t)  result = t(); if (result) goto fail

int port_tests(int argc, const console_cmd_args *argv) {
    int result;
    int count = 3;
    while (count--) {
        RUN_TEST(single_thread_basic);
        RUN_TEST(two_threads_basic);
        RUN_TEST(two_threads_race);
        RUN_TEST(group_basic);
        RUN_TEST(group_dynamic);
    }

    printf("all tests passed\n");
    return 0;
fail:
    printf("test failed at line %d\n", result);
    return 1;
}

#undef RUN_TEST
