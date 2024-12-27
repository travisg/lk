/*
 * Copyright (c) 2008-2015 Travis Geiselbrecht
 *
 * Use of this source code is governed by a MIT-style
 * license that can be found in the LICENSE file or at
 * https://opensource.org/licenses/MIT
 */

/**
 * @file
 * @brief  Kernel threading
 *
 * This file is the core kernel threading interface.
 *
 * @defgroup thread Threads
 * @{
 */
#include <kernel/thread.h>

#include <assert.h>
#include <kernel/debug.h>
#include <kernel/mp.h>
#include <kernel/timer.h>
#include <lib/heap.h>
#include <lk/debug.h>
#include <lk/err.h>
#include <lk/list.h>
#include <lk/trace.h>
#include <malloc.h>
#include <platform.h>
#include <printf.h>
#include <string.h>
#include <target.h>
#if WITH_KERNEL_VM
#include <kernel/vm.h>
#endif

#if THREAD_STATS
struct thread_stats thread_stats[SMP_MAX_CPUS];
#endif

#define STACK_DEBUG_BYTE (0x99)
#define STACK_DEBUG_WORD (0x99999999)

#define DEBUG_THREAD_CONTEXT_SWITCH 0

/* global thread list */
struct list_node thread_list;

/* master thread spinlock */
spin_lock_t thread_lock = SPIN_LOCK_INITIAL_VALUE;

/* the run queue */
static struct list_node run_queue[NUM_PRIORITIES];
static uint32_t run_queue_bitmap;

/* make sure the bitmap is large enough to cover our number of priorities */
STATIC_ASSERT(NUM_PRIORITIES <= sizeof(run_queue_bitmap) * 8);

/* the idle thread(s) (statically allocated) */
#if WITH_SMP
static thread_t _idle_threads[SMP_MAX_CPUS];
#define idle_thread(cpu) (&_idle_threads[cpu])
#else
static thread_t _idle_thread;
#define idle_thread(cpu) (&_idle_thread)
#endif

/* local routines */
static void thread_resched(void);
static void idle_thread_routine(void) __NO_RETURN;

#if PLATFORM_HAS_DYNAMIC_TIMER
/* preemption timer */
static timer_t preempt_timer[SMP_MAX_CPUS];
#endif

/* run queue manipulation */
static void insert_in_run_queue_head(thread_t *t) {
    DEBUG_ASSERT(t->magic == THREAD_MAGIC);
    DEBUG_ASSERT(t->state == THREAD_READY);
    DEBUG_ASSERT(!list_in_list(&t->queue_node));
    DEBUG_ASSERT(arch_ints_disabled());
    DEBUG_ASSERT(spin_lock_held(&thread_lock));

    list_add_head(&run_queue[t->priority], &t->queue_node);
    run_queue_bitmap |= (1<<t->priority);
}

static void insert_in_run_queue_tail(thread_t *t) {
    DEBUG_ASSERT(t->magic == THREAD_MAGIC);
    DEBUG_ASSERT(t->state == THREAD_READY);
    DEBUG_ASSERT(!list_in_list(&t->queue_node));
    DEBUG_ASSERT(arch_ints_disabled());
    DEBUG_ASSERT(spin_lock_held(&thread_lock));

    list_add_tail(&run_queue[t->priority], &t->queue_node);
    run_queue_bitmap |= (1<<t->priority);
}

static void wakeup_cpu_for_thread(thread_t *t)
{
    /* Wake up the core to which this thread is pinned
     * or wake up all if thread is unpinned */
    int pinned_cpu = thread_pinned_cpu(t);
    if (pinned_cpu < 0)
        mp_reschedule(MP_CPU_ALL_BUT_LOCAL, 0);
    else
        mp_reschedule(1U << pinned_cpu, 0);
}

static void init_thread_struct(thread_t *t, const char *name) {
    memset(t, 0, sizeof(thread_t));
    t->magic = THREAD_MAGIC;
    thread_set_pinned_cpu(t, -1);
    strlcpy(t->name, name, sizeof(t->name));
}

/**
 * @brief  Create a new thread
 *
 * This function creates a new thread.  The thread is initially suspended, so you
 * need to call thread_resume() to execute it.
 *
 * @param  name        Name of thread
 * @param  entry       Entry point of thread
 * @param  arg         Arbitrary argument passed to entry()
 * @param  priority    Execution priority for the thread.
 * @param  stack_size  Stack size for the thread.
 *
 * Thread priority is an integer from 0 (lowest) to 31 (highest).  Some standard
 * prioritys are defined in <kernel/thread.h>:
 *
 *  HIGHEST_PRIORITY
 *  DPC_PRIORITY
 *  HIGH_PRIORITY
 *  DEFAULT_PRIORITY
 *  LOW_PRIORITY
 *  IDLE_PRIORITY
 *  LOWEST_PRIORITY
 *
 * Stack size is typically set to DEFAULT_STACK_SIZE
 *
 * @return  Pointer to thread object, or NULL on failure.
 */
thread_t *thread_create_etc(thread_t *t, const char *name, thread_start_routine entry, void *arg, int priority, void *stack, size_t stack_size) {
    unsigned int flags = 0;

    if (!t) {
        t = malloc(sizeof(thread_t));
        if (!t)
            return NULL;
        flags |= THREAD_FLAG_FREE_STRUCT;
    }

    init_thread_struct(t, name);

    t->entry = entry;
    t->arg = arg;
    t->priority = priority;
    t->state = THREAD_SUSPENDED;
    t->blocking_wait_queue = NULL;
    t->wait_queue_block_ret = NO_ERROR;
    thread_set_curr_cpu(t, -1);

    t->retcode = 0;
    wait_queue_init(&t->retcode_wait_queue);

#if WITH_KERNEL_VM
    t->aspace = NULL;
#endif

    /* create the stack */
    if (!stack) {
#if THREAD_STACK_BOUNDS_CHECK
        stack_size += THREAD_STACK_PADDING_SIZE;
        flags |= THREAD_FLAG_DEBUG_STACK_BOUNDS_CHECK;
#endif
        t->stack = malloc(stack_size);
        if (!t->stack) {
            if (flags & THREAD_FLAG_FREE_STRUCT)
                free(t);
            return NULL;
        }
        flags |= THREAD_FLAG_FREE_STACK;
#if THREAD_STACK_BOUNDS_CHECK
        memset(t->stack, STACK_DEBUG_BYTE, THREAD_STACK_PADDING_SIZE);
#endif
    } else {
        t->stack = stack;
    }
#if THREAD_STACK_HIGHWATER
    if (flags & THREAD_FLAG_DEBUG_STACK_BOUNDS_CHECK) {
        memset(t->stack + THREAD_STACK_PADDING_SIZE, STACK_DEBUG_BYTE,
               stack_size - THREAD_STACK_PADDING_SIZE);
    } else {
        memset(t->stack, STACK_DEBUG_BYTE, stack_size);
    }
#endif

    t->stack_size = stack_size;

    /* save whether or not we need to free the thread struct and/or stack */
    t->flags = flags;

    /* inherit thread local storage from the parent */
    thread_t *current_thread = get_current_thread();
    int i;
    for (i=0; i < MAX_TLS_ENTRY; i++)
        t->tls[i] = current_thread->tls[i];

    /* set up the initial stack frame */
    arch_thread_initialize(t);

    /* add it to the global thread list */
    THREAD_LOCK(state);
    list_add_head(&thread_list, &t->thread_list_node);
    THREAD_UNLOCK(state);

    return t;
}

thread_t *thread_create(const char *name, thread_start_routine entry, void *arg, int priority, size_t stack_size) {
    return thread_create_etc(NULL, name, entry, arg, priority, NULL, stack_size);
}

/**
 * @brief Flag a thread as real time
 *
 * @param t Thread to flag
 *
 * @return NO_ERROR on success
 */
status_t thread_set_real_time(thread_t *t) {
    if (!t)
        return ERR_INVALID_ARGS;

    DEBUG_ASSERT(t->magic == THREAD_MAGIC);

    THREAD_LOCK(state);
#if PLATFORM_HAS_DYNAMIC_TIMER
    if (t == get_current_thread()) {
        /* if we're currently running, cancel the preemption timer. */
        timer_cancel(&preempt_timer[arch_curr_cpu_num()]);
    }
#endif
    t->flags |= THREAD_FLAG_REAL_TIME;
    THREAD_UNLOCK(state);

    return NO_ERROR;
}

static bool thread_is_realtime(thread_t *t) {
    return (t->flags & THREAD_FLAG_REAL_TIME) && t->priority > DEFAULT_PRIORITY;
}

static bool thread_is_idle(thread_t *t) {
    return !!(t->flags & THREAD_FLAG_IDLE);
}

static bool thread_is_real_time_or_idle(thread_t *t) {
    return !!(t->flags & (THREAD_FLAG_REAL_TIME | THREAD_FLAG_IDLE));
}

/**
 * @brief  Make a suspended thread executable.
 *
 * This function is typically called to start a thread which has just been
 * created with thread_create()
 *
 * @param t  Thread to resume
 *
 * @return NO_ERROR on success, ERR_NOT_SUSPENDED if thread was not suspended.
 */
status_t thread_resume(thread_t *t) {
    DEBUG_ASSERT(t->magic == THREAD_MAGIC);
    DEBUG_ASSERT(t->state != THREAD_DEATH);

    bool resched = false;
    bool ints_disabled = arch_ints_disabled();
    THREAD_LOCK(state);
    if (t->state == THREAD_SUSPENDED) {
        t->state = THREAD_READY;
        insert_in_run_queue_head(t);
        if (!ints_disabled) /* HACK, don't resced into bootstrap thread before idle thread is set up */
            resched = true;
    }

    wakeup_cpu_for_thread(t);

    THREAD_UNLOCK(state);

    if (resched)
        thread_yield();

    return NO_ERROR;
}

status_t thread_detach_and_resume(thread_t *t) {
    status_t err;
    err = thread_detach(t);
    if (err < 0)
        return err;
    return thread_resume(t);
}

status_t thread_join(thread_t *t, int *retcode, lk_time_t timeout) {
    DEBUG_ASSERT(t->magic == THREAD_MAGIC);

    THREAD_LOCK(state);

    if (t->flags & THREAD_FLAG_DETACHED) {
        /* the thread is detached, go ahead and exit */
        THREAD_UNLOCK(state);
        return ERR_THREAD_DETACHED;
    }

    /* wait for the thread to die */
    if (t->state != THREAD_DEATH) {
        status_t err = wait_queue_block(&t->retcode_wait_queue, timeout);
        if (err < 0) {
            THREAD_UNLOCK(state);
            return err;
        }
    }

    DEBUG_ASSERT(t->magic == THREAD_MAGIC);
    DEBUG_ASSERT(t->state == THREAD_DEATH);
    DEBUG_ASSERT(t->blocking_wait_queue == NULL);
    DEBUG_ASSERT(!list_in_list(&t->queue_node));

    /* save the return code */
    if (retcode)
        *retcode = t->retcode;

    /* remove it from the master thread list */
    list_delete(&t->thread_list_node);

    /* clear the structure's magic */
    t->magic = 0;

    THREAD_UNLOCK(state);

    /* free its stack and the thread structure itself */
    if (t->flags & THREAD_FLAG_FREE_STACK && t->stack)
        free(t->stack);

    if (t->flags & THREAD_FLAG_FREE_STRUCT)
        free(t);

    return NO_ERROR;
}

status_t thread_detach(thread_t *t) {
    DEBUG_ASSERT(t->magic == THREAD_MAGIC);

    THREAD_LOCK(state);

    /* if another thread is blocked inside thread_join() on this thread,
     * wake them up with a specific return code */
    wait_queue_wake_all(&t->retcode_wait_queue, false, ERR_THREAD_DETACHED);

    /* if it's already dead, then just do what join would have and exit */
    if (t->state == THREAD_DEATH) {
        t->flags &= ~THREAD_FLAG_DETACHED; /* makes sure thread_join continues */
        THREAD_UNLOCK(state);
        return thread_join(t, NULL, 0);
    } else {
        t->flags |= THREAD_FLAG_DETACHED;
        THREAD_UNLOCK(state);
        return NO_ERROR;
    }
}

/**
 * @brief  Terminate the current thread
 *
 * Current thread exits with the specified return code.
 *
 * This function does not return.
 */
void thread_exit(int retcode) {
    thread_t *current_thread = get_current_thread();

    DEBUG_ASSERT(current_thread->magic == THREAD_MAGIC);
    DEBUG_ASSERT(current_thread->state == THREAD_RUNNING);
    DEBUG_ASSERT(!thread_is_idle(current_thread));

//  dprintf("thread_exit: current %p\n", current_thread);

    THREAD_LOCK(state);

    /* enter the dead state */
    current_thread->state = THREAD_DEATH;
    current_thread->retcode = retcode;

    /* if we're detached, then do our teardown here */
    if (current_thread->flags & THREAD_FLAG_DETACHED) {
        /* remove it from the master thread list */
        list_delete(&current_thread->thread_list_node);

        /* clear the structure's magic */
        current_thread->magic = 0;

        /* free its stack and the thread structure itself */
        if (current_thread->flags & THREAD_FLAG_FREE_STACK && current_thread->stack) {
            heap_delayed_free(current_thread->stack);

            /* make sure its not going to get a bounds check performed on the half-freed stack */
            current_thread->flags &= ~THREAD_FLAG_DEBUG_STACK_BOUNDS_CHECK;
        }

        if (current_thread->flags & THREAD_FLAG_FREE_STRUCT)
            heap_delayed_free(current_thread);
    } else {
        /* signal if anyone is waiting */
        wait_queue_wake_all(&current_thread->retcode_wait_queue, false, 0);
    }

    /* reschedule */
    thread_resched();

    panic("somehow fell through thread_exit()\n");
}

static void idle_thread_routine(void) {
    for (;;)
        arch_idle();
}

static thread_t *get_top_thread(int cpu) {
    thread_t *newthread;
    uint32_t local_run_queue_bitmap = run_queue_bitmap;

    while (local_run_queue_bitmap) {
        /* find the first (remaining) queue with a thread in it */
        uint next_queue = sizeof(run_queue_bitmap) * 8 - 1 - __builtin_clz(local_run_queue_bitmap);

        list_for_every_entry(&run_queue[next_queue], newthread, thread_t, queue_node) {
#if WITH_SMP
            if (newthread->pinned_cpu < 0 || newthread->pinned_cpu == cpu)
#endif
            {
                list_delete(&newthread->queue_node);

                if (list_is_empty(&run_queue[next_queue]))
                    run_queue_bitmap &= ~(1<<next_queue);

                return newthread;
            }
        }

        local_run_queue_bitmap &= ~(1<<next_queue);
    }
    /* no threads to run, select the idle thread for this cpu */
    return idle_thread(cpu);
}

/**
 * @brief  Cause another thread to be executed.
 *
 * Internal reschedule routine. The current thread needs to already be in whatever
 * state and queues it needs to be in. This routine simply picks the next thread and
 * switches to it.
 *
 * This is probably not the function you're looking for. See
 * thread_yield() instead.
 */
void thread_resched(void) {
    thread_t *oldthread;
    thread_t *newthread;

    thread_t *current_thread = get_current_thread();
    uint cpu = arch_curr_cpu_num();

    /* Assert that cpu is not in active irq handling
     * context */
    ASSERT(!arch_in_int_handler());

    DEBUG_ASSERT(arch_ints_disabled());
    DEBUG_ASSERT(spin_lock_held(&thread_lock));
    DEBUG_ASSERT(current_thread->state != THREAD_RUNNING);

    THREAD_STATS_INC(reschedules);

    newthread = get_top_thread(cpu);

    DEBUG_ASSERT(newthread);

    newthread->state = THREAD_RUNNING;

    oldthread = current_thread;

    if (newthread == oldthread)
        return;

    /* set up quantum for the new thread if it was consumed */
    if (newthread->remaining_quantum <= 0) {
        newthread->remaining_quantum = 5; // XXX make this smarter
    }

    /* mark the cpu ownership of the threads */
    thread_set_curr_cpu(oldthread, -1);
    thread_set_curr_cpu(newthread, cpu);

#if WITH_SMP
    if (thread_is_idle(newthread)) {
        mp_set_cpu_idle(cpu);
    } else {
        mp_set_cpu_busy(cpu);
    }

    if (thread_is_realtime(newthread)) {
        mp_set_cpu_realtime(cpu);
    } else {
        mp_set_cpu_non_realtime(cpu);
    }
#endif

#if THREAD_STATS
    THREAD_STATS_INC(context_switches);

    lk_bigtime_t now = current_time_hires();
    if (thread_is_idle(oldthread)) {
        thread_stats[cpu].idle_time += now - thread_stats[cpu].last_idle_timestamp;
    } else {
        oldthread->stats.total_run_time += now - oldthread->stats.last_run_timestamp;
    }
    if (thread_is_idle(newthread)) {
        thread_stats[cpu].last_idle_timestamp = now;
    } else {
        newthread->stats.last_run_timestamp = now;
        newthread->stats.schedules++;
    }
#endif

    KEVLOG_THREAD_SWITCH(oldthread, newthread);

#if PLATFORM_HAS_DYNAMIC_TIMER
    if (thread_is_real_time_or_idle(newthread)) {
        if (!thread_is_real_time_or_idle(oldthread)) {
            /* if we're switching from a non real time to a real time, cancel
             * the preemption timer. */
#if DEBUG_THREAD_CONTEXT_SWITCH
            dprintf(ALWAYS, "arch_context_switch: stop preempt, cpu %d, old %p (%s), new %p (%s)\n",
                    cpu, oldthread, oldthread->name, newthread, newthread->name);
#endif
            timer_cancel(&preempt_timer[cpu]);
        }
    } else if (thread_is_real_time_or_idle(oldthread)) {
        /* if we're switching from a real time (or idle thread) to a regular one,
         * set up a periodic timer to run our preemption tick. */
#if DEBUG_THREAD_CONTEXT_SWITCH
        dprintf(ALWAYS, "arch_context_switch: start preempt, cpu %d, old %p (%s), new %p (%s)\n",
                cpu, oldthread, oldthread->name, newthread, newthread->name);
#endif
        timer_set_periodic(&preempt_timer[cpu], 10, thread_timer_tick, NULL);
    }
#endif

    /* set some optional target debug leds */
    target_set_debug_led(0, !thread_is_idle(newthread));

    /* do the switch */
    set_current_thread(newthread);

#if DEBUG_THREAD_CONTEXT_SWITCH
    dprintf(ALWAYS, "arch_context_switch: cpu %d, old %p (%s, pri %d, flags 0x%x), new %p (%s, pri %d, flags 0x%x)\n",
            cpu, oldthread, oldthread->name, oldthread->priority,
            oldthread->flags, newthread, newthread->name,
            newthread->priority, newthread->flags);
#endif

#if THREAD_STACK_BOUNDS_CHECK
    /* check that the old thread has not blown its stack just before pushing its context */
    if (oldthread->flags & THREAD_FLAG_DEBUG_STACK_BOUNDS_CHECK) {
        STATIC_ASSERT((THREAD_STACK_PADDING_SIZE % sizeof(uint32_t)) == 0);
        uint32_t *s = (uint32_t *)oldthread->stack;
        for (size_t i = 0; i < THREAD_STACK_PADDING_SIZE / sizeof(uint32_t); i++) {
            if (unlikely(s[i] != STACK_DEBUG_WORD)) {
                /* NOTE: will probably blow the stack harder here, but hopefully enough
                 * state exists to at least get some sort of debugging done.
                 */
                panic("stack overrun at %p: thread %p (%s), stack %p\n", &s[i],
                      oldthread, oldthread->name, oldthread->stack);
            }
        }
    }
#endif

#ifdef WITH_LIB_UTHREAD
    uthread_context_switch(oldthread, newthread);
#endif

#if WITH_KERNEL_VM
    /* see if we need to swap mmu context */
    if (newthread->aspace != oldthread->aspace) {
        vmm_context_switch(oldthread->aspace, newthread->aspace);
    }
#endif

    /* do the low level context switch */
    arch_context_switch(oldthread, newthread);
}

/**
 * @brief Yield the cpu to another thread
 *
 * This function places the current thread at the end of the run queue
 * and yields the cpu to another waiting thread (if any.)
 *
 * This function will return at some later time. Possibly immediately if
 * no other threads are waiting to execute.
 */
void thread_yield(void) {
    thread_t *current_thread = get_current_thread();

    DEBUG_ASSERT(current_thread->magic == THREAD_MAGIC);
    DEBUG_ASSERT(current_thread->state == THREAD_RUNNING);

    THREAD_LOCK(state);

    THREAD_STATS_INC(yields);

    /* we are yielding the cpu, so stick ourselves into the tail of the run queue and reschedule */
    current_thread->state = THREAD_READY;
    current_thread->remaining_quantum = 0;
    if (likely(!thread_is_idle(current_thread))) { /* idle thread doesn't go in the run queue */
        insert_in_run_queue_tail(current_thread);
    }
    thread_resched();

    THREAD_UNLOCK(state);
}

/**
 * @brief  Briefly yield cpu to another thread
 *
 * This function is similar to thread_yield(), except that it will
 * restart more quickly.
 *
 * This function places the current thread at the head of the run
 * queue and then yields the cpu to another thread.
 *
 * Exception:  If the time slice for this thread has expired, then
 * the thread goes to the end of the run queue.
 *
 * This function will return at some later time. Possibly immediately if
 * no other threads are waiting to execute.
 */
void thread_preempt(void) {
    thread_t *current_thread = get_current_thread();

    DEBUG_ASSERT(current_thread->magic == THREAD_MAGIC);
    DEBUG_ASSERT(current_thread->state == THREAD_RUNNING);

#if THREAD_STATS
    if (!thread_is_idle(current_thread))
        THREAD_STATS_INC(preempts); /* only track when a meaningful preempt happens */
#endif

    KEVLOG_THREAD_PREEMPT(current_thread);

    THREAD_LOCK(state);

    /* we are being preempted, so we get to go back into the front of the run queue if we have quantum left */
    current_thread->state = THREAD_READY;
    if (likely(!thread_is_idle(current_thread))) { /* idle thread doesn't go in the run queue */
        if (current_thread->remaining_quantum > 0)
            insert_in_run_queue_head(current_thread);
        else
            insert_in_run_queue_tail(current_thread); /* if we're out of quantum, go to the tail of the queue */
    }
    thread_resched();

    THREAD_UNLOCK(state);
}

/**
 * @brief  Suspend thread until woken.
 *
 * This function schedules another thread to execute.  This function does not
 * return until the thread is made runable again by some other module.
 *
 * You probably don't want to call this function directly; it's meant to be called
 * from other modules, such as mutex, which will presumably set the thread's
 * state to blocked and add it to some queue or another.
 */
void thread_block(void) {
    __UNUSED thread_t *current_thread = get_current_thread();

    DEBUG_ASSERT(current_thread->magic == THREAD_MAGIC);
    DEBUG_ASSERT(current_thread->state == THREAD_BLOCKED);
    DEBUG_ASSERT(spin_lock_held(&thread_lock));
    DEBUG_ASSERT(!thread_is_idle(current_thread));

    /* we are blocking on something. the blocking code should have already stuck us on a queue */
    thread_resched();
}

void thread_unblock(thread_t *t, bool resched) {
    DEBUG_ASSERT(t->magic == THREAD_MAGIC);
    DEBUG_ASSERT(t->state == THREAD_BLOCKED);
    DEBUG_ASSERT(spin_lock_held(&thread_lock));
    DEBUG_ASSERT(!thread_is_idle(t));

    t->state = THREAD_READY;
    insert_in_run_queue_head(t);
    wakeup_cpu_for_thread(t);

    if (resched)
        thread_resched();
}

enum handler_return thread_timer_tick(struct timer *t, lk_time_t now, void *arg) {
    thread_t *current_thread = get_current_thread();

    if (thread_is_real_time_or_idle(current_thread))
        return INT_NO_RESCHEDULE;

    current_thread->remaining_quantum--;
    if (current_thread->remaining_quantum <= 0) {
        return INT_RESCHEDULE;
    } else {
        return INT_NO_RESCHEDULE;
    }
}

/* timer callback to wake up a sleeping thread */
static enum handler_return thread_sleep_handler(timer_t *timer, lk_time_t now, void *arg) {
    thread_t *t = (thread_t *)arg;

    DEBUG_ASSERT(t->magic == THREAD_MAGIC);
    DEBUG_ASSERT(t->state == THREAD_SLEEPING);

    THREAD_LOCK(state);

    t->state = THREAD_READY;
    insert_in_run_queue_head(t);

    THREAD_UNLOCK(state);

    return INT_RESCHEDULE;
}

/**
 * @brief  Put thread to sleep; delay specified in ms
 *
 * This function puts the current thread to sleep until the specified
 * delay in ms has expired.
 *
 * Note that this function could sleep for longer than the specified delay if
 * other threads are running.  When the timer expires, this thread will
 * be placed at the head of the run queue.
 */
void thread_sleep(lk_time_t delay) {
    timer_t timer;

    thread_t *current_thread = get_current_thread();

    DEBUG_ASSERT(current_thread->magic == THREAD_MAGIC);
    DEBUG_ASSERT(current_thread->state == THREAD_RUNNING);
    DEBUG_ASSERT(!thread_is_idle(current_thread));

    timer_initialize(&timer);

    THREAD_LOCK(state);
    timer_set_oneshot(&timer, delay, thread_sleep_handler, (void *)current_thread);
    current_thread->state = THREAD_SLEEPING;
    thread_resched();
    THREAD_UNLOCK(state);
}

/**
 * @brief  Initialize threading system
 *
 * This function is called once, from kmain()
 */
void thread_init_early(void) {
    int i;

    DEBUG_ASSERT(arch_curr_cpu_num() == 0);

    /* initialize the run queues */
    for (i=0; i < NUM_PRIORITIES; i++)
        list_initialize(&run_queue[i]);

    /* initialize the thread list */
    list_initialize(&thread_list);

    /* create a thread to cover the current running state */
    thread_t *t = idle_thread(0);
    init_thread_struct(t, "bootstrap");

    /* half construct this thread, since we're already running */
    t->priority = HIGHEST_PRIORITY;
    t->state = THREAD_RUNNING;
    t->flags = THREAD_FLAG_DETACHED;
    thread_set_curr_cpu(t, 0);
    thread_set_pinned_cpu(t, 0);
    wait_queue_init(&t->retcode_wait_queue);
    list_add_head(&thread_list, &t->thread_list_node);
    set_current_thread(t);
}

/**
 * @brief Complete thread initialization
 *
 * This function is called once at boot time
 */
void thread_init(void) {
#if PLATFORM_HAS_DYNAMIC_TIMER
    for (uint i = 0; i < SMP_MAX_CPUS; i++) {
        timer_initialize(&preempt_timer[i]);
    }
#endif
}

/**
 * @brief Change name of current thread
 */
void thread_set_name(const char *name) {
    thread_t *current_thread = get_current_thread();
    strlcpy(current_thread->name, name, sizeof(current_thread->name));
}

/**
 * @brief Change priority of current thread
 *
 * See thread_create() for a discussion of priority values.
 */
void thread_set_priority(int priority) {
    thread_t *current_thread = get_current_thread();

    THREAD_LOCK(state);

    if (priority <= IDLE_PRIORITY)
        priority = IDLE_PRIORITY + 1;
    if (priority > HIGHEST_PRIORITY)
        priority = HIGHEST_PRIORITY;
    current_thread->priority = priority;

    current_thread->state = THREAD_READY;
    insert_in_run_queue_head(current_thread);
    thread_resched();

    THREAD_UNLOCK(state);
}

/**
 * @brief  Become an idle thread
 *
 * This function marks the current thread as the idle thread -- the one which
 * executes when there is nothing else to do.  This function does not return.
 * This function is called once at boot time.
 */
void thread_become_idle(void) {
    DEBUG_ASSERT(arch_ints_disabled());

    thread_t *t = get_current_thread();

#if WITH_SMP
    char name[16];
    snprintf(name, sizeof(name), "idle %d", arch_curr_cpu_num());
    thread_set_name(name);
#else
    thread_set_name("idle");
#endif

    /* mark ourself as idle */
    t->priority = IDLE_PRIORITY;
    t->flags |= THREAD_FLAG_IDLE;
    thread_set_pinned_cpu(t, arch_curr_cpu_num());

    mp_set_curr_cpu_active(true);
    mp_set_cpu_idle(arch_curr_cpu_num());

    /* enable interrupts and start the scheduler */
    arch_enable_ints();
    thread_yield();

    idle_thread_routine();
}

/* create an idle thread for the cpu we're on, and start scheduling */

void thread_secondary_cpu_init_early(void) {
    DEBUG_ASSERT(arch_ints_disabled());

    /* construct an idle thread to cover our cpu */
    uint cpu = arch_curr_cpu_num();
    thread_t *t = idle_thread(cpu);

    char name[16];
    snprintf(name, sizeof(name), "idle %u", cpu);
    init_thread_struct(t, name);
    thread_set_pinned_cpu(t, cpu);

    /* half construct this thread, since we're already running */
    t->priority = HIGHEST_PRIORITY;
    t->state = THREAD_RUNNING;
    t->flags = THREAD_FLAG_DETACHED | THREAD_FLAG_IDLE;
    thread_set_curr_cpu(t, cpu);
    thread_set_pinned_cpu(t, cpu);
    wait_queue_init(&t->retcode_wait_queue);

    THREAD_LOCK(state);

    list_add_head(&thread_list, &t->thread_list_node);
    set_current_thread(t);

    THREAD_UNLOCK(state);
}

void thread_secondary_cpu_entry(void) {
    uint cpu = arch_curr_cpu_num();
    thread_t *t = get_current_thread();
    t->priority = IDLE_PRIORITY;

    mp_set_curr_cpu_active(true);
    mp_set_cpu_idle(cpu);

    /* enable interrupts and start the scheduler on this cpu */
    arch_enable_ints();
    thread_yield();

    idle_thread_routine();
}

static const char *thread_state_to_str(enum thread_state state) {
    switch (state) {
        case THREAD_SUSPENDED:
            return "susp";
        case THREAD_READY:
            return "rdy";
        case THREAD_RUNNING:
            return "run";
        case THREAD_BLOCKED:
            return "blok";
        case THREAD_SLEEPING:
            return "slep";
        case THREAD_DEATH:
            return "deth";
        default:
            return "unkn";
    }
}

static size_t thread_stack_used(thread_t *t) {
#ifdef THREAD_STACK_HIGHWATER
    uint8_t *stack_base;
    size_t stack_size;
    size_t i;

    stack_base = t->stack;
    stack_size = t->stack_size;

    for (i = 0; i < stack_size; i++) {
        if (stack_base[i] != STACK_DEBUG_BYTE)
            break;
    }
    return stack_size - i;
#else
    return 0;
#endif
}
/**
 * @brief  Dump debugging info about the specified thread.
 */
void dump_thread(thread_t *t) {
    dprintf(INFO, "dump_thread: t %p (%s)\n", t, t->name);
#if WITH_SMP
    dprintf(INFO, "\tstate %s, curr_cpu %d, pinned_cpu %d, priority %d, remaining quantum %d\n",
            thread_state_to_str(t->state), t->curr_cpu, t->pinned_cpu, t->priority, t->remaining_quantum);
#else
    dprintf(INFO, "\tstate %s, priority %d, remaining quantum %d\n",
            thread_state_to_str(t->state), t->priority, t->remaining_quantum);
#endif
#ifdef THREAD_STACK_HIGHWATER
    dprintf(INFO, "\tstack %p, stack_size %zd, stack_used %zd\n",
            t->stack, t->stack_size, thread_stack_used(t));
#else
    dprintf(INFO, "\tstack %p, stack_size %zd\n", t->stack, t->stack_size);
#endif
    dprintf(INFO, "\tentry %p, arg %p, flags 0x%x\n", t->entry, t->arg, t->flags);
    dprintf(INFO, "\twait queue %p, wait queue ret %d\n", t->blocking_wait_queue, t->wait_queue_block_ret);
#if WITH_KERNEL_VM
    dprintf(INFO, "\taspace %p\n", t->aspace);
#endif
#if (MAX_TLS_ENTRY > 0)
    dprintf(INFO, "\ttls:");
    int i;
    for (i=0; i < MAX_TLS_ENTRY; i++) {
        dprintf(INFO, " 0x%lx", t->tls[i]);
    }
    dprintf(INFO, "\n");
#endif
    arch_dump_thread(t);
}

void dump_all_threads_unlocked(void) {
    thread_t *t;
    list_for_every_entry(&thread_list, t, thread_t, thread_list_node) {
        if (t->magic != THREAD_MAGIC) {
            dprintf(INFO, "bad magic on thread struct %p, aborting.\n", t);
            hexdump(t, sizeof(thread_t));
            break;
        }
        dump_thread(t);
    }
}

/**
 * @brief  Dump debugging info about all threads
 */
void dump_all_threads(void) {
    THREAD_LOCK(state);
    dump_all_threads_unlocked();
    THREAD_UNLOCK(state);
}

#if THREAD_STATS
void dump_threads_stats(void) {
    thread_t *t;

    THREAD_LOCK(state);
    list_for_every_entry (&thread_list, t, thread_t, thread_list_node) {
        if (t->magic != THREAD_MAGIC) {
            dprintf(INFO, "bad magic on thread struct %p, aborting.\n", t);
            hexdump(t, sizeof(thread_t));
            break;
        }
        if (thread_is_idle(t)) {
            continue;
        }
        // thread specific stats
        dprintf(INFO, "\t(%s):\n", t->name);
        dprintf(INFO, "\t\tScheduled: %ld\n", t->stats.schedules);
        uint percent = (t->stats.total_run_time * 10000) / current_time_hires();
        dprintf(INFO, "\t\tTotal run time: %lld, %u.%02u%%\n", t->stats.total_run_time,
                percent / 100, percent % 100);
        dprintf(INFO, "\t\tLast time run: %lld\n", t->stats.last_run_timestamp);
    }
    THREAD_UNLOCK(state);
}
#endif

/** @} */


/**
 * @defgroup  wait  Wait Queue
 * @{
 */
void wait_queue_init(wait_queue_t *wait) {
    *wait = (wait_queue_t)WAIT_QUEUE_INITIAL_VALUE(*wait);
}

static enum handler_return wait_queue_timeout_handler(timer_t *timer, lk_time_t now, void *arg) {
    thread_t *thread = (thread_t *)arg;

    DEBUG_ASSERT(thread->magic == THREAD_MAGIC);

    spin_lock(&thread_lock);

    enum handler_return ret = INT_NO_RESCHEDULE;
    if (thread_unblock_from_wait_queue(thread, ERR_TIMED_OUT) >= NO_ERROR) {
        ret = INT_RESCHEDULE;
    }

    spin_unlock(&thread_lock);

    return ret;
}

/**
 * @brief  Block until a wait queue is notified.
 *
 * This function puts the current thread at the end of a wait
 * queue and then blocks until some other thread wakes the queue
 * up again.
 *
 * @param  wait     The wait queue to enter
 * @param  timeout  The maximum time, in ms, to wait
 *
 * If the timeout is zero, this function returns immediately with
 * ERR_TIMED_OUT.  If the timeout is INFINITE_TIME, this function
 * waits indefinitely.  Otherwise, this function returns with
 * ERR_TIMED_OUT at the end of the timeout period.
 *
 * @return ERR_TIMED_OUT on timeout, else returns the return
 * value specified when the queue was woken by wait_queue_wake_one().
 */
status_t wait_queue_block(wait_queue_t *wait, lk_time_t timeout) {
    timer_t timer;

    thread_t *current_thread = get_current_thread();

    DEBUG_ASSERT(wait->magic == WAIT_QUEUE_MAGIC);
    DEBUG_ASSERT(current_thread->state == THREAD_RUNNING);
    DEBUG_ASSERT(arch_ints_disabled());
    DEBUG_ASSERT(spin_lock_held(&thread_lock));

    if (timeout == 0)
        return ERR_TIMED_OUT;

    list_add_tail(&wait->list, &current_thread->queue_node);
    wait->count++;
    current_thread->state = THREAD_BLOCKED;
    current_thread->blocking_wait_queue = wait;
    current_thread->wait_queue_block_ret = NO_ERROR;

    /* if the timeout is nonzero or noninfinite, set a callback to yank us out of the queue */
    if (timeout != INFINITE_TIME) {
        timer_initialize(&timer);
        timer_set_oneshot(&timer, timeout, wait_queue_timeout_handler, (void *)current_thread);
    }

    thread_resched();

    /* we don't really know if the timer fired or not, so it's better safe to try to cancel it */
    if (timeout != INFINITE_TIME) {
        timer_cancel(&timer);
    }

    return current_thread->wait_queue_block_ret;
}

/**
 * @brief  Wake up one thread sleeping on a wait queue
 *
 * This function removes one thread (if any) from the head of the wait queue and
 * makes it executable.  The new thread will be placed at the head of the
 * run queue.
 *
 * @param wait  The wait queue to wake
 * @param reschedule  If true, the newly-woken thread will run immediately.
 * @param wait_queue_error  The return value which the new thread will receive
 * from wait_queue_block().
 *
 * @return  The number of threads woken (zero or one)
 */
int wait_queue_wake_one(wait_queue_t *wait, bool reschedule, status_t wait_queue_error) {
    thread_t *t;
    int ret = 0;

    thread_t *current_thread = get_current_thread();

    DEBUG_ASSERT(wait->magic == WAIT_QUEUE_MAGIC);
    DEBUG_ASSERT(arch_ints_disabled());
    DEBUG_ASSERT(spin_lock_held(&thread_lock));

    t = list_remove_head_type(&wait->list, thread_t, queue_node);
    if (t) {
        wait->count--;
        DEBUG_ASSERT(t->state == THREAD_BLOCKED);
        t->state = THREAD_READY;
        t->wait_queue_block_ret = wait_queue_error;
        t->blocking_wait_queue = NULL;

        /* if we're instructed to reschedule, stick the current thread on the head
         * of the run queue first, so that the newly awakened thread gets a chance to run
         * before the current one, but the current one doesn't get unnecessarilly punished.
         */
        if (reschedule) {
            current_thread->state = THREAD_READY;
            insert_in_run_queue_head(current_thread);
        }
        insert_in_run_queue_head(t);
        wakeup_cpu_for_thread(t);
        if (reschedule) {
            thread_resched();
        }
        ret = 1;

    }

    return ret;
}


/**
 * @brief  Wake all threads sleeping on a wait queue
 *
 * This function removes all threads (if any) from the wait queue and
 * makes them executable.  The new threads will be placed at the head of the
 * run queue.
 *
 * @param wait  The wait queue to wake
 * @param reschedule  If true, the newly-woken threads will run immediately.
 * @param wait_queue_error  The return value which the new thread will receive
 * from wait_queue_block().
 *
 * @return  The number of threads woken (zero or one)
 */
int wait_queue_wake_all(wait_queue_t *wait, bool reschedule, status_t wait_queue_error) {
    thread_t *t;
    int ret = 0;
    uint32_t cpu_mask = 0;

    thread_t *current_thread = get_current_thread();

    DEBUG_ASSERT(wait->magic == WAIT_QUEUE_MAGIC);
    DEBUG_ASSERT(arch_ints_disabled());
    DEBUG_ASSERT(spin_lock_held(&thread_lock));

    if (reschedule && wait->count > 0) {
        /* if we're instructed to reschedule, stick the current thread on the head
         * of the run queue first, so that the newly awakened threads get a chance to run
         * before the current one, but the current one doesn't get unnecessarilly punished.
         */
        current_thread->state = THREAD_READY;
        insert_in_run_queue_head(current_thread);
    }

    /* pop all the threads off the wait queue into the run queue */
    while ((t = list_remove_head_type(&wait->list, thread_t, queue_node))) {
        wait->count--;
        DEBUG_ASSERT(t->state == THREAD_BLOCKED);
        t->state = THREAD_READY;
        t->wait_queue_block_ret = wait_queue_error;
        t->blocking_wait_queue = NULL;
        int pinned_cpu = thread_pinned_cpu(t);
        if (pinned_cpu < 0) {
            /* assumes MP_CPU_ALL_BUT_LOCAL is defined as all bits on */
            cpu_mask = MP_CPU_ALL_BUT_LOCAL;
        } else {
            cpu_mask |= (1U << pinned_cpu);
        }
        insert_in_run_queue_head(t);
        ret++;
    }

    DEBUG_ASSERT(wait->count == 0);

    if (ret > 0) {
        mp_reschedule(cpu_mask, 0);
        if (reschedule) {
            thread_resched();
        }
    }

    return ret;
}

/**
 * @brief  Free all resources allocated in wait_queue_init()
 *
 * If any threads were waiting on this queue, they are all woken.
 */
void wait_queue_destroy(wait_queue_t *wait, bool reschedule) {
    DEBUG_ASSERT(wait->magic == WAIT_QUEUE_MAGIC);
    DEBUG_ASSERT(arch_ints_disabled());
    DEBUG_ASSERT(spin_lock_held(&thread_lock));

    wait_queue_wake_all(wait, reschedule, ERR_OBJECT_DESTROYED);
    wait->magic = 0;
}

/**
 * @brief  Wake a specific thread in a wait queue
 *
 * This function extracts a specific thread from a wait queue, wakes it, and
 * puts it at the head of the run queue.
 *
 * @param t  The thread to wake
 * @param wait_queue_error  The return value which the new thread will receive
 *   from wait_queue_block().
 *
 * @return ERR_NOT_BLOCKED if thread was not in any wait queue.
 */
status_t thread_unblock_from_wait_queue(thread_t *t, status_t wait_queue_error) {
    DEBUG_ASSERT(t->magic == THREAD_MAGIC);
    DEBUG_ASSERT(arch_ints_disabled());
    DEBUG_ASSERT(spin_lock_held(&thread_lock));

    if (t->state != THREAD_BLOCKED)
        return ERR_NOT_BLOCKED;

    DEBUG_ASSERT(t->blocking_wait_queue != NULL);
    DEBUG_ASSERT(t->blocking_wait_queue->magic == WAIT_QUEUE_MAGIC);
    DEBUG_ASSERT(list_in_list(&t->queue_node));

    list_delete(&t->queue_node);
    t->blocking_wait_queue->count--;
    t->blocking_wait_queue = NULL;
    t->state = THREAD_READY;
    t->wait_queue_block_ret = wait_queue_error;
    insert_in_run_queue_head(t);
    wakeup_cpu_for_thread(t);

    return NO_ERROR;
}
