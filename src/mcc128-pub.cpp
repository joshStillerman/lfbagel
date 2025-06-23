#include <cstdint>
#include <unistd.h>
#include <stdint.h>
#include <ev.h>
#include <time.h>
#include <iostream>
#include "daqhats_utils.h"


#ifndef HAVE_PTHREAD_H
#define HAVE_PTHREAD_H 1
#endif
#ifdef NDEBUG
#undef NDEBUG
#include <mdsobjects.h>
#else
#include <mdsobjects.h>
#undef NDEBUG
#endif
#include <pthread.h>
#include <sched.h>

// hardware stuff
//include <pigpio.h>
#include "tiny_gpio.h"


#define EVENT_PASSIVE "BAGEL_PASSIVE"

pthread_mutex_t lock;

static void
run_on_isolated_cpus(const uint8_t *cpus, uint8_t ncpus)
{
    cpu_set_t set;
    CPU_ZERO(&set);

    for (int i = 0; i < ncpus; i++)
        CPU_SET(cpus[i], &set);

    if (sched_setaffinity(getpid(), sizeof(set), &set) == -1)
        perror("Couldn't set affinity with sched_setaffinity");

    struct sched_param params;

    sched_getparam(0, &params);
    params.sched_priority = sched_get_priority_max(SCHED_RR);
    sched_setscheduler(0, SCHED_RR, &params);

}

void sigtermhandler(struct ev_loop *l, ev_signal *w, int revents)
{
    printf("SIGTERM handler called in process %d\n", getpid());
    ev_break(l, EVBREAK_ALL);
}

static uint8_t address;
static uint32_t options = OPTS_DEFAULT;
#define STOP_ON_ERR(result)\
{\
    if (result != RESULT_SUCCESS ){\
        print_error(result);\
        exit(1);\
    }\
}

static void setup_hardware(void)
{
    // The first 3 double ended inputs will be 
    // - 1 the double full coil around the magnet
    // - 2 the first figure 8 coil
    // - 3 the second figure 8 coil

    uint8_t address;
    uint32_t options = OPTS_DEFAULT;
    double value;
    char mode_string[32];
    char range_string[32];

    uint8_t low_chan = 0;
    uint8_t high_chan = 2;

    char display_string[256] = "";
    char c;

    int result = RESULT_SUCCESS;
    int samples_per_channel = 0;
    int sample_interval = 500;  // ms
    uint8_t input_mode = A_IN_MODE_DIFF;
    uint8_t input_range = A_IN_RANGE_BIP_1V;

    int mcc128_num_channels = mcc128_info()->NUM_AI_CHANNELS[input_mode];

    // Ensure low_chan and high_chan are valid
    if ((low_chan >= mcc128_num_channels) ||
        (high_chan >= mcc128_num_channels))
        {
            fprintf(stderr, "Error: Invalid channel - must be 0 - %d.\n",
                mcc128_num_channels - 1);
            exit(-1);
    }
        if (select_hat_device(HAT_ID_MCC_128, &address) != 0)
    {
        exit(-1);
    }

    // Open a connection to each device
    result = mcc128_open(address);
    STOP_ON_ERR(result);

    result = mcc128_a_in_mode_write(address, input_mode);
    STOP_ON_ERR(result);

    result = mcc128_a_in_range_write(address, input_range);
    STOP_ON_ERR(result);
    gpioInitialise();
}
static uint32_t get_inputs(float *buf)
{
    uint8_t channel;
    for (channel = 0; channel <= 2; channel++)
    {
        unsigned int result;
	double value;
        result = mcc128_a_in_read(address, channel, options, &value); 
        STOP_ON_ERR(result);
        buf[channel] = value;
    }
    return 0;
}

static void send_data(const char *EVENT, ev_tstamp  now, float *samples, uint8_t nsamp)
{
    uint64_t inow;
    int dims[] = {nsamp};
    int shot = 1; // dummy "shot" info
    inow = (uint64_t)(now * 1000);
    MDSplus::EventStream::send(shot, EVENT, true, 1, (void *)&inow, 1, dims, samples);
}// end send_data

static void periodic_task(struct ev_loop *loop, ev_timer *w, int revents)
{
    // here's where the whole loop happens!
    // lets get the data.. then start a new data gather.. then send the data
    ev_tstamp now;
    float samples[3];
    static uint16_t cycle_count=0;
    gpioWrite(12, 1);           // start the timing pulse.. (down)
    if (get_inputs(samples) != 0)
    {
        printf("Error reading inputs\n");
        return;
    }
    now = ev_now(loop);
    send_data(EVENT_PASSIVE, now, samples, 3);
    gpioWrite(12, 0);           // start the timing pulse.. (down)
    cycle_count++;
}

int main(int argc, char *argv[])
{
    const uint8_t cpus[] = {2, 3};
    struct ev_signal signal_watcher;
    struct ev_async async_watch;
    struct ev_timer periodic_timer;

    struct ev_loop *loop = ev_default_loop(EVBACKEND_IOURING);

    // run on its own cpu
    run_on_isolated_cpus(cpus, sizeof(cpus));

    // do the hardware setup
    setup_hardware();
    // enable the magnet

    ev_signal_init(&signal_watcher, sigtermhandler, SIGINT);
    ev_signal_start(loop, &signal_watcher);

    ev_now_update(loop);

    ev_timer_init(&periodic_timer, periodic_task, 0.015, 0.001);
    ev_timer_start(loop, &periodic_timer);

    ev_run(loop, 0);

    printf("Exiting normally.\n");
}
