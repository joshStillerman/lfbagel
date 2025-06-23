#include <unistd.h>
#include <stdint.h>
#include <ev.h>
#include <time.h>
#include <iostream>
#ifndef HAVE_PTHREAD_H
#define HAVE_PTHREAD_H 1
#endif
#include <mdsobjects.h>
#include <pthread.h>
#include <sched.h>

// hardware stuff
#include "i2c.h"
#include "pca9685.h"
#include "ibt2.h"
//#include "Adafruit_ADS1X15.h"
//#include "VL6180XSet.h"
//include <pigpio.h>
#include "tiny_gpio.h"

#define POLARITY (-1)

//#define EVENT_HEIGHTS "BAGEL_HEIGHTS"
#define EVENT_DEMAND "LIFT_COIL_DEMAND"
//#define EVENT_CURRENT "LIFT_COIL_CURRENT"

#define CURRENT_CHANNEL 0
#define TOF_I2C_CHAN 3
#define TOF_NUM 4
const uint8_t TOF_GPIOS[] = {16, 17, 25, 27};

pthread_mutex_t lock;
struct ev_timer watchdog_watch;

IBT2 *magnet;
//Adafruit_ADS1115 *adc;
PCA9685 pwm;
//struct VL6180XSet *tofs = NULL;

bool powered_up = false;
float magnet_demand = 0.0;

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
    magnet->powerDown();
    ev_break(l, EVBREAK_ALL);
}

static void
setup_hardware(void)
{
    // hardware is a IBT2 connected to the first 4 channels of a PCA9685
    //
    // current is measured with a ACS712-20A connected to a ADS1115
    // (sensitivity = 100 mV/A => 10A/V)
    // https://www.allegromicro.com/~/media/Files/Datasheets/ACS712-Datasheet.ashx
    // https://cdn-shop.adafruit.com/datasheets/ads1115.pdf
    //
    i2c bus;
    bus = i2c_open("/dev/i2c-1");
    pwm = PCA9685_init(bus, 0x40);
    PCA9685_setFreq(pwm, 1500);

    // tofs = VL6180XSet_Setup(TOF_I2C_CHAN, TOF_NUM, TOF_GPIOS);

#if !PWM_TEST

#define PWM_TEST 0

    // magnet = new IBT2(pwm, IBT2::LOCK_ANTIPHASE);
#if POLARITY > 0
    magnet = new IBT2(pwm, IBT2::SIGN_MAGN_LO);
#else
    magnet = new IBT2(pwm, IBT2::SIGN_MAGN_LO, 1, 0, 3, 2);
#endif

#else
    magnet = new IBT2(pwm, IBT2::SIGN_MAGN_HI, 4, 5, 7, 6);
#endif

    //adc = new Adafruit_ADS1115();
    //adc->begin(ADS1X15_ADDRESS, bus);
    //adc->setGain(GAIN_ONE); // to 4.096V max

    //VL6180XSet_Start(tofs);
    //usleep(6000);

    gpioInitialise();
    gpioSetMode(12, PI_OUTPUT);
}

//static void start_read_current(void)
//{
//    adc->startADC_SingleEnded(CURRENT_CHANNEL);
//}

//static float get_current(void)
//{
//    float volts;
//    volts = adc->computeVolts(adc->getLastConversionResults());
//    return POLARITY * 10 * (volts - 2.5) ;
//}

//static void send_data(const char *EVENT, double now, float *samples, uint8_t nsamp)
//{
//    uint64_t inow;
//    int dims[] = {nsamp};
//    int shot = 1; // dummy "shot" info
//    inow = (uint64_t)(now * 1000);
//    // MDSplus::EventStream::send(shot, EVENT, true, 1, (void *)&inow, nsamp, samples);
//    MDSplus::EventStream::send(shot, EVENT, true, 1, (void *)&inow, 1, dims, samples);
//}

static void
watchdog_reset(EV_P_ ev_async *w, int revents)
{
    float current;
    if (!powered_up)
    {
        powered_up = true;
        printf("Powered up!\n");
    }
    ev_now_update(loop);
    ev_timer_again(EV_A_ & watchdog_watch);

}

static void
watchdog_fired(EV_P_ ev_timer *w, int revents)
{
    // watchdog fired.. reset the outputs!
    if (powered_up)
    {
        printf("Powerdown.\n");
        magnet->powerDown();
        magnet_demand = 0;
        powered_up = false;
    }
}

class DemandListener : public MDSplus::DataStreamListener
{
protected:
    struct ev_loop *_loop;
    struct ev_async *_watchdog;

public:
    DemandListener(struct ev_loop *loop, struct ev_async *watchdog)
    {
        _loop = loop;
        _watchdog = watchdog;
    }

    void dataReceived(MDSplus::Data *samples, MDSplus::Data *times, int shot)
    {
        // gpioWrite(12,1); // end of timing pulse (up)
        pthread_mutex_lock(&lock); //Don't forget locking
        magnet_demand = samples->getFloat();
        pthread_mutex_unlock(&lock);
        // reset the watchdog in the other thread because can't mix them except for this mechanism.
        if (ev_async_pending(_watchdog) == false)
        {
            //the event has not yet been processed (or even noted) by the event loop? (i.e. Is it serviced? If yes then proceed to)
            ev_async_send(_loop, _watchdog); //Sends/signals/activates the given ev_async watcher, that is, feeds an EV_ASYNC event on the watcher into the event loop.
        }
    }
};


static void
periodic_task(struct ev_loop *loop, ev_timer *w, int revents)
{
    // here's where the whole loop happens!
    // lets get the data.. then start a new data gather.. then send the data
    float now;
    float samples[4];
    float current;
    uint8_t raw[4];
    uint64_t nowi;
    // MDSplus::Uint8Array *data;
    gpioWrite(12, 1);           // start the timing pulse.. (down)

    //current = get_current();
    //start_read_current();

    //VL6180XSet_Read(tofs, raw);
    //for (int i = 0; i < 4; i++)
    //{
    //    samples[i] = raw[i];
    //}
    //send_data(EVENT_CURRENT, ev_now(loop), &current, 1);
    //send_data(EVENT_HEIGHTS, ev_now(loop), samples, 4);

    magnet->setDutyCycle(magnet_demand);

    //start_read_current();

    //const struct timespec t200_us = { 0, 200000};

    const ev_tstamp t200_us = .0002;
    // usleep(200);
    //nanosleep(&t200_us, NULL);
    ev_sleep(t200_us);

    //VL6180XSet_Start(tofs);
    gpioWrite(12, 0);           // start the timing pulse.. (down)
}

int main(int argc, char *argv[])
{
    MDSplus::EventStream *stream;
    const uint8_t cpus[] = {2, 3};
    struct ev_signal signal_watcher;
    struct ev_async async_watch;
    struct ev_timer periodic_timer;
    DemandListener *listener;

    struct ev_loop *loop = ev_default_loop(EVBACKEND_IOURING);
    //struct ev_loop *loop = ev_default_loop(0);

    // run on its own cpu
    run_on_isolated_cpus(cpus, sizeof(cpus));

    // do the hardware setup
    setup_hardware();
    // enable the magnet
    magnet->enable();

    ev_signal_init(&signal_watcher, sigtermhandler, SIGINT);
    ev_signal_start(loop, &signal_watcher);

    ev_now_update(loop);

    ev_async_init(&async_watch, watchdog_reset);
    ev_async_start(loop, &async_watch);

    ev_timer_init(&watchdog_watch, watchdog_fired, 0, 0.05);
    ev_timer_again(loop, &watchdog_watch);

    listener = new DemandListener(loop, &async_watch);

    stream = new MDSplus::EventStream(EVENT_DEMAND);
    stream->registerListener(listener, EVENT_DEMAND);
    stream->start();

    ev_timer_init(&periodic_timer, periodic_task, 0.015, 0.010);
    ev_timer_start(loop, &periodic_timer);

    ev_run(loop, 0);
    stream->stop();
    magnet->powerDown();
    ev_sleep(.02);
    magnet->disable();

    printf("Exiting normally after magnet was shut off.\n");
}
