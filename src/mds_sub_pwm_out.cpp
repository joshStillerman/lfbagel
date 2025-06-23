#include <unistd.h>
#include <stdint.h>
#include <ev.h>
#include <iostream>
#ifdef NDEBUG
#undef NDEBUG
#endif

#ifndef HAVE_PTHREAD_H
#define HAVE_PTHREAD_H 1
#endif

#include <mdsobjects.h>
#include <pthread.h>
#include <sched.h>

#include "i2c.h"
#include "pca9685.h"
#include "ibt2.h"
#include "Adafruit_ADS1X15.h"

#include <pigpio.h>

char MDSPLUS_IN_EVENT[] = "LIFT_COIL_DEMAND";
char MDSPLUS_OUT_EVENT[] = "LIFT_COIL_CURRENT";
#define CURRENT_CHANNEL 0


pthread_mutex_t lock; 
struct ev_timer watchdog_watch;

IBT2 *magnet;
Adafruit_ADS1115 *adc;
PCA9685 pwm;

bool powered_up = false;

double start_time;

static void
run_on_isolated_cpus(const uint8_t *cpus, uint8_t ncpus)
{
    cpu_set_t set;
    CPU_ZERO(&set);

    for (int i=0; i<ncpus; i++)
        CPU_SET(cpus[i], &set);

    if (sched_setaffinity(getpid(), sizeof(set), &set) == -1)
        perror("Couldn't set affinity with sched_setaffinity");
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
    #if !PWM_TEST

    #define PWM_TEST 0

    // magnet = new IBT2(pwm, IBT2::LOCK_ANTIPHASE);
    magnet = new IBT2(pwm, IBT2::SIGN_MAGN_LO);
    #else
 //   magnet = new IBT2(pwm, IBT2::SIGN_MAGN_HI, 4,5,7,6);  
    magnet = new IBT2(pwm, IBT2::SIGN_MAGN_HI, 0,1,3,2);

    #endif

    adc = new Adafruit_ADS1115();
    adc->begin(ADS1X15_ADDRESS, bus);
    adc->setGain(GAIN_ONE);  // to 4.096V max 

    gpioInitialise();
    gpioSetMode(12, PI_OUTPUT);
}

static void start_read(void)
{
    adc->startADC_SingleEnded(CURRENT_CHANNEL);
}

static float read_current(void)
{
    float volts;
    volts = adc->computeVolts(adc->getLastConversionResults());
    return (volts-2.5)*10;
}

static void send_data(double now, float *samples, uint8_t nsamp)
{
    uint64_t inow;
    float fnow;
    int dims[] = {nsamp};
    int shot = 1; // dummy "shot" info

    inow = (uint64_t)(now * 1000);
    fnow = (float)(now - start_time);
    //MDSplus::EventStream::send(shot, MDSPLUS_OUT_EVENT, false, 1, (void *)&fnow,
    //                           1, dims, samples);
    MDSplus::EventStream::send(shot, MDSPLUS_OUT_EVENT, true, 1, (void *)&inow,
                               1, dims, samples);
}

static void 
watchdog_reset(EV_P_ ev_async *w, int revents)
{
    float current;
    if (!powered_up) {
        powered_up = true;
        printf("Powered up!\n");
    }
    current = read_current();
    ev_now_update(loop);
    send_data(ev_now(loop), &current, 1);
    ev_timer_again(EV_A_ & watchdog_watch);
    start_read();
    gpioWrite(12, 0);
}

static void 
watchdog_fired(EV_P_ ev_timer *w, int revents)
{
    // watchdog fired.. reset the outputs!
    if (powered_up) {
        pthread_mutex_lock(&lock); //Don't forget locking
        printf("Powerdown.\n");
        magnet->powerDown();
        powered_up = false;
        pthread_mutex_unlock(&lock);
    }
}

class DemandListener : public MDSplus::DataStreamListener
{
protected:
    struct ev_loop * loop;
    struct ev_async * async_watch;

public:
    DemandListener(struct ev_loop *loop, struct ev_async *watchdog)
    {
        this->loop = loop;
        this->async_watch = watchdog;
    }

    void dataReceived(MDSplus::Data *samples, MDSplus::Data *times, int shot)
    {
        // std::cout << "Channel: " << channelName << "   Time: " << times << "  Value: " << samples << std::endl;
        // set the outputs
        gpioWrite(12, 1);
        // reset the watchdog in the other thread because can't mix them except for this mechanism.
        if (ev_async_pending(async_watch) == false) {                                         
            //the event has not yet been processed (or even noted) by the event loop? (i.e. Is it serviced? If yes then proceed to)
            ev_async_send(loop, async_watch); //Sends/signals/activates the given ev_async watcher, that is, feeds an EV_ASYNC event on the watcher into the event loop.
        }
        // now lets set the output... which should be a float from -1 to 1.

        //pthread_mutex_lock(&lock); //Don't forget locking

        //std::vector<float> values = samples->getFloat();
        //magnet->setDutyCycle(values[0]);
        
        magnet->setDutyCycle(samples->getFloat());

        // std::cout << "setting output to " << samples->getFloat() << "\n";

        //pthread_mutex_unlock(&lock);
    }
};


int main(int argc, char *argv[])
{
    MDSplus::EventStream *stream; 
    const uint8_t cpus[] = {3};
    struct ev_signal signal_watcher;
    struct ev_async async_watch;
    DemandListener *listener;

    struct ev_loop *loop = ev_default_loop();

    run_on_isolated_cpus(cpus, sizeof(cpus));

    setup_hardware();
    magnet->enable();

    ev_signal_init(&signal_watcher, sigtermhandler, SIGINT);
    ev_signal_start(loop, &signal_watcher);

    ev_now_update(loop);
    start_time = ev_now(loop);

    ev_async_init(&async_watch, watchdog_reset);
    ev_async_start(loop, &async_watch);

    ev_timer_init(&watchdog_watch, watchdog_fired, 0, 0.05);
    ev_timer_again(loop, &watchdog_watch);
    
    listener = new DemandListener(loop, &async_watch);

    stream = new MDSplus::EventStream(MDSPLUS_IN_EVENT);
    stream->registerListener(listener, MDSPLUS_IN_EVENT);
    stream->start();

    ev_run (loop, 0);

    stream->stop();
    magnet->powerDown();
    magnet->disable();

    printf("Exiting normally after magnet was shut off.\n");
}
