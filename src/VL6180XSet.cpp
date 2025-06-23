#include <unistd.h>
#include <string.h>
#include <stdio.h>
#include "VL6180XSet.h"
#include "Adafruit_VL6180X.h"

// use pigpio for gpio and i2c functions


// lets use standard linux userland libraries for gpio and i2c
// libgpiod  AND calls to i2c dev ports

//#include <gpiod.h>
//#include <pigpio.h>
#include "tiny_gpio.h"

#include <linux/i2c-dev.h>
#include <fcntl.h>

class VL6180XSet {
    int i2c_bus_fd;
    uint8_t ndev;
    uint8_t gpios[4];
    class Adafruit_VL6180X * devs[4];
public:
    VL6180XSet(uint8_t i2c_bus_no, uint8_t ndev, const uint8_t *gpios);
    void start(void);
    void read(uint8_t *buff);
};

#define RESET 0

#define ENABLE_HWDEBUG_PIN gpioSetMode(18, PI_OUTPUT);
#define HWDEBUG_LO         gpioWrite(18,0);
#define HWDEBUG_HI         gpioWrite(18,1);

VL6180XSet::VL6180XSet(uint8_t i2c_bus_no, uint8_t ndev, const uint8_t *gpios)
{
    int fd;
    char devFileName[20];

    // lets open the i2c bus first
    snprintf(devFileName, sizeof(devFileName), "/dev/i2c-%d", i2c_bus_no);
    fd = open(devFileName, O_RDWR);
    if (fd == -1)
    {
        perror("Open I2C bus failed\n");
        return;
    }
    // should we check if it is capable of I2C calls?  (I2C_FUNCS ?)

    this->i2c_bus_fd = fd;
    this->ndev = ndev;

    // now lets put all of the devices in reset.
    // stupid british spelling.
    gpioInitialise();

    // debug pin
    ENABLE_HWDEBUG_PIN
    HWDEBUG_LO

    for (int i = 0; i < ndev; i++)
    {
        this->gpios[i] = gpios[i];
        gpioWrite(gpios[i], RESET);
        gpioSetMode(gpios[i], PI_OUTPUT);
    }
    usleep(2000);

    for (int i = 0; i < ndev; i++)
    {
        uint8_t addr;
        gpioWrite(gpios[i], ~RESET);
        usleep(2000);
        devs[i] = new Adafruit_VL6180X();
        devs[i]->begin(this->i2c_bus_fd);
        addr = devs[i]->getAddress();
        devs[i]->setAddress(addr + i + 1);
    }
}

#define SINGLE_SHOT 1

void VL6180XSet::start(void)
{
    #if SINGLE_SHOT
    for (int i = 0; i < ndev; i++) {
        devs[i]->startRange();        
    }
    #else 
    static bool started = false;
    if (!started) {
        for (int i = 0; i < ndev; i++) {
            devs[i]->startRangeContinuous(10);        
        }
        started = true;
    }
    #endif
    HWDEBUG_LO
}

void VL6180XSet::read(uint8_t *buffer)
{
    HWDEBUG_HI
    #if SINGLE_SHOT
    for(int i=0; i<ndev; i++) {
        // check for a bad one and give 0?  Or just give the last?
//        buffer[i] = (devs[i]->isRangeComplete()) ? devs[i]->readRangeResult() : 0;
        buffer[i] = devs[i]->readRangeResult();
        // devs[i]->stopRangeContinuous();
    }
    #else
    for (int i = 0; i < ndev; i++)
    {
        // check for a bad one and give 0?  Or just give the last?
        //        buffer[i] = (devs[i]->isRangeComplete()) ? devs[i]->readRangeResult() : 0;
        buffer[i] = devs[i]->readRangeResult();
        // devs[i]->stopRangeContinuous();
    }
    #endif
}

extern "C"
{
    struct VL6180XSet *VL6180XSet_Setup(uint8_t i2c_bus, uint8_t ndev, const uint8_t *gpios)
    {
        return new VL6180XSet(i2c_bus, ndev, gpios);
    }

    void VL6180XSet_Start(struct VL6180XSet *handle)
    {
        handle->start();
    }

    void VL6180XSet_Read(struct VL6180XSet *handle, uint8_t *buffer)
    {
        handle->read(buffer);
    }
}