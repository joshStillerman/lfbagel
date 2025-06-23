#ifndef _PCA9685_H_
#define _PCA9685_H_

#include "i2c.h"

struct sPCA9685;
typedef struct PCA9685_s *PCA9685;

#ifdef __cplusplus
extern "C" {
#endif

    PCA9685 PCA9685_init(i2c bus, uint8_t i2caddr);
    void PCA9685_setPWM(PCA9685 pwm, uint8_t channel, uint16_t on, uint16_t off);
    void PCA9685_setPWMs(PCA9685 pwm, uint8_t nchan, 
            const uint8_t *chans, const uint16_t *on, const uint16_t *off);
    void PCA9685_setPWMs_contiguous(PCA9685 pwm, uint8_t nchan, uint8_t first_chan, uint16_t *on_off); 
    void PCA9685_setFreq(PCA9685 pwm, uint16_t freq);
    void PCA9685_stop(PCA9685 pwm);

#ifdef __cplusplus
};
#endif
#endif
