/**
 * @file ibt2.h
 * @author Darren Garnier (garnier@mit.edu)
 * @brief Handle control of IBT-2 bridge module
 * @version 0.1
 * @date 2021-04-03
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#include "i2c.h"
#include "pca9685.h"

class IBT2 {

    public:
        // different h-bridge modes
        // see for example
        // http://modularcircuits.tantosonline.com/blog/articles/h-bridge-secrets/
        enum Mode
        {
            SIGN_MAGN_LO,
            SIGN_MAGN_HI,
            LOCK_ANTIPHASE,
            ASYNC_SIGN_MAG_LO,  // not supported yet
            ASYNC_SIGN_MAG_HI,
            ASYNC_LOCK_ANTIPHASE
        };

        IBT2(PCA9685 pwm, Mode mode,
             uint8_t ch_R_EN = 0,
             uint8_t ch_L_EN = 1,
             uint8_t ch_R_PWM = 2,
             uint8_t ch_L_PWM = 3
            );

        virtual ~IBT2(void);

        void setDutyCycle(float dutyCycle);
        void enable(void);
        void disable(void);
        void powerDown(void);


    protected:
        PCA9685 _pwm;
        Mode _mode;
        uint8_t _R_EN;
        uint8_t _L_EN;
        uint8_t _R_PWM;
        uint8_t _L_PWM;
        uint8_t _first;
        uint8_t _first_EN;
        uint8_t _first_PWM;
        bool _enabled;
        int16_t _setting;
};
extern "C" {
#include "ibt2_c.h"
}
