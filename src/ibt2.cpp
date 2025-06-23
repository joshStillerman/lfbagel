/**
 * @file ibt2.cpp
 * @author Darren Garnier (garnier@mit.edu)
 * @brief 
 * @version 0.1
 * @date 2021-04-03
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#include <stdint.h>
#include <unistd.h>
#include <algorithm>
#include <stdio.h>
#include "ibt2.h"

/**
 * @brief Construct a new IBT2::IBT2 object
 * 
 * @param pwm 
 * @param mode 
 * @param ch_R_EN 
 * @param ch_L_EN 
 * @param ch_R_PWM 
 * @param ch_L_PWM 
 */
IBT2::IBT2  (PCA9685 pwm, IBT2::Mode mode,
             uint8_t ch_R_EN,  uint8_t ch_L_EN,
             uint8_t ch_R_PWM, uint8_t ch_L_PWM
            )
{
    _pwm = pwm;
    _first_EN = std::min(ch_R_EN, ch_L_EN);
    _first_PWM = std::min(ch_R_PWM, ch_L_PWM);
    _first = std::min(_first_EN, _first_PWM);

    // check channel layouts
    {
        bool contiguous, contiguous_en, contiguous_pwm;

        contiguous = 3 == -_first + 
                    std::max(std::max(ch_R_EN, ch_L_EN), std::max(ch_R_PWM, ch_R_PWM));
        contiguous_en = abs(ch_R_EN - ch_L_EN) == 1;
        contiguous_pwm =  abs(ch_R_PWM - ch_L_PWM) == 1;

        if (!contiguous || !contiguous_en || !contiguous_pwm) {
            perror("Need to have contiguous EN, and PWM channels");
        }
    }
    
    _R_PWM = 2 * (ch_R_PWM - _first_PWM);
    _L_PWM = 2 * (ch_L_PWM - _first_PWM);
    _R_EN  = 2 * (ch_R_EN  - _first_EN);
    _L_EN  = 2 * (ch_L_EN  - _first_EN);
    
    if (mode >= ASYNC_SIGN_MAG_LO)
        perror("Mode not handled yet.");

    _mode = mode;
    _enabled = false;
    powerDown();
}

IBT2::~IBT2()
{
    // leave in powered down state
    // its best if this is powered down until current drops to zero
    // before disabling.
    powerDown();
    disable();
}

void IBT2::powerDown(void)
{
    uint16_t on_off[] = {0, 4096, 0, 4096};
    if (_mode == LOCK_ANTIPHASE) {
        PCA9685_setPWMs_contiguous(_pwm, 2, _first_PWM, on_off);
    } else {
        setDutyCycle(0);
    }
}

void IBT2::disable(void)
{
    uint16_t on_off[] = {0, 4096, 0, 4096};
    PCA9685_setPWMs_contiguous(_pwm, 2, _first_EN, on_off);
    _enabled = false;
}

void IBT2::enable(void)
{
    uint16_t on_off[4];
    
    switch (_mode) {
        //case LOCK_ANTIPHASE:
        //    on_off[_R_PWM] = 0;
        //    on_off[_R_PWM + 1] = 2048;
        //    on_off[_L_PWM] = 2048;
        //    on_off[_L_PWM + 1] = 0;
        //    PCA9685_setPWMs_contiguous(_pwm, 2, _first_PWM, on_off);
        //    break;
        case SIGN_MAGN_HI:
            on_off[_R_PWM] = 4096;
            on_off[_R_PWM + 1] = 0;
            on_off[_L_PWM] = 4096;
            on_off[_L_PWM + 1] = 0;
            PCA9685_setPWMs_contiguous(_pwm, 2, _first_PWM, on_off);
            break;
        case LOCK_ANTIPHASE:
        case SIGN_MAGN_LO:
            on_off[_R_PWM] = 0;
            on_off[_R_PWM + 1] = 4096;
            on_off[_L_PWM] = 0;
            on_off[_L_PWM + 1] = 4096;
            PCA9685_setPWMs_contiguous(_pwm, 2, _first_PWM, on_off);
            break;
        case ASYNC_SIGN_MAG_LO:  // not supported yet
        case ASYNC_SIGN_MAG_HI:
        case ASYNC_LOCK_ANTIPHASE:
        default:
            break;
    }

    on_off[_R_EN] = 4096;
    on_off[_R_EN + 1] = 0;
    on_off[_L_EN] = 4096;
    on_off[_L_EN + 1] = 0;
    PCA9685_setPWMs_contiguous(_pwm, 2, _first_EN, on_off);
    _enabled = true;
}

void IBT2::setDutyCycle(float value)
{
    //
    int16_t ival;
    uint16_t on_off[8];
    uint16_t break_point;

    ival = (int16_t)(value * 4096);

    if (ival >= 4096) {  // full on.. all modes the same
        on_off[_R_PWM] = 4096;
        on_off[_R_PWM + 1] = 0;
        on_off[_L_PWM] = 0;
        on_off[_L_PWM + 1] = 4096;
        ival = 4096;
        PCA9685_setPWMs_contiguous(_pwm, 2, _first_PWM, on_off);
    } 
    else if (ival <= -4096) // full on negative
    {
        on_off[_R_PWM] = 0;
        on_off[_R_PWM + 1] = 4096;
        on_off[_L_PWM] = 4096;
        on_off[_L_PWM + 1] = 0;
        ival = -4096;
        PCA9685_setPWMs_contiguous(_pwm, 2, _first_PWM, on_off);
    }
    else // real PWM.. 
    {
        switch (_mode) {
            case LOCK_ANTIPHASE:
                break_point = 2048 - ival/2;
                on_off[_L_PWM]     = 0;
                on_off[_L_PWM + 1] = break_point;
                on_off[_R_PWM]     = break_point;
                on_off[_R_PWM + 1] = 0;
#if 1
                // the second one always misses first pulse with 
                // PCA9685.. kinda a bummer.. makes this noisy
                // and inaccurate with rapid changes.
                PCA9685_setPWMs_contiguous(_pwm, 2, _first_PWM, on_off);
#else
                PCA9685_setPWM(_pwm, _first_PWM + 1, break_point, 0);
                PCA9685_setPWM(_pwm, _first_PWM, 0, break_point);
#endif
                break;
            case SIGN_MAGN_HI:
                if (ival > 0) {
                    on_off[_R_PWM] = 4096;
                    on_off[_R_PWM + 1] = 0;
                    on_off[_L_PWM] = ival;
                    on_off[_L_PWM + 1] = 0;
                } else if (ival < 0) {
                    on_off[_R_PWM] = ival;
                    on_off[_R_PWM + 1] = 0;
                    on_off[_L_PWM] = 4096;
                    on_off[_L_PWM + 1] = 0;
                }
                else
                {
                    on_off[_R_PWM] = 4096;
                    on_off[_R_PWM + 1] = 0;
                    on_off[_L_PWM] = 4096;
                    on_off[_L_PWM + 1] = 0;
                }
                PCA9685_setPWMs_contiguous(_pwm, 2, _first_PWM, on_off);
                break;
            case SIGN_MAGN_LO:
                if (ival > 0)
                {
                    on_off[_R_PWM] = 0;
                    on_off[_R_PWM + 1] = ival;
                    on_off[_L_PWM] = 0;
                    on_off[_L_PWM + 1] = 4096;
                }
                else if (ival < 0)
                {
                    on_off[_R_PWM] = 0;
                    on_off[_R_PWM + 1] = 4096;
                    on_off[_L_PWM] = 0;
                    on_off[_L_PWM + 1] = -ival;
                }
                else
                {
                    on_off[_R_PWM] = 0;
                    on_off[_R_PWM + 1] = 4096;
                    on_off[_L_PWM] = 0;
                    on_off[_L_PWM + 1] = 4096;
                }
                PCA9685_setPWMs_contiguous(_pwm, 2, _first_PWM, on_off);
                break;
            default:
                break;
        }
    }
    _setting = ival;
}
extern "C" {
	void  *IBT2_new(PCA9685 pwm, IBT2_Mode mode, uint8_t ch_R_EN,  
			uint8_t ch_L_EN,uint8_t ch_R_PWM, uint8_t ch_L_PWM)
	{
			return (void *)new IBT2(pwm, (IBT2::Mode)mode, ch_R_EN,  
					        ch_L_EN, ch_R_PWM, ch_L_PWM);
	}
	void  IBT2_delete(void * ctx)
	{
			delete (IBT2 *) ctx;
	}
        void  IBT2_powerDown(void * ctx)
	{
		((IBT2 *) ctx)->powerDown();

	}
	void  IBT2_enable(void * ctx)
        {
	                ((IBT2 *) ctx)->enable();	        
	}       
	void  IBT2_disable(void * ctx)
        {
	                ((IBT2 *) ctx)->disable();
        }       
	void  IBT2_setDutyCycle(void *ctx, float magnet_demand)
        {
		        ((IBT2 *) ctx)->setDutyCycle(magnet_demand); 
	}
}	
