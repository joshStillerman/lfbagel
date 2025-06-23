/**
 * @file ibt2_c.h
 * @author Joshua Stillerman (jas@mit.edu)
 * @brief C Wrapper for control of IBT-2 bridge module
 * @version 0.1
 * @date 2025-04-08
 * 
 * @copyright Copyright (c) 2025
 * 
 */

#ifndef __IBT2_C__
#define __IBT2_C__
#include "i2c.h"
#include "pca9685.h"
typedef enum IBT2_Mode
{
    SIGN_MAGN_LO,
    SIGN_MAGN_HI,
    LOCK_ANTIPHASE,
    ASYNC_SIGN_MAG_LO,  // not supported yet
    ASYNC_SIGN_MAG_HI,
    ASYNC_LOCK_ANTIPHASE
} IBT2_Mode;

void  *IBT2_new(PCA9685 pwm, IBT2_Mode mode, uint8_t ch_R_EN,  uint8_t ch_L_EN,uint8_t ch_R_PWM, uint8_t ch_L_PWM);
void  IBT2_delete(void * ctx);
void  IBT2_powerDown(void * ctx);
void  IBT2_enable(void * ctx);
void  IBT2_disable(void * ctx);
void  IBT2_setDutyCycle(void *ctx, float magnet_demand);
#endif
