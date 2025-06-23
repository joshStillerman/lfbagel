#include "pca9685.h"
#include <arpa/inet.h>
#include <string.h>

// REGISTER ADDRESSES
#define PCA9685_MODE1 0x00		/**< Mode Register 1 */
#define PCA9685_MODE2 0x01		/**< Mode Register 2 */
#define PCA9685_SUBADR1 0x02	/**< I2C-bus subaddress 1 */
#define PCA9685_SUBADR2 0x03	/**< I2C-bus subaddress 2 */
#define PCA9685_SUBADR3 0x04	/**< I2C-bus subaddress 3 */
#define PCA9685_ALLCALLADR 0x05 /**< LED All Call I2C-bus address */
#define PCA9685_LED0_ON_L 0x06	/**< LED0 on tick, low byte*/
#define PCA9685_LED0_ON_H 0x07	/**< LED0 on tick, high byte*/
#define PCA9685_LED0_OFF_L 0x08 /**< LED0 off tick, low byte */
#define PCA9685_LED0_OFF_H 0x09 /**< LED0 off tick, high byte */
// etc all 16:  LED15_OFF_H 0x45
#define PCA9685_ALLLED_ON_L 0xFA  /**< load all the LEDn_ON registers, low */
#define PCA9685_ALLLED_ON_H 0xFB  /**< load all the LEDn_ON registers, high */
#define PCA9685_ALLLED_OFF_L 0xFC /**< load all the LEDn_OFF registers, low */
#define PCA9685_ALLLED_OFF_H 0xFD /**< load all the LEDn_OFF registers,high */
#define PCA9685_PRESCALE 0xFE	  /**< Prescaler for PWM output frequency */
#define PCA9685_TESTMODE 0xFF	  /**< defines the test mode to be entered */

#define PCA9685_REGISTERS_PER_CHANNEL 4
#define PCA9685_ADDRESS 0x40

// MODE1 bits
#define MODE1_ALLCAL 0x01  /**< respond to LED All Call I2C-bus address */
#define MODE1_SUB3 0x02	   /**< respond to I2C-bus subaddress 3 */
#define MODE1_SUB2 0x04	   /**< respond to I2C-bus subaddress 2 */
#define MODE1_SUB1 0x08	   /**< respond to I2C-bus subaddress 1 */
#define MODE1_SLEEP 0x10   /**< Low power mode. Oscillator off */
#define MODE1_AI 0x20	   /**< Auto-Increment enabled */
#define MODE1_EXTCLK 0x40  /**< Use EXTCLK pin clock */
#define MODE1_RESTART 0x80 /**< Restart enabled */
// MODE2 bits
#define MODE2_OUTNE_0 0x01 /**< Active LOW output enable input */
#define MODE2_OUTNE_1 \
	0x02				  /**< Active LOW output enable input - high impedience */
#define MODE2_OUTDRV 0x04 /**< totem pole structure vs open-drain */
#define MODE2_OCH 0x08	  /**< Outputs change on ACK vs STOP */
#define MODE2_INVRT 0x10  /**< Output logic state inverted */

#define PCA9685_I2C_ADDRESS 0x40	  /**< Default PCA9685 I2C Slave Address */
#define FREQUENCY_OSCILLATOR 25000000 /**< Int. osc. frequency in datasheet */

#define PCA9685_PRESCALE_MIN 3	 /**< minimum prescale value */
#define PCA9685_PRESCALE_MAX 255 /**< maximum prescale value */

struct PCA9685_s {
	i2c bus;
	uint8_t addr;
};

PCA9685 PCA9685_init(i2c bus, uint8_t addr) {

	PCA9685 pwm = (PCA9685) malloc(sizeof(struct PCA9685_s));
	if(addr == 0)
		addr = PCA9685_ADDRESS;
	pwm->bus = bus;
	pwm->addr = addr;

	i2c_reg8_write8(bus, addr, PCA9685_MODE1, MODE1_AI);
	i2c_reg8_write8(bus, addr, PCA9685_MODE2, MODE2_OUTDRV);
	
	return pwm;
}

void PCA9685_setPWMs(PCA9685 pwm, uint8_t nchan,
					 const uint8_t *chans, const uint16_t *on, const uint16_t *off)
{
	uint8_t regs[nchan];
	uint32_t values[nchan];
	int i;
	for(i=0; i<nchan; i++) {
		regs[i] = PCA9685_LED0_ON_L + (PCA9685_REGISTERS_PER_CHANNEL * chans[i]);
		values[i] = (((uint32_t) htons(on[i])) << 16) + htons(off[i]);
	}

	i2c_reg8_write32_n(pwm->bus, pwm->addr, nchan, regs, values);
}

/**
 * @brief set PWM start stops using contiguous channels, minimzing communication
 * 
 * @param pwm 
 * @param nchan - number of channels to set
 * @param first_chan - lowest channel number
 * @param on_off - array of 16bit start stop values for each channel.
 */
void PCA9685_setPWMs_contiguous(PCA9685 pwm, uint8_t nchan, uint8_t first_chan, uint16_t *on_off)
{
	uint8_t buf[1+nchan*4];
	buf[0] = PCA9685_LED0_ON_L + (PCA9685_REGISTERS_PER_CHANNEL * first_chan);
	memcpy(buf+1, on_off, nchan*4);
	i2c_write(pwm->bus, pwm->addr, buf, 9);
}

void PCA9685_setPWM(PCA9685 pwm, uint8_t channel, uint16_t on, uint16_t off)
{
	uint8_t buf[5];
	buf[0] = PCA9685_LED0_ON_L + (PCA9685_REGISTERS_PER_CHANNEL * channel);
	buf[1] = on  & 0xFF; buf[2] = (on  >> 8) & 0xFF;
	buf[3] = off & 0xFF; buf[4] = (off >> 8) & 0xFF;

	return i2c_write(pwm->bus, pwm->addr, buf, 5);
}

void PCA9685_setFreq(PCA9685 pwm, uint16_t freq)
{
	uint8_t prescale;
	uint8_t oldmode, newmode;

	prescale = freq < 24? 0xFF:
			freq > 1526? 0x03:
			25000000 / (4096 * freq);
	

	oldmode = i2c_reg8_read8(pwm->bus, pwm->addr, PCA9685_MODE1);
	newmode = (oldmode & ~MODE1_RESTART) | MODE1_SLEEP; // sleep
	i2c_reg8_write8(pwm->bus, pwm->addr, PCA9685_MODE1, newmode); // go to sleep
	i2c_reg8_write8(pwm->bus, pwm->addr, PCA9685_PRESCALE, prescale); // set the prescaler
	i2c_reg8_write8(pwm->bus, pwm->addr, PCA9685_MODE1, oldmode);
	usleep(5000);
	// This sets the MODE1 register to turn on auto increment.
	i2c_reg8_write8(pwm->bus, pwm->addr,PCA9685_MODE1, oldmode | MODE1_RESTART | MODE1_AI);
}

void PCA9685_stop(PCA9685 pwm) {
	return i2c_reg8_write8(pwm->bus, pwm->addr, PCA9685_MODE1, MODE1_SLEEP);
}
