#ifndef ___ADS1X15_H___
#ifndef __ADS1X15_H__

#include "i2c.h"

#define ADS1X15_ADDRESS (0x48) ///< 1001 000 (ADDR = GND)
#define ADS1X15_REG_CONFIG_OS_MASK (0x8000) ///< OS Mask
#define ADS1X15_REG_CONFIG_OS_SINGLE                                           \
	  (0x8000) ///< Write: Set to start a single-conversion
#define ADS1X15_REG_CONFIG_OS_BUSY                                             \
	  (0x0000) ///< Read: Bit = 0 when conversion is in progress
#define ADS1X15_REG_CONFIG_OS_NOTBUSY                                          \
	  (0x8000) ///< Read: Bit = 1 when device is not performing a conversion

#define ADS1X15_REG_CONFIG_MUX_MASK (0x7000) ///< Mux Mask
#define ADS1X15_REG_CONFIG_MUX_DIFF_0_1                                        \
	  (0x0000) ///< Differential P = AIN0, N = AIN1 (default)
#define ADS1X15_REG_CONFIG_MUX_DIFF_0_3                                        \
	  (0x1000) ///< Differential P = AIN0, N = AIN3
#define ADS1X15_REG_CONFIG_MUX_DIFF_1_3                                        \
	  (0x2000) ///< Differential P = AIN1, N = AIN3
#define ADS1X15_REG_CONFIG_MUX_DIFF_2_3                                        \
	  (0x3000) ///< Differential P = AIN2, N = AIN3
#define ADS1X15_REG_CONFIG_MUX_SINGLE_0 (0x4000) ///< Single-ended AIN0
#define ADS1X15_REG_CONFIG_MUX_SINGLE_1 (0x5000) ///< Single-ended AIN1
#define ADS1X15_REG_CONFIG_MUX_SINGLE_2 (0x6000) ///< Single-ended AIN2
#define ADS1X15_REG_CONFIG_MUX_SINGLE_3 (0x7000) ///< Single-ended AIN3

#define ADS1X15_REG_CONFIG_PGA_MASK (0x0E00)   ///< PGA Mask
#define ADS1X15_REG_CONFIG_PGA_6_144V (0x0000) ///< +/-6.144V range = Gain 2/3
#define ADS1X15_REG_CONFIG_PGA_4_096V (0x0200) ///< +/-4.096V range = Gain 1
#define ADS1X15_REG_CONFIG_PGA_2_048V                                          \
	  (0x0400) ///< +/-2.048V range = Gain 2 (default)
#define ADS1X15_REG_CONFIG_PGA_1_024V (0x0600) ///< +/-1.024V range = Gain 4
#define ADS1X15_REG_CONFIG_PGA_0_512V (0x0800) ///< +/-0.512V range = Gain 8
#define ADS1X15_REG_CONFIG_PGA_0_256V (0x0A00) ///< +/-0.256V range = Gain 16

#define ADS1X15_REG_CONFIG_MODE_MASK (0x0100)   ///< Mode Mask
#define ADS1X15_REG_CONFIG_MODE_CONTIN (0x0000) ///< Continuous conversion mode
#define ADS1X15_REG_CONFIG_MODE_SINGLE                                         \
	  (0x0100) ///< Power-down single-shot mode (default)

#define ADS1X15_REG_CONFIG_RATE_MASK (0x00E0) ///< Data Rate Mask

#define ADS1X15_REG_CONFIG_CMODE_MASK (0x0010) ///< CMode Mask
#define ADS1X15_REG_CONFIG_CMODE_TRAD                                          \
	  (0x0000) ///< Traditional comparator with hysteresis (default)
#define ADS1X15_REG_CONFIG_CMODE_WINDOW (0x0010) ///< Window comparator

#define ADS1X15_REG_CONFIG_CPOL_MASK (0x0008) ///< CPol Mask
#define ADS1X15_REG_CONFIG_CPOL_ACTVLOW                                        \
	  (0x0000) ///< ALERT/RDY pin is low when active (default)
#define ADS1X15_REG_CONFIG_CPOL_ACTVHI                                         \
	  (0x0008) ///< ALERT/RDY pin is high when active

#define ADS1X15_REG_CONFIG_CLAT_MASK                                           \
	  (0x0004) ///< Determines if ALERT/RDY pin latches once asserted
#define ADS1X15_REG_CONFIG_CLAT_NONLAT                                         \
	  (0x0000) ///< Non-latching comparator (default)
#define ADS1X15_REG_CONFIG_CLAT_LATCH (0x0004) ///< Latching comparator

#define ADS1X15_REG_CONFIG_CQUE_MASK (0x0003) ///< CQue Mask
#define ADS1X15_REG_CONFIG_CQUE_1CONV                                          \
	  (0x0000) ///< Assert ALERT/RDY after one conversions
#define ADS1X15_REG_CONFIG_CQUE_2CONV                                          \
	  (0x0001) ///< Assert ALERT/RDY after two conversions
#define ADS1X15_REG_CONFIG_CQUE_4CONV                                          \
	  (0x0002) ///< Assert ALERT/RDY after four conversions
#define ADS1X15_REG_CONFIG_CQUE_NONE                                           \
	  (0x0003) ///< Disable the comparator and put ALERT/RDY in high state (default)

/** Gain settings */
typedef enum {
	  GAIN_TWOTHIRDS = ADS1X15_REG_CONFIG_PGA_6_144V,
	  GAIN_ONE = ADS1X15_REG_CONFIG_PGA_4_096V,
	  GAIN_TWO = ADS1X15_REG_CONFIG_PGA_2_048V,
	  GAIN_FOUR = ADS1X15_REG_CONFIG_PGA_1_024V,
	  GAIN_EIGHT = ADS1X15_REG_CONFIG_PGA_0_512V,
	  GAIN_SIXTEEN = ADS1X15_REG_CONFIG_PGA_0_256V
} adsGain_t;
#endif

#define TwoWire struct i2c_t

void  *Adafruit_ADS1115_new();
void  Adafruit_ADS1115_delete(void * ctx);
void  Adafruit_ADS1115_begin(void * ctx, uint8_t i2c_addr, TwoWire *wire);
void  Adafruit_ADS1115_setGain(void * ctx, adsGain_t gain); // to 4.096V max
void  Adafruit_ADS1115_startADC_SingleEnded(void * ctx, uint8_t channel);
float Adafruit_ADS1115_computeVolts(void * ctx, int16_t value);
int16_t Adafruit_ADS1115_getLastConversionResults(void * ctx);
#endif
