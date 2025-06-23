#ifndef _VL6180XSET_H_
#define _VL6180XSET_H_

#include <stdint.h>

#ifdef  __cplusplus
extern "C" {
#endif

struct VL6180XSet;

struct VL6180XSet *VL6180XSet_Setup(uint8_t i2c_bus, uint8_t ndev, const uint8_t *gpios);
void VL6180XSet_Start(struct VL6180XSet *handle);
void VL6180XSet_Read(struct VL6180XSet *handle, uint8_t *buffer);

#ifdef __cplusplus
}
#endif
#endif
