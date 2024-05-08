// (c)2012 by Texas Instruments Incorporated, All Rights Reserved.

#ifndef SYSTEM_I2C_H_
#define SYSTEM_I2C_H_
#ifndef XT1HFOFFG               // Defines if not available in header file
#define XT1HFOFFG   0
#endif
#define st(x)      do { x } while (__LINE__ == -1)
#define SELECT_MCLK_SMCLK(sources) st(UCSCTL4 = (UCSCTL4 & ~(SELM_7 + SELS_7)) | (sources);)
#include <stdint.h>
int16_t Device_portSetup(void);
int16_t Device_clockSetup(void);
void Init_FLL_Settle(uint16_t fsystem, uint16_t ratio);
void Init_FLL(uint16_t fsystem, uint16_t ratio);
int16_t System_i2c_UCB0_init(void);
int16_t System_i2c_UCB0_isPresent(uint8_t slaveAddress);
int16_t System_i2c_UCB0_write(uint8_t slaveAddress, uint8_t regAddress, const uint8_t *writeBuffer, int16_t writeBufferSize);
int16_t System_i2c_UCB0_write_16bSubAddr(uint8_t slaveAddress, uint16_t regAddress, const uint8_t *writeBuffer, int16_t writeBufferSize);
int16_t System_i2c_UCB0_read(uint8_t slaveAddress, uint8_t regAddress, uint8_t *readBuffer, int16_t readBufferSize);
int16_t System_i2c_UCB0_read_16bSubAddr(uint8_t slaveAddress, uint16_t regAddress, uint8_t *readBuffer, int16_t readBufferSize);
int16_t System_i2c_UCB1_init(void);

#endif /* SYSTEM_I2C_H_ */
