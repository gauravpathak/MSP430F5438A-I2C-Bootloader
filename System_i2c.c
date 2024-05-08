// (c)2012 by Texas Instruments Incorporated, All Rights Reserved.

// =============================================================================
// System_i2c.c
// -----------------------------------------------------------------------------
// Description:
//      TODO
// Version Information:
// 		v1.0.0 / Oct 4, 2012
//			- Initial Release.
// =============================================================================

// =================================================================== Includes
#include <msp430.h>
#include <stdint.h>
#include "System_i2c.h"

// ==================================================================== Defines
#define     STATUS_PASS         0
#define     STATUS_FAIL         1
#define     TIMEOUT_CNT     50000
#define PASS 0
// -----------------------------------------------------------------------------
// Description:
//      Configures the MSP430 I/O ports.
// -----------------------------------------------------------------------------
int16_t Device_portSetup(void)
{
    int16_t status = PASS;



    // Initialize all pins
    P4OUT = 0x00;
    P4DIR = 0xFF &
          ~(BIT7 | BIT6 |       // Keep UCB0 I2C lines floating
            BIT5 | BIT4);       // Keep UCB1 UART lines floating
    // Re-map P4 as I2C/SPI pins
    PMAPPWD = 0x02D52;                          // Enable Write-access to modify port mapping registers
    PMAPCTL = PMAPRECFG;                        // Allow reconfiguration during runtime
    P4MAP0 = PM_UCA0CLK;                        // P4.0 = UCA0CLK   (SPI)
    P4MAP1 = PM_UCA0SIMO;                       // P4.1 = UCA0SIMO  (SPI)
    P4MAP2 = PM_UCA0SOMI;                       // P4.2 = UCA0SOMI  (SPI)
    P4MAP4 = PM_UCA1TXD;                        // P4.4 = UCA1TX    (UART)
    P4MAP5 = PM_UCA1RXD;                        // P4.5 = UCA1RX    (UART)
    P4MAP6 = PM_UCB0SCL;                        // P4.6 = UCB0SCL   (I2C)
    P4MAP7 = PM_UCB0SDA;                        // P4.7 = UCB0SDA   (I2C)
    PMAPPWD = 0;                                // Disable Write-Access to modify port mapping registers

    // P4 Configuration
    P4SEL = 0x00 |                              // P4: Configure for Port Mapping function
           (BIT7 | BIT6 |
            BIT5 | BIT4 |
            BIT2 | BIT1 | BIT0 );

    P4OUT |= BIT3;                              // P4: Configure P4.3 pull-up for ADBM SHTDWN

    return status;
}
void Init_FLL_Settle(uint16_t fsystem, uint16_t ratio)
{
  volatile uint16_t x = ratio * 32;

  Init_FLL(fsystem, ratio);

  while (x--) {
   __delay_cycles(30);
  }
}

void Init_FLL(uint16_t fsystem, uint16_t ratio)
{
  uint16_t d, dco_div_bits;
  uint16_t mode = 0;

  // Save actual state of FLL loop control, then disable it. This is needed to
  // prevent the FLL from acting as we are making fundamental modifications to
  // the clock setup.
  uint16_t srRegisterState = __get_SR_register() & SCG0;
  __bic_SR_register(SCG0);

  d = ratio;
  dco_div_bits = FLLD__2;        // Have at least a divider of 2

  if (fsystem > 16000) {
    d >>= 1 ;
    mode = 1;
  }
  else {
    fsystem <<= 1;               // fsystem = fsystem * 2
  }

  while (d > 512) {
    dco_div_bits = dco_div_bits + FLLD0;  // Set next higher div level
    d >>= 1;
  }

  UCSCTL0 = 0x0000;              // Set DCO to lowest Tap

  UCSCTL2 &= ~(0x03FF);          // Reset FN bits
  UCSCTL2 = dco_div_bits | (d - 1);

  if (fsystem <= 630)            //           fsystem < 0.63MHz
	UCSCTL1 = DCORSEL_0;
  else if (fsystem <  1250)      // 0.63MHz < fsystem < 1.25MHz
	UCSCTL1 = DCORSEL_1;
  else if (fsystem <  2500)      // 1.25MHz < fsystem <  2.5MHz
	UCSCTL1 = DCORSEL_2;
  else if (fsystem <  5000)      // 2.5MHz  < fsystem <    5MHz
	UCSCTL1 = DCORSEL_3;
  else if (fsystem <  10000)     // 5MHz    < fsystem <   10MHz
	UCSCTL1 = DCORSEL_4;
  else if (fsystem <  20000)     // 10MHz   < fsystem <   20MHz
	UCSCTL1 = DCORSEL_5;
  else if (fsystem <  40000)     // 20MHz   < fsystem <   40MHz
	UCSCTL1 = DCORSEL_6;
  else
	UCSCTL1 = DCORSEL_7;

  while (SFRIFG1 & OFIFG) {                               // Check OFIFG fault flag
    UCSCTL7 &= ~(DCOFFG+XT1LFOFFG+XT1HFOFFG+XT2OFFG);     // Clear OSC flaut Flags
    SFRIFG1 &= ~OFIFG;                                    // Clear OFIFG fault flag
  }

  if (mode == 1) {                              		  // fsystem > 16000
    SELECT_MCLK_SMCLK(SELM__DCOCLK + SELS__DCOCLK);       // Select DCOCLK
  }
  else {
    SELECT_MCLK_SMCLK(SELM__DCOCLKDIV + SELS__DCOCLKDIV); // Select DCODIVCLK
  }

  __bis_SR_register(srRegisterState);	                  // Restore previous SCG0
}

int16_t Device_clockSetup(void)
{
    int16_t status = PASS;

    // Configure FLL Input
    UCSCTL3 = (UCSCTL3 & ~(SELREF_7))
             | SELREF__REFOCLK;         // FLLREFCLK = REFOCLK
    UCSCTL3 = (UCSCTL3 & ~(FLLREFDIV_7))
             | FLLREFDIV__1;            // FLLREFDIV = 1

    // Select Clock Sources
    UCSCTL4 = (UCSCTL4 & ~(SELA_7))
             | SELA__DCOCLKDIV;         // ACLK = DCOCLKDIV
    UCSCTL4 = (UCSCTL4 & ~(SELS_7))
             | SELS__DCOCLKDIV;         // SMCLK = DCOCLKDIV
    UCSCTL4 = (UCSCTL4 & ~(SELM_7))
             | SELM__DCOCLKDIV;         // MCLK = DCOCLKDIV

    // Set Clock Dividers
    UCSCTL5 = (UCSCTL5 & ~(DIVPA_7))
             | DIVPA__1;                // DIVPA = 1
    UCSCTL5 = (UCSCTL5 & ~(DIVA_7))
             | DIVA__1;                 // DIVA = 1
    UCSCTL5 = (UCSCTL5 & ~(DIVS_7))
             | DIVS__1;                 // DIVS = 1
    UCSCTL5 = (UCSCTL5 & ~(DIVM_7))
             | DIVM__1;                 // DIVM = 1


    // Configure FLL Frequency
    Init_FLL_Settle(8000000 / 1000, 8000000 / 32768);

    // Loop until XT1,XT2 & DCO stabilizes
    do
    {
      UCSCTL7 &= ~(XT2OFFG + XT1LFOFFG + DCOFFG);
                                        // Clear XT2,XT1,DCO fault flags
      SFRIFG1 &= ~OFIFG;                // Clear fault flags
    }
    while (SFRIFG1&OFIFG);             // Test oscillator fault flag

    UCSCTL4 = (UCSCTL4 & ~(SELA_7))
             | SELA__XT2CLK;            // ACLK = XT2CLK

    return status;
}


// -----------------------------------------------------------------------------
// System_i2c_UCB0_init
// -----------------------------------------------------------------------------
// Description:
//      Initializes I2C Master Interface (USCI_B0)
// -----------------------------------------------------------------------------
int16_t System_i2c_UCB0_init(void)
{
    // UCB0 I2C Configuration
    UCB0CTL1 |= UCSWRST;                         // Enable SW reset
    // UCB0CTL0 = UCMST + UCMODE_3 + UCSYNC;     // I2C Master, synchronous mode
    UCB0CTL0 = UCMM + UCMST + UCMODE_3 + UCSYNC; // I2C Multi-Master

    UCB0CTL1 = UCSSEL_2 + UCSWRST;               // Use SMCLK, keep SW reset
    UCB0BR1 = 0;                                 // fSCL = SMCLK/20 = 400kHz
    UCB0BR0 = 20;
//    UCB0I2CSA = slaveAddress;                   // Slave Address
    UCB0CTL1 &= ~UCSWRST;                       // Clear SW reset, resume operation

    return STATUS_PASS;
}

// -----------------------------------------------------------------------------
// System_i2c_UCB1_init
// -----------------------------------------------------------------------------
// Description:
//      Initializes I2C Slave Interface (USCI_B1)
// -----------------------------------------------------------------------------
int16_t System_i2c_UCB1_init(void)
{
    // UCB0 I2C Configuration
    UCB1CTL1 |= UCSWRST;                        // Enable SW reset
    UCB1CTL0 = UCMODE_3 + UCSYNC;               // I2C Slave, synchronous mode
    UCB1CTL1 = UCSSEL_2 + UCSWRST;              // Use SMCLK, keep SW reset
    UCB1BR1 = 0;                                // fSCL = SMCLK/20 = 400kHz
    UCB1BR0 = 20;
    UCB1I2COA = 0x1C;                           // Own Address (0x38 left justified)
    UCB1CTL1 &= ~UCSWRST;                       // Clear SW reset, resume operation

    return STATUS_PASS;
}

// -----------------------------------------------------------------------------
// System_i2c_UCB0_isPresent
// -----------------------------------------------------------------------------
// Description:
//      Verifies if an I2C device is present. The I2C slaveAddress is assumed
//      as right justified.
// -----------------------------------------------------------------------------
int16_t System_i2c_UCB0_isPresent(uint8_t slaveAddress)
{
    UCB0IFG &= ~UCNACKIFG;                  // Clear any previous NACK
    UCB0IFG &= ~UCTXIFG;                    // Clear any previous UCTXIFG
    UCB0IFG &= ~UCRXIFG;                    // Clear any previous UCRXIFG
    UCB0I2CSA = slaveAddress;               // Program I2C Slave Address

    UCB0CTL1 |= UCTR + UCTXSTT;             // Send TX START and Slave Address
    while (!(UCB0IFG & UCTXIFG))            // Wait for UCTXIFG generated by UCTXSTT
    {
        if (UCB0IFG & UCNACKIFG)            // Check if NAK occurred
        {
            UCB0CTL1 |= UCTXSTP;            // I2C STOP condition
            UCB0IFG &= ~UCTXIFG;            // Clear UCTXIFG (no byte was sent)
            return STATUS_FAIL;             // Exit with failure
        }
    }
    UCB0CTL1 |= UCTXSTP;                    // I2C STOP condition
    UCB0IFG &= ~UCTXIFG;                    // Clear UCTXIFG (no byte was sent)
    while(UCB0CTL1 & UCTXSTP);              // Wait for I2C STOP condition

    return (UCB0IFG & UCNACKIFG);
}


// -----------------------------------------------------------------------------
// System_i2c_UCB0_write
// -----------------------------------------------------------------------------
// Description:
//      Low level generic I2C write for 8-bit register addresses. Refer to I2C
//      USCI section of MSP430 User's Guide for flag timing. The slaveAddress
//      input is the 7-bit I2C slave address (right justified). The regAddress
//      input is the first data byte sent through the bus. This function
//      returns 1 if NAK occurred.
// -----------------------------------------------------------------------------
int16_t System_i2c_write(uint8_t slaveAddress, uint8_t regAddress,
                              const uint8_t *writeBuffer,
                              int16_t writeBufferSize)
{
    int16_t i;
    uint16_t cnt;
    UCB0IFG &= ~UCNACKIFG;                  // Clear any previous NACK
    UCB0IFG &= ~UCTXIFG;                    // Clear any previous UCTXIFG
    UCB0IFG &= ~UCRXIFG;                    // Clear any previous UCRXIFG
    UCB0I2CSA = slaveAddress;               // Program I2C Slave Address

    UCB0CTL1 |= UCTR + UCTXSTT;             // Send TX START and Slave Address
    cnt = 0;
    while (!(UCB0IFG & UCTXIFG))            // Wait for UCTXIFG generated by UCTXSTT
    {
    	if(cnt<TIMEOUT_CNT)
    		cnt++;
    	else
    		return STATUS_FAIL;

        if (UCB0IFG & UCNACKIFG)            // Check if NAK occurred
        {
            UCB0CTL1 |= UCTXSTP;            // I2C STOP condition
            UCB0IFG &= ~UCTXIFG;            // Clear UCTXIFG (no byte was sent)
            return STATUS_FAIL;             // Exit with failure
        }
    }
    UCB0TXBUF = regAddress;                 // Send register byte during SLA/W
    cnt = 0;
    while (UCB0CTL1 & UCTXSTT)              // Wait for UCTXSTT to clear
    {
    	if(cnt<TIMEOUT_CNT)
    		cnt++;
    	else
    		return STATUS_FAIL;

        if (UCB0IFG & UCNACKIFG)            // Check if NAK occurred
        {
            UCB0CTL1 |= UCTXSTP;            // I2C STOP condition
            UCB0IFG &= ~UCTXIFG;            // Clear UCTXIFG (no byte was sent)
            return STATUS_FAIL;             // Exit with failure
        }
    }

    for(i=0; i<writeBufferSize; i++)
    {
    	cnt = 0;
        while (!(UCB0IFG & UCTXIFG))        // Wait for UCB0TXBUF to be ready
        {
        	if(cnt<TIMEOUT_CNT)
        		cnt++;
        	else
        		return STATUS_FAIL;

            if (UCB0IFG & UCNACKIFG)        // Check if NAK occurred
            {
                UCB0CTL1 |= UCTXSTP;        // I2C STOP condition
                UCB0IFG &= ~UCTXIFG;        // Clear UCTXIFG (no byte was sent)
                return STATUS_FAIL;         // Exit with failure
            }
        }
        UCB0TXBUF = writeBuffer[i];         // Send data byte
    }
    cnt = 0;
    while (!(UCB0IFG & UCTXIFG))            // Wait for UCB0TXBUF to be ready
    {
    	if(cnt<TIMEOUT_CNT)
    		cnt++;
    	else
    		return STATUS_FAIL;

        if (UCB0IFG & UCNACKIFG)            // Check if NAK occurred
        {
            UCB0CTL1 |= UCTXSTP;            // I2C STOP condition
            UCB0IFG &= ~UCTXIFG;            // Clear UCTXIFG (no byte was sent)
            return STATUS_FAIL;             // Exit with failure
        }
    }
    UCB0CTL1 |= UCTXSTP;                    // I2C STOP condition
    UCB0IFG &= ~UCTXIFG;                    // Clear UCTXIFG (no byte was sent)
    while(UCB0CTL1 & UCTXSTP);              // Wait for I2C STOP condition

    return (UCB0IFG & UCNACKIFG);
}

int16_t System_i2c_UCB0_write(uint8_t slaveAddress, uint8_t regAddress,
                              const uint8_t *writeBuffer,
                              int16_t writeBufferSize)
{
	int16_t state;

	state = System_i2c_write(slaveAddress, regAddress, writeBuffer, writeBufferSize);

	if(state == STATUS_FAIL) {
		System_i2c_UCB0_init();
		state = System_i2c_write(slaveAddress, regAddress, writeBuffer, writeBufferSize);
	}

	return state;
}

// -----------------------------------------------------------------------------
// System_i2c_UCB0_read
// -----------------------------------------------------------------------------
// Description:
//      Low level generic I2C read for 8-bit register addresses. Refer to I2C
//      USCI section of MSP430 User's Guide for flag timing. The slaveAddress
//      input is the 7-bit I2C slave address (right justified). The regAddress
//      input is the first data byte sent through the bus.
//      This routine performs a Repeated Start (Sr) between write and read.
//      This function returns 1 if NAK occurred.
// -----------------------------------------------------------------------------
int16_t System_i2c_read(uint8_t slaveAddress, uint8_t regAddress,
                             uint8_t *readBuffer, int16_t readBufferSize)
{

    int16_t i;
    uint16_t cnt;
    UCB0IFG &= ~UCNACKIFG;                  // Clear any previous NACK
    UCB0IFG &= ~UCTXIFG;                    // Clear any previous UCTXIFG
    UCB0IFG &= ~UCRXIFG;                    // Clear any previous UCRXIFG
    UCB0I2CSA = slaveAddress;               // Program I2C Slave Address

    // Write Phase
    UCB0CTL1 |= UCTR + UCTXSTT;             // Send TX START and Slave Address
    cnt = 0;
    while (!(UCB0IFG & UCTXIFG))            // Wait for UCTXIFG generated by UCTXSTT
    {
    	if(cnt<TIMEOUT_CNT)
    		cnt++;
    	else
    		return STATUS_FAIL;

        if (UCB0IFG & UCNACKIFG)            // Check if NAK occurred
        {
            UCB0CTL1 |= UCTXSTP;            // I2C STOP condition
            UCB0IFG &= ~UCTXIFG;            // Clear UCTXIFG (no byte was sent)
            return STATUS_FAIL;             // Exit with failure
        }
    }
    UCB0TXBUF = regAddress;                 // Send register byte during SLA/W
    cnt = 0;
    while (UCB0CTL1 & UCTXSTT)              // Wait for UCTXSTT to clear
    {
    	if(cnt<TIMEOUT_CNT)
    		cnt++;
    	else
    		return STATUS_FAIL;

        if (UCB0IFG & UCNACKIFG)            // Check if NAK occurred
        {
            UCB0CTL1 |= UCTXSTP;            // I2C STOP condition
            UCB0IFG &= ~UCTXIFG;            // Clear UCTXIFG (no byte was sent)
            return STATUS_FAIL;             // Exit with failure
        }
    }
    cnt = 0;
    while (!(UCB0IFG & UCTXIFG))            // Wait for UCB0TXBUF to be ready
    {
    	if(cnt<TIMEOUT_CNT)
    		cnt++;
    	else
    		return STATUS_FAIL;

        if (UCB0IFG & UCNACKIFG)            // Check if NAK occurred
        {
            UCB0CTL1 |= UCTXSTP;            // I2C STOP condition
            UCB0IFG &= ~UCTXIFG;            // Clear UCTXIFG (no byte was sent)
            return STATUS_FAIL;             // Exit with failure
        }
    }

    // Read Phase
    UCB0CTL1 &= ~UCTR;                      // Reconfigure for RX Mode
    UCB0CTL1 |= UCTXSTT;                    // Initiate repeated START (Sr)
    UCB0IFG &= ~UCTXIFG;                    // Clear UCTXIFG (no byte was sent)
    cnt = 0;
    while(UCB0CTL1 & UCTXSTT)               // Wait for UCTXSTT to clear
    {
    	if(cnt<TIMEOUT_CNT)
    		cnt++;
    	else
    		return STATUS_FAIL;

        if (UCB0IFG & UCNACKIFG)            // Check if NAK occurred
        {
            UCB0CTL1 |= UCTXSTP;            // I2C STOP condition
            UCB0IFG &= ~UCTXIFG;            // Clear UCTXIFG (no byte was sent)
            return STATUS_FAIL;             // Exit with failure
        }
    }
    for(i=0; i<(readBufferSize-1); i++)
    {
    	cnt = 0;
        while (!(UCB0IFG & UCRXIFG))        // Wait for UCB0RXBUF byte to arrive
        {
        	if(cnt<TIMEOUT_CNT)
        		cnt++;
        	else
        		return STATUS_FAIL;

            if (UCB0IFG & UCNACKIFG)        // Check if NAK occurred
            {
                UCB0CTL1 |= UCTXSTP;        // I2C STOP condition
                UCB0IFG &= ~UCTXIFG;        // Clear UCTXIFG (no byte was sent)
                return STATUS_FAIL;         // Exit with failure
            }
        }
        readBuffer[i] = UCB0RXBUF;          // Read data byte
    }
    UCB0CTL1 |= UCTXSTP;                    // I2C STOP condition
    cnt = 0;
    while (!(UCB0IFG & UCRXIFG))            // Wait for UCB0RXBUF byte to arrive
    {
    	if(cnt<TIMEOUT_CNT)
    		cnt++;
    	else
    		return STATUS_FAIL;

        if (UCB0IFG & UCNACKIFG)            // Check if NAK occurred
        {
            UCB0CTL1 |= UCTXSTP;            // I2C STOP condition
            UCB0IFG &= ~UCTXIFG;            // Clear UCTXIFG (no byte was sent)
            return STATUS_FAIL;             // Exit with failure
        }
    }
    readBuffer[readBufferSize-1]=UCB0RXBUF; // Read last data byte
    while(UCB0CTL1 & UCTXSTP);              // Wait for I2C STOP condition

    return ((UCB0IFG & UCNACKIFG));
}

int16_t System_i2c_UCB0_read(uint8_t slaveAddress, uint8_t regAddress,
                             uint8_t *readBuffer, int16_t readBufferSize)
{
	int16_t state;

	state = System_i2c_read(slaveAddress, regAddress, readBuffer, readBufferSize);

	if(state == STATUS_FAIL) {
		System_i2c_UCB0_init();
		state = System_i2c_read(slaveAddress, regAddress, readBuffer, readBufferSize);
	}

	return state;
}

//=============================================================================================================

// -----------------------------------------------------------------------------
// System_i2c_UCB0_write
// -----------------------------------------------------------------------------
// Description:
//      Low level generic I2C write for 8-bit register addresses. Refer to I2C
//      USCI section of MSP430 User's Guide for flag timing. The slaveAddress
//      input is the 7-bit I2C slave address (right justified). The regAddress
//      input is the first data byte sent through the bus. This function
//      returns 1 if NAK occurred.
// -----------------------------------------------------------------------------
int16_t System_i2c_write_16bSubAddr(uint8_t slaveAddress, uint16_t regAddress,
                              const uint8_t *writeBuffer,
                              int16_t writeBufferSize)
{
    int16_t i;
    uint16_t cnt;
    UCB0IFG &= ~UCNACKIFG;                  // Clear any previous NACK
    UCB0IFG &= ~UCTXIFG;                    // Clear any previous UCTXIFG
    UCB0IFG &= ~UCRXIFG;                    // Clear any previous UCRXIFG
    UCB0I2CSA = slaveAddress;               // Program I2C Slave Address

    UCB0CTL1 |= UCTR + UCTXSTT;             // Send TX START and Slave Address
    cnt = 0;
    while (!(UCB0IFG & UCTXIFG))            // Wait for UCTXIFG generated by UCTXSTT
    {
    	if(cnt<TIMEOUT_CNT)
    		cnt++;
    	else
    		return STATUS_FAIL;

        if (UCB0IFG & UCNACKIFG)            // Check if NAK occurred
        {
            UCB0CTL1 |= UCTXSTP;            // I2C STOP condition
            UCB0IFG &= ~UCTXIFG;            // Clear UCTXIFG (no byte was sent)
            return STATUS_FAIL;             // Exit with failure
        }
    }
    UCB0TXBUF = (uint8_t)(regAddress >> 8); // Send register H byte during SLA/W
    cnt = 0;
    while (UCB0CTL1 & UCTXSTT)              // Wait for UCTXSTT to clear
    {
    	if(cnt<TIMEOUT_CNT)
    		cnt++;
    	else
    		return STATUS_FAIL;

        if (UCB0IFG & UCNACKIFG)            // Check if NAK occurred
        {
            UCB0CTL1 |= UCTXSTP;            // I2C STOP condition
            UCB0IFG &= ~UCTXIFG;            // Clear UCTXIFG (no byte was sent)
            return STATUS_FAIL;             // Exit with failure
        }
    }

    for(i=0; i<=writeBufferSize; i++)
    {
    	cnt = 0;
        while (!(UCB0IFG & UCTXIFG))        // Wait for UCB0TXBUF to be ready
        {
        	if(cnt<TIMEOUT_CNT)
        		cnt++;
        	else
        		return STATUS_FAIL;

            if (UCB0IFG & UCNACKIFG)        // Check if NAK occurred
            {
                UCB0CTL1 |= UCTXSTP;        // I2C STOP condition
                UCB0IFG &= ~UCTXIFG;        // Clear UCTXIFG (no byte was sent)
                return STATUS_FAIL;         // Exit with failure
            }
        }
        if(i == 0)
        	UCB0TXBUF = (uint8_t)(regAddress & 0x00FF); // Send register L byte during SLA/W
        else
            UCB0TXBUF = writeBuffer[i-1];         // Send data byte
    }
    cnt = 0;
    while (!(UCB0IFG & UCTXIFG))            // Wait for UCB0TXBUF to be ready
    {
    	if(cnt<TIMEOUT_CNT)
    		cnt++;
    	else
    		return STATUS_FAIL;

        if (UCB0IFG & UCNACKIFG)            // Check if NAK occurred
        {
            UCB0CTL1 |= UCTXSTP;            // I2C STOP condition
            UCB0IFG &= ~UCTXIFG;            // Clear UCTXIFG (no byte was sent)
            return STATUS_FAIL;             // Exit with failure
        }
    }
    UCB0CTL1 |= UCTXSTP;                    // I2C STOP condition
    UCB0IFG &= ~UCTXIFG;                    // Clear UCTXIFG (no byte was sent)
    while(UCB0CTL1 & UCTXSTP);              // Wait for I2C STOP condition

    return (UCB0IFG & UCNACKIFG);
}

// -----------------------------------------------------------------------------
// System_i2c_UCB0_read
// -----------------------------------------------------------------------------
// Description:
//      Low level generic I2C read for 8-bit register addresses. Refer to I2C
//      USCI section of MSP430 User's Guide for flag timing. The slaveAddress
//      input is the 7-bit I2C slave address (right justified). The regAddress
//      input is the first data byte sent through the bus.
//      This routine performs a Repeated Start (Sr) between write and read.
//      This function returns 1 if NAK occurred.
// -----------------------------------------------------------------------------
int16_t System_i2c_read_16bSubAddr(uint8_t slaveAddress, uint16_t regAddress,
                             uint8_t *readBuffer, int16_t readBufferSize)
{

    int16_t i;
    uint16_t cnt;
    UCB0IFG &= ~UCNACKIFG;                  // Clear any previous NACK
    UCB0IFG &= ~UCTXIFG;                    // Clear any previous UCTXIFG
    UCB0IFG &= ~UCRXIFG;                    // Clear any previous UCRXIFG
    UCB0I2CSA = slaveAddress;               // Program I2C Slave Address

    // Write Phase
    UCB0CTL1 |= UCTR + UCTXSTT;             // Send TX START and Slave Address
    cnt = 0;
    while (!(UCB0IFG & UCTXIFG))            // Wait for UCTXIFG generated by UCTXSTT
    {
    	if(cnt<TIMEOUT_CNT)
    		cnt++;
    	else
    		return STATUS_FAIL;

        if (UCB0IFG & UCNACKIFG)            // Check if NAK occurred
        {
            UCB0CTL1 |= UCTXSTP;            // I2C STOP condition
            UCB0IFG &= ~UCTXIFG;            // Clear UCTXIFG (no byte was sent)
            return STATUS_FAIL;             // Exit with failure
        }
    }

    UCB0TXBUF = (uint8_t)(regAddress >> 8); // Send register H byte during SLA/W
    //UCB0TXBUF = regAddress;                  // Send register H byte during SLA/W
    cnt = 0;
    while (UCB0CTL1 & UCTXSTT)              // Wait for UCTXSTT to clear
    {
    	if(cnt<TIMEOUT_CNT)
    		cnt++;
    	else
    		return STATUS_FAIL;

        if (UCB0IFG & UCNACKIFG)            // Check if NAK occurred
        {
            UCB0CTL1 |= UCTXSTP;            // I2C STOP condition
            UCB0IFG &= ~UCTXIFG;            // Clear UCTXIFG (no byte was sent)
            return STATUS_FAIL;             // Exit with failure
        }
    }


    cnt = 0;
    while (!(UCB0IFG & UCTXIFG))            // Wait for UCB0TXBUF to be ready
    {
    	if(cnt<TIMEOUT_CNT)
    		cnt++;
    	else
    		return STATUS_FAIL;

        if (UCB0IFG & UCNACKIFG)            // Check if NAK occurred
        {
            UCB0CTL1 |= UCTXSTP;            // I2C STOP condition
            UCB0IFG &= ~UCTXIFG;            // Clear UCTXIFG (no byte was sent)
            return STATUS_FAIL;             // Exit with failure
        }
    }
    UCB0TXBUF = (uint8_t)(regAddress & 0x00FF); // Send register L byte during SLA/W

    while (!(UCB0IFG & UCTXIFG))            // Wait for UCB0TXBUF to be ready
    {
    	if(cnt<TIMEOUT_CNT)
    		cnt++;
    	else
    		return STATUS_FAIL;

        if (UCB0IFG & UCNACKIFG)            // Check if NAK occurred
        {
            UCB0CTL1 |= UCTXSTP;            // I2C STOP condition
            UCB0IFG &= ~UCTXIFG;            // Clear UCTXIFG (no byte was sent)
            return STATUS_FAIL;             // Exit with failure
        }
    }

    //uint8_t temp = (uint8_t)(regAddress & 0x00FF);
    //System_i2c_write(slaveAddress, (uint8_t)(regAddress >> 8), &temp, 1);

    // Read Phase
    UCB0CTL1 &= ~UCTR;                      // Reconfigure for RX Mode
    UCB0CTL1 |= UCTXSTT;                    // Initiate repeated START (Sr)
    UCB0IFG &= ~UCTXIFG;                    // Clear UCTXIFG (no byte was sent)
    cnt = 0;
    while(UCB0CTL1 & UCTXSTT)               // Wait for UCTXSTT to clear
    {
    	if(cnt<TIMEOUT_CNT)
    		cnt++;
    	else
    		return STATUS_FAIL;

        if (UCB0IFG & UCNACKIFG)            // Check if NAK occurred
        {
            UCB0CTL1 |= UCTXSTP;            // I2C STOP condition
            UCB0IFG &= ~UCTXIFG;            // Clear UCTXIFG (no byte was sent)
            return STATUS_FAIL;             // Exit with failure
        }
    }
    for(i=0; i<(readBufferSize-1); i++)
    {
    	cnt = 0;
        while (!(UCB0IFG & UCRXIFG))        // Wait for UCB0RXBUF byte to arrive
        {
        	if(cnt<TIMEOUT_CNT)
        		cnt++;
        	else
        		return STATUS_FAIL;

            if (UCB0IFG & UCNACKIFG)        // Check if NAK occurred
            {
                UCB0CTL1 |= UCTXSTP;        // I2C STOP condition
                UCB0IFG &= ~UCTXIFG;        // Clear UCTXIFG (no byte was sent)
                return STATUS_FAIL;         // Exit with failure
            }
        }
        readBuffer[i] = UCB0RXBUF;          // Read data byte
    }
    UCB0CTL1 |= UCTXSTP;                    // I2C STOP condition
    cnt = 0;
    while (!(UCB0IFG & UCRXIFG))            // Wait for UCB0RXBUF byte to arrive
    {
    	if(cnt<TIMEOUT_CNT)
    		cnt++;
    	else
    		return STATUS_FAIL;

        if (UCB0IFG & UCNACKIFG)            // Check if NAK occurred
        {
            UCB0CTL1 |= UCTXSTP;            // I2C STOP condition
            UCB0IFG &= ~UCTXIFG;            // Clear UCTXIFG (no byte was sent)
            return STATUS_FAIL;             // Exit with failure
        }
    }
    readBuffer[readBufferSize-1]=UCB0RXBUF; // Read last data byte
    while(UCB0CTL1 & UCTXSTP);              // Wait for I2C STOP condition

    return ((UCB0IFG & UCNACKIFG));
}

int16_t System_i2c_UCB0_write_16bSubAddr(uint8_t slaveAddress, uint16_t regAddress,
                              const uint8_t *writeBuffer,
                              int16_t writeBufferSize)
{
	int16_t state;

	state = System_i2c_write_16bSubAddr(slaveAddress, regAddress, writeBuffer, writeBufferSize);

	if(state == STATUS_FAIL) {
		System_i2c_UCB0_init();
		state = System_i2c_write_16bSubAddr(slaveAddress, regAddress, writeBuffer, writeBufferSize);
	}

	return state;
}

int16_t System_i2c_UCB0_read_16bSubAddr(uint8_t slaveAddress, uint16_t regAddress,
                             uint8_t *readBuffer, int16_t readBufferSize)
{
	int16_t state;

	state = System_i2c_read_16bSubAddr(slaveAddress, regAddress, readBuffer, readBufferSize);

	if(state == STATUS_FAIL) {
		System_i2c_UCB0_init();
		state = System_i2c_read_16bSubAddr(slaveAddress, regAddress, readBuffer, readBufferSize);
	}

	return state;
}

void I2C_Bus_Busy(void)
{
	while(UCB0STAT & UCBBUSY);
}
