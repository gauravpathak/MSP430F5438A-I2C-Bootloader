#include "System_i2c.h"
#include "msp430.h"

#define SW_RESET()      PMMCTL0 = PMMPW + PMMSWBOR + (PMMCTL0 & 0x0003);  // software BOR reset

#define INTERRUPT_VECTOR_START 0xFFE0

#pragma DATA_SECTION(BSL430_Version, "BSL430_Version_SEG")

const unsigned char BSL430_Version [4] = { 0x01, 0x02, 0x03, 0x04 };

unsigned char flash[128];

void BSL430_massErase(void);	//Function for Performing Mass Erase of Flash Memory except INFO SEGMENTS and BSL Segments

void write_reset_vector(unsigned short);

/*****************************************************************************************************************************************
 * Function Name	: Main
 * Return Type		: None
 * Reentrant		: No
 * Description		:
 * Parameters		: None
 ******************************************************************************************************************************************/
void main (void)
{
	char *Flashptr;
	int i,length=0,regaddr=0x0100,j,vect_addr;;

	WDTCTL = WDTPW | WDTHOLD;				// Stop watchdog timer

	Device_clockSetup();					//Configures SMCLK for 8MHz
	Device_portSetup();						//Configures Port Mapping for SPI, I2C and UART pins

	BSL430_massErase();						//Performs Erase of Flash from 0x8000 and Unlocks the Boot Strap Loader Memory Section
	System_i2c_UCB0_init();					//Initialises on chip I2C for frequency 400KHz

	System_i2c_UCB0_isPresent(0x50);		//Checks whether I2C Slave is present and ready or not

	System_i2c_UCB0_read_16bSubAddr(0x50,0x0000,flash,128);		//Read Meta-data from EEPROM stored at Page 0 and offset zero
	length = (flash[3]);								// Read the length of Firmware stored at byte 2 and byte 3 in page 0 of
	length = length << 8;								// EEPROM and store it in a variable name Length
	length |= (flash[2]);

	vect_addr = flash[10];					//Read High Byte of reset-vector stored at byte 10
	vect_addr = vect_addr << 8;
	vect_addr |= (flash[9]);				//Read Low Byte of reset-vector stored at byte 9

	Flashptr = (char *)0x8000;					//Assign the address of Flash memory where we want to write new firmware
	FCTL3 = FWKEY;                            	// Clear Lock bit to Unlock Flash of MSP430
	FCTL1 = FWKEY+WRT;                        	// Set WRT bit for write operation
	while(FCTL3 & BUSY);						//Check whether Flash Controller is ready for operation, if not wait for it to get ready

	for(i=0; i<length; i=(i+128))				//Increment the loop by 128, as per EEPROM 24AA412 datasheet we can read 128bytes at a time
	{
	  System_i2c_UCB0_read_16bSubAddr(0x50,regaddr,flash,128);	//Read the firmware stored at Page 1 offset 0, 128 bytes at a time
	  for(j=0; j<128; j++)										//Starting at address 0x0100 of EEPROM
	  {
		  while(FCTL3 & BUSY);
		  *Flashptr = flash[j];									//Write the contents of EEPROM on Flash Memory
		  Flashptr++;											//Increment Flashptr to point at next memory address of MSP430
	  }
	  regaddr += 0x0080;							//Increment EEPROM register address by 128bytes according to EEPROM 24AA512 datasheet
	}
	FCTL1 = FWKEY;                            					// Clear WRT bit
	FCTL3 = FWKEY+LOCK;											//Lock Flash Memory after Firmware Upgrade

	flash[0] &= ~(0x04);								//Clear Byte Zero Bit1 of EEPROM i.e. Flag indicating New Firmware Update Availability
	System_i2c_UCB0_write_16bSubAddr(0x50,0x0000,flash,1);	//Write this bit on EEPROM

	write_reset_vector(vect_addr);//Write the reset vector location so that MSP430 get to know the starting address of new code updated in Flash
	SW_RESET();						//Perform Brown Out Reset to redirect PC to Reset Vector
}

/*****************************************************************************************************************************************
 * Function Name	: BSL430_massErase
 * Return Type		: None
 * Reentrant		: No
 * Parameters		: None
 * Description		: Performs Mass Erase of flash memory starting at address 0x8000; Does not Erase Information segments as well as Boot Strap Loader
 * 					segments and after a mass erase operation Boot Strap Loader is unlocked as Passwords stored at Interrupt Vector locations
 * 					are cleared.
 ******************************************************************************************************************************************/
void BSL430_massErase(void)
{
	char *Flash_ptr;                 				// Flash pointer
	while(FCTL3 & BUSY);
	FCTL3 = FWKEY;
	Flash_ptr = (char *)INTERRUPT_VECTOR_START;		// Initialise Flash pointer
	while(FCTL3 & BUSY);
	FCTL1 = FWKEY + MERAS + ERASE;        			// Set Mass Erase bit
	while(FCTL3 & BUSY);
	*Flash_ptr = 0;                           		// Dummy write to erase main flash
	while(FCTL3 & BUSY);
	FCTL3 = FWKEY + LOCK;                  			// Set LOCK bit
}

/*****************************************************************************************************************************************
 * Function Name	: write_reset_vector
 * Return Type		: None
 * Reentrant		: No
 * Parameters		: Starting Address of the boot code to be stored at Reset Vector location
 * Description		: Writes the address of boot-code at Reset Vector Location so as to redirect the PC to start execution after a Reset or BOR
  ******************************************************************************************************************************************/
void write_reset_vector(unsigned short address)
{
	char *Flashptr;
	Flashptr = (char *)0xFFFE;
	FCTL1 = FWKEY + ERASE;         			// Set Mass Erase bit
	while(FCTL3 & BUSY);
	*Flashptr = 0;                         	// Dummy write to erase main flash
	while(FCTL3 & BUSY);
	FCTL3 = FWKEY;                          // Clear Lock bit
	FCTL1 = FWKEY+WRT;                      // Set WRT bit for write operation
	while(FCTL3 & BUSY);
	*((unsigned short *)(Flashptr)) = address;
	FCTL1 = FWKEY;                            // Clear WRT bit
	FCTL3 = FWKEY+LOCK;
}
/*END OF FILE*/
