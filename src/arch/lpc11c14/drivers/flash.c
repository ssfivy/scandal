/* Scandal wrappers
 * *****************/

#include <scandal/eeprom.h>
#include <scandal/utils.h>

#include <arch/flash.h>

#include <string.h>

/* On the LPC11C14, we have 32KiB of flash memory. The actual program is stored in this
 * flash too. The linker script is set up to only allow the program to use the first
 * 7 'blocks' of 4KiB, with the last block of 4KiB being used for 'user' storage area.
 * The first few bytes of this area are used to store the scandal_config struct, so when
 * the user read and write functions are actually implemented, the start address should
 * be *user_flash_start + sizeof(scandal_config).
 */

void sc_init_eeprom(void) {
	/* no init required! */
}

void sc_read_conf(scandal_config *conf) {
	IAP iap_entry;
	iap_entry = (IAP)IAP_LOCATION;

	/* since we are storing the config at the start of 'user' flash, we can just cast the address to
	 * that type and do a memcpy */
	scandal_config *config_start = ((scandal_config *)(0x00007000));

	memcpy(conf, config_start, sizeof(scandal_config));
}

void sc_write_conf(scandal_config*	conf) {
	IAP iap_entry;
	iap_entry = (IAP)IAP_LOCATION;

	unsigned int iapCommand[5] = {0};
	unsigned int iapResult[4] = {0};
	uint32_t write_buffer[64] = {0}; /* 256 byte block min write size */

	/* prepare for erase */
	iapCommand[0] = 50;					// command to prepare
	iapCommand[1] = 7;					// from sector 7
	iapCommand[2] = 7;					// to sector 7
	iap_entry (iapCommand, iapResult); 	// send the commands
	scandal_delay(1);

	/* do the erase */
	iapCommand[0] = 52;					// command to erase
	iapCommand[1] = 7;					// from sector 7
	iapCommand[2] = 7;					// to sector 7
	iapCommand[3] = 48000;				// clock rate in khz 
	iap_entry (iapCommand, iapResult); 	// send the commands
	scandal_delay(120);					// wait for it to be done

	/* prepare for write */
	iapCommand[0] = 50;					// command to prepare
	iapCommand[1] = 7;					// from sector 7
	iapCommand[2] = 7;					// to sector 7
	iap_entry (iapCommand, iapResult); 	// send the commands
	scandal_delay(1);

	/* copy the config data into the write buffer */
	memcpy(write_buffer, conf, sizeof(scandal_config));

	/* actually do the write */
	iapCommand[0] = 51;					// write to flash command
	iapCommand[1] = 0x00007000;			// write starting from this flash address
	iapCommand[2] = (uint32_t)(&write_buffer);		// read from this memory address
	iapCommand[3] = 256;				// 256 bytes is the minimum size we can write to at once
	iapCommand[4] = 48000;				// clock speed in khz - 48000
	iap_entry (iapCommand, iapResult);
	scandal_delay(100);
}

void sc_user_eeprom_read_block(u32 loc, u08* data, u08 length) {

}

void sc_user_eeprom_write_block(u32 loc, u08* data, u08 length) {

}

/* *******************
 * End Scandal wrappers
 */
