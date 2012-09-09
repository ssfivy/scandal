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

}

void sc_write_conf(scandal_config*	conf) {

}

void sc_user_eeprom_read_block(u32 loc, u08* data, u08 length) {

}

void sc_user_eeprom_write_block(u32 loc, u08* data, u08 length) {

}

/* *******************
 * End Scandal wrappers
 */
