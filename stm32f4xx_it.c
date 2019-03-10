/**
  ******************************************************************************
  * @author  Eduard S.
  * @version V1.0.0
  * @date    23-December-2014
  * @brief   Interrupt handlers.
  *          The interrupt handler for the rising flag trigger is defined here.
  *          It handles the read and write operations of the gameboy to the
  *          cartridge.
  ******************************************************************************
  */

#include "stm32f4xx_it.h"
#include "repeat.h"

//#include "roms/tetris_rom.h"
//#include "roms/drmario_rom.h"
//#include "roms/jml_rom.h"
//#include "roms/zelda_rom.h"
//#include "roms/fubu_rom.h"
//#include "roms/dmgp_rom.h"
//#include "roms/zelda_f_rom.h"
//#include "roms/20y_rom.h"
//#include "roms/gejmboj_rom.h"
//#include "roms/oh_rom.h"
//#include "roms/mcmrder_rom.h"
//#include "roms/cpu_test_rom.h"
//#include "roms/gemgem_rom.h"
//#include "roms/organizer_rom.h"
//#include "roms/organizer_sav.h"
//#include "roms/demonblood_rom.h"
// #include "roms/joul_rom.h"
// #include "nanoloop_demo.h"
// #include "keytest.h"
// #include "lsdj_682_full.h"
// #include "mychara2.h"
// #include "dangan.h"
// #include "pixelboy.h"
#include "dhole2_logo.h"

#define GB_ROM_ADDR 0x08100000

/*
 * Macros to relate the GPIO and the functionality
 */
#define BUS_RD (GPIOC->IDR & 0x0002)
#define BUS_WR (GPIOC->IDR & 0x0004)
#define ADDR_IN GPIOD->IDR
#define DATA_IN GPIOE->IDR
#define DATA_OUT GPIOE->ODR
#define SET_DATA_MODE_IN GPIOE->MODER = 0x00000000;
#define SET_DATA_MODE_OUT GPIOE->MODER = 0x55550000;


/* Defines wether to show the Nintendo logo or the custom logo */
uint8_t no_show_logo;

uint16_t rom_bank = 0x01;
uint8_t ram_bank;
uint8_t ram_enable;
uint8_t rom_ram_mode;

uint8_t ram[0x20000] = {0xFF}; // 32K
uint8_t ram_uart_ptr = 0x00;

/* Write cartridge operation for MBC1 */
inline void mbc1_write(uint16_t addr, uint8_t data) {
	if (addr < 0x2000) {
		if (data) {
			ram_enable = 1;
		} else {
			ram_enable = 0;
		}
	} else if (addr >= 0x2000 && addr < 0x3000) {
		/* ROM Bank Number */
		rom_bank = data;
		if (data == 0x00) {
			// rom_bank |= 0x01;
		}
	} else if (addr >= 0x3000 && addr < 0x4000) {
		if(data){
			rom_bank = 0x0100 + (rom_bank & 0x00FF);
		} else {
			rom_bank = 0x0000 + (rom_bank & 0x00FF);
		}
	} else if (addr < 0x6000) {
		/*RAM Bank Number - or - Upper Bits of ROM Bank Number */
		if (rom_ram_mode) {
			/* ROM mode */
			data &= 0x07;
			// rom_bank = (rom_bank & 0x1F) | (data << 5);
		} else {
			/* RAM mode */
			ram_bank = data & 0x03;
		}
	} else if (addr < 0x8000) {
		/* ROM/RAM Mode Select */
		if (data) {
			/* Emable RAM Banking mode */
			rom_ram_mode = 1;
		} else {
			/* Emable ROM Banking mode */
			rom_ram_mode = 0;
		}
	}
	else if (addr >= 0xA000 && addr < 0xC000) {
 		/* 8KB RAM Bank 00-03, if any */
		ram[addr - 0xA000 + 0x2000 * ram_bank] = data;
		if(addr == 0xA000){
			USART_SendData(USART2, ram[0]);
		}
	}

}

/* Read cartridge operation for MBC1 */
inline uint8_t mbc1_read(uint16_t addr) {
	if (addr < 0x4000) {
		/* 16KB ROM bank 00 */
		if (no_show_logo) {
			/* Custom logo disabled */
			// return rom_gb[addr];
			return *((uint8_t *)(GB_ROM_ADDR+addr));
		} else {
			/* Custom logo enabled, only during first read at boot */
			if (addr == 0x133) {
				no_show_logo = 1;
			}
			return logo_bin[addr - 0x104];
		}
	} else if (addr < 0x8000) {
		/* 16KB ROM Bank 01-7F */
		// return rom_gb[addr + 0x4000 * (rom_bank - 1)];
		unsigned int bankaddr = addr + 0x4000 * (rom_bank-1);
		return *((uint8_t *)(GB_ROM_ADDR+bankaddr));
	} else if (addr >= 0xA000 && addr < 0xC000) {
		/* 8KB RAM Bank 00-03, if any */
		return ram[addr - 0xA000 + 0x2000 * ram_bank];
	}
	return 0x00;
}

/* Handle PC0 interrupt (rising edge of the gameboy CLK) */
void EXTI0_IRQHandler(void) {
	uint16_t addr;
	uint8_t data;

	uint32_t enablestatus;
	enablestatus =  EXTI->IMR & EXTI_Line0;

	if (((EXTI->PR & EXTI_Line0) != (uint32_t)RESET) &&
	    (enablestatus != (uint32_t)RESET)) {
		/* Do stuff on trigger */

		/* Wait 10 NOPs, until the ADDR is ready in the bus */
		REP(1,0,asm("NOP"););
		/* Read ADDR from the bus */
		addr = ADDR_IN;

		if (BUS_RD || !BUS_WR) {
			/* Write operation */

			/* Wait 30 NOPs, until the DATA is ready in the bus */
			REP(3,0,asm("NOP"););
			/* Read DATA from the bus */
			data = DATA_IN >> 8;
			/* Write data to cartridge at addr */
			mbc1_write(addr, data);
		} else {
			/* Read operation */

			/* Set the GPIOE in output mode */
			SET_DATA_MODE_OUT;
			/* Output the data read at addr through GPIOE */
			DATA_OUT = ((uint16_t)mbc1_read(addr)) << 8;
			/* Wait 14 NOPs, until the gameboy has read the DATA
			 * in the bus */
			REP(1,4,asm("NOP"););
			/* Set the GPIOE back to input mode */
			SET_DATA_MODE_IN;
		}
	}
	/* Clear interrupt flag */
	EXTI->PR = EXTI_Line0;
	//EXTI_ClearITPendingBit(EXTI_Line0);
}

void EXTI3_IRQHandler(void) {
	uint16_t addr;
	uint8_t data;

	uint32_t enablestatus;
	enablestatus =  EXTI->IMR & EXTI_Line3;

	if (((EXTI->PR & EXTI_Line3) != (uint32_t)RESET) &&
	    (enablestatus != (uint32_t)RESET)) {
		ram[0] = 0xFF;
	}
	EXTI->PR = EXTI_Line3;
}

void USART2_IRQHandler(void) {
	uint16_t i=2;
	if(USART_GetITStatus(USART2, USART_IT_RXNE) == SET) {
		i=USART_ReceiveData(USART2); //受信したデータ読み込み
		USART_SendData(USART2, i); //そのデータを送信
	}
	ram[ram_uart_ptr+0x0010] = (uint8_t)i;
	ram_uart_ptr++;
}
