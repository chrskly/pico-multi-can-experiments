#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "can.h"

#define SPI_PORT  spi0

#define SPI_MISO  6
#define SPI_CLK   9
#define SPI_MOSI  10

// CAN board 0
#define SPI_0_CS    7

// CAN board 1
#define SPI_1_CS    11

//#define CAN_INT   20 // Interrupt from CAN controller

void SPI_configure() {
    spi_init(SPI_PORT, 1000000);
    spi_set_format(SPI_PORT, 8, 0, 0, SPI_MSB_FIRST);
    gpio_set_function(SPI_MISO, GPIO_FUNC_SPI);
    gpio_set_function(SPI_MOSI, GPIO_FUNC_SPI);
    gpio_set_function(SPI_CLK, GPIO_FUNC_SPI);
    // board 0
    gpio_init(SPI_0_CS);
    gpio_set_dir(SPI_0_CS, GPIO_OUT);
    gpio_put(SPI_0_CS, 1); // set inactive
    // board 1
    gpio_init(SPI_1_CS);
    gpio_set_dir(SPI_1_CS, GPIO_OUT);
    gpio_put(SPI_1_CS, 1); // set inactive
}

void CAN_reset() {
	// board 0
	gpio_put(SPI_0_CS, 0);
	spi_write_blocking(SPI_PORT, (uint8_t[]){CMD_RESET}, 1);
	gpio_put(SPI_0_CS, 1);
	busy_wait_us(100);
	// board 1
	gpio_put(SPI_1_CS, 0);
	spi_write_blocking(SPI_PORT, (uint8_t[]){CMD_RESET}, 1);
	gpio_put(SPI_1_CS, 1);
	busy_wait_us(100);
}

uint8_t CAN_0_reg_read(uint8_t reg) {
	uint8_t data;
	gpio_put(SPI_0_CS, 0);
	spi_write_blocking(SPI_PORT, (uint8_t[]){CMD_READ, reg}, 2);
	spi_read_blocking(SPI_PORT, 0, &data, 1);
	gpio_put(SPI_0_CS, 1);
	return(data);
}

uint8_t CAN_1_reg_read(uint8_t reg) {
	uint8_t data;
	gpio_put(SPI_1_CS, 0);
	spi_write_blocking(SPI_PORT, (uint8_t[]){CMD_READ, reg}, 2);
	spi_read_blocking(SPI_PORT, 0, &data, 1);
	gpio_put(SPI_1_CS, 1);
	return(data);
}

void CAN_0_reg_write(uint8_t reg, uint8_t val) {
  gpio_put(SPI_0_CS, 0);
  spi_write_blocking(SPI_PORT, (uint8_t[]){CMD_WRITE, reg, val}, 3);
  gpio_put(SPI_0_CS, 1);
}

void CAN_1_reg_write(uint8_t reg, uint8_t val) {
  gpio_put(SPI_1_CS, 0);
  spi_write_blocking(SPI_PORT, (uint8_t[]){CMD_WRITE, reg, val}, 3);
  gpio_put(SPI_1_CS, 1);
}

void CAN_0_reg_modify(uint8_t reg, uint8_t mask, uint8_t val) {
  gpio_put(SPI_0_CS, 0);
  busy_wait_us(2);
  spi_write_blocking(SPI_PORT, (uint8_t[]){CMD_MODIFY, reg, mask, val}, 4);
  busy_wait_us(2);
  gpio_put(SPI_0_CS, 1);
}

void CAN_1_reg_modify(uint8_t reg, uint8_t mask, uint8_t val) {
  gpio_put(SPI_1_CS, 0);
  busy_wait_us(2);
  spi_write_blocking(SPI_PORT, (uint8_t[]){CMD_MODIFY, reg, mask, val}, 4);
  busy_wait_us(2);
  gpio_put(SPI_1_CS, 1);
}

void CAN_configure(uint16_t id) {
  // Configure speed to 500kbps based on 8MHz Crystal
  // Magic constants from https://github.com/sandeepmistry/arduino-CAN/blob/master/src/MCP2515.cpp
  CAN_0_reg_write(REG_CNF1, 0x00);
  CAN_0_reg_write(REG_CNF2, 0x90);
  CAN_0_reg_write(REG_CNF3, 0x02);

  // Enable receive interrupts
  //CAN_reg_write(REG_CANINTE, 3);

  // Set normal operation mode
  CAN_0_reg_write(REG_CANCTRL, MODE_NORMAL);

  // Configure speed to 500kbps based on 8MHz Crystal
  // Magic constants from https://github.com/sandeepmistry/arduino-CAN/blob/master/src/MCP2515.cpp
  CAN_1_reg_write(REG_CNF1, 0x00);
  CAN_1_reg_write(REG_CNF2, 0x90);
  CAN_1_reg_write(REG_CNF3, 0x02);

  // Enable receive interrupts
  //CAN_reg_write(REG_CANINTE, 3);

  // Set normal operation mode
  CAN_1_reg_write(REG_CANCTRL, MODE_NORMAL);
}

void CAN_0_transmit(uint16_t id, uint8_t* data, uint8_t length) {
  CAN_0_reg_write(REG_TXBnSIDH(0), id >> 3); // Set CAN ID
  CAN_0_reg_write(REG_TXBnSIDL(0), id << 5); // Set CAN ID
  CAN_0_reg_write(REG_TXBnEID8(0), 0x00);    // Extended ID
  CAN_0_reg_write(REG_TXBnEID0(0), 0x00);    // Extended ID

  CAN_0_reg_write(REG_TXBnDLC(0), length);   // Frame length

  for (int i = 0; i < length; i++) {       // Write the frame data
    CAN_0_reg_write(REG_TXBnD0(0) + i, data[i]);
  }

  CAN_0_reg_write(REG_TXBnCTRL(0), 0x08);    // Start sending
  busy_wait_us(1000); // Allow up to 1ms to transmit
  CAN_0_reg_write(REG_TXBnCTRL(0), 0);    // Stop sending
  CAN_0_reg_modify(REG_CANINTF, FLAG_TXnIF(0), 0x00); // Clear interrupt flag
}


void CAN_1_transmit(uint16_t id, uint8_t* data, uint8_t length) {
  CAN_1_reg_write(REG_TXBnSIDH(0), id >> 3); // Set CAN ID
  CAN_1_reg_write(REG_TXBnSIDL(0), id << 5); // Set CAN ID
  CAN_1_reg_write(REG_TXBnEID8(0), 0x00);    // Extended ID
  CAN_1_reg_write(REG_TXBnEID0(0), 0x00);    // Extended ID

  CAN_1_reg_write(REG_TXBnDLC(0), length);   // Frame length

  for (int i = 0; i < length; i++) {       // Write the frame data
    CAN_1_reg_write(REG_TXBnD0(0) + i, data[i]);
  }

  CAN_1_reg_write(REG_TXBnCTRL(0), 0x08);    // Start sending
  busy_wait_us(1000); // Allow up to 1ms to transmit
  CAN_1_reg_write(REG_TXBnCTRL(0), 0);    // Stop sending
  CAN_1_reg_modify(REG_CANINTF, FLAG_TXnIF(0), 0x00); // Clear interrupt flag
}

