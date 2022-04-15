#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "hardware/uart.h"
#include "can.h"

#define SPI_PORT  spi0

#define SPI_MISO    6
#define SPI_CLK     9
#define SPI_MOSI   10

// CAN board 0
#define CAN_0_CS    7
#define CAN_0_INT   5

// CAN board 1
#define CAN_1_CS   11
#define CAN_1_INT  13

// Serial port
#define UART_ID uart0
#define BAUD_RATE 115200
#define UART_TX_PIN 0
#define UART_RX_PIN 1

const uint LED_PIN = PICO_DEFAULT_LED_PIN;


uint8_t can_data_buffer[16];


void SPI_configure() {
    spi_init(SPI_PORT, 1000000);
    spi_set_format(SPI_PORT, 8, 0, 0, SPI_MSB_FIRST);
    gpio_set_function(SPI_MISO, GPIO_FUNC_SPI);
    gpio_set_function(SPI_MOSI, GPIO_FUNC_SPI);
    gpio_set_function(SPI_CLK, GPIO_FUNC_SPI);
    // board 0
    gpio_init(CAN_0_CS);
    gpio_set_dir(CAN_0_CS, GPIO_OUT);
    gpio_put(CAN_0_CS, 1); // set inactive
    // board 1
    gpio_init(CAN_1_CS);
    gpio_set_dir(CAN_1_CS, GPIO_OUT);
    gpio_put(CAN_1_CS, 1); // set inactive
}

void CAN_reset() {
    // board 0
    gpio_put(CAN_0_CS, 0);
    spi_write_blocking(SPI_PORT, (uint8_t[]){CMD_RESET}, 1);
    gpio_put(CAN_0_CS, 1);
    busy_wait_us(100);
    // board 1
    gpio_put(CAN_1_CS, 0);
    spi_write_blocking(SPI_PORT, (uint8_t[]){CMD_RESET}, 1);
    gpio_put(CAN_1_CS, 1);
    busy_wait_us(100);
}

uint8_t CAN_0_reg_read(uint8_t reg) {
    uint8_t data;
    gpio_put(CAN_0_CS, 0);
    spi_write_blocking(SPI_PORT, (uint8_t[]){CMD_READ, reg}, 2);
    spi_read_blocking(SPI_PORT, 0, &data, 1);
    gpio_put(CAN_0_CS, 1);
    return(data);
}

uint8_t CAN_1_reg_read(uint8_t reg) {
    uint8_t data;
    gpio_put(CAN_1_CS, 0);
    spi_write_blocking(SPI_PORT, (uint8_t[]){CMD_READ, reg}, 2);
    spi_read_blocking(SPI_PORT, 0, &data, 1);
    gpio_put(CAN_1_CS, 1);
    return(data);
}

void CAN_0_reg_write(uint8_t reg, uint8_t val) {
    gpio_put(CAN_0_CS, 0);
    spi_write_blocking(SPI_PORT, (uint8_t[]){CMD_WRITE, reg, val}, 3);
    gpio_put(CAN_0_CS, 1);
}

void CAN_1_reg_write(uint8_t reg, uint8_t val) {
    gpio_put(CAN_1_CS, 0);
    spi_write_blocking(SPI_PORT, (uint8_t[]){CMD_WRITE, reg, val}, 3);
    gpio_put(CAN_1_CS, 1);
}

void CAN_0_reg_modify(uint8_t reg, uint8_t mask, uint8_t val) {
    gpio_put(CAN_0_CS, 0);
    busy_wait_us(2);
    spi_write_blocking(SPI_PORT, (uint8_t[]){CMD_MODIFY, reg, mask, val}, 4);
    busy_wait_us(2);
    gpio_put(CAN_0_CS, 1);
}

void CAN_1_reg_modify(uint8_t reg, uint8_t mask, uint8_t val) {
    gpio_put(CAN_1_CS, 0);
    busy_wait_us(2);
    spi_write_blocking(SPI_PORT, (uint8_t[]){CMD_MODIFY, reg, mask, val}, 4);
    busy_wait_us(2);
    gpio_put(CAN_1_CS, 1);
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

uint8_t CAN_0_receive(uint8_t * can_rx_data) {
  uint8_t intf = CAN_0_reg_read(REG_CANINTF);
  uint8_t rtr;
  uint8_t n; // One of two receive buffers
  if(intf & FLAG_RXnIF(0)) {
    n = 0;
  } else if (intf & FLAG_RXnIF(1)) {
    n = 1;
  } else {
    return 0;
  }
  rtr = (CAN_0_reg_read(REG_RXBnSIDL(n)) & FLAG_SRR) ? true : false;
  uint8_t dlc = CAN_0_reg_read(REG_RXBnDLC(n)) & 0x0f;

  uint8_t length;
  if (rtr) {
    length = 0;
  } else {
    length = dlc;

    for (int i = 0; i < length; i++)
      can_rx_data[i] = CAN_0_reg_read(REG_RXBnD0(n) + i);
  }

  CAN_0_reg_modify(REG_CANINTF, FLAG_RXnIF(n), 0x00);
  return(length);
}

uint8_t CAN_1_receive(uint8_t * can_rx_data) {
  uint8_t intf = CAN_1_reg_read(REG_CANINTF);
  uint8_t rtr;
  uint8_t n; // One of two receive buffers
  if(intf & FLAG_RXnIF(0)) {
    n = 0;
  } else if (intf & FLAG_RXnIF(1)) {
    n = 1;
  } else {
    return 0;
  }
  rtr = (CAN_1_reg_read(REG_RXBnSIDL(n)) & FLAG_SRR) ? true : false;
  uint8_t dlc = CAN_1_reg_read(REG_RXBnDLC(n)) & 0x0f;

  uint8_t length;
  if (rtr) {
    length = 0;
  } else {
    length = dlc;

    for (int i = 0; i < length; i++)
      can_rx_data[i] = CAN_1_reg_read(REG_RXBnD0(n) + i);
  }

  CAN_1_reg_modify(REG_CANINTF, FLAG_RXnIF(n), 0x00);
  return(length);
}

void CAN_0_receive_callback(uint gpio, uint32_t events) {
  gpio_put(LED_PIN, 1);
  uint8_t data = CAN_0_receive(can_data_buffer);
  gpio_acknowledge_irq(CAN_0_INT, GPIO_IRQ_LEVEL_LOW);
  printf("CAN0 received : 0x%.8X", data);
  gpio_put(LED_PIN, 0);
}

void CAN_1_receive_callback(uint gpio, uint32_t events) {
  gpio_put(LED_PIN, 1);
  uint8_t data = CAN_1_receive(can_data_buffer);
  gpio_acknowledge_irq(CAN_1_INT, GPIO_IRQ_LEVEL_LOW);
  printf("CAN1 received : 0x%.8X", data);
  gpio_put(LED_PIN, 0);
}

void CAN_configure(uint16_t id) {
    // Configure speed to 500kbps based on 8MHz Crystal
    // Magic constants from https://github.com/sandeepmistry/arduino-CAN/blob/master/src/MCP2515.cpp
    CAN_0_reg_write(REG_CNF1, 0x00);
    CAN_0_reg_write(REG_CNF2, 0x90);
    CAN_0_reg_write(REG_CNF3, 0x02);

    // set up intterupt handler
    gpio_init(CAN_0_INT);
    gpio_set_dir(CAN_0_INT,GPIO_IN);
    gpio_disable_pulls(CAN_0_INT);
    gpio_set_irq_enabled_with_callback(CAN_0_INT, GPIO_IRQ_LEVEL_LOW, true, &CAN_0_receive_callback);

    // Enable receive interrupts
    //CAN_reg_write(REG_CANINTE, 3);

    // Set normal operation mode
    CAN_0_reg_write(REG_CANCTRL, MODE_NORMAL);

    // Configure speed to 500kbps based on 8MHz Crystal
    // Magic constants from https://github.com/sandeepmistry/arduino-CAN/blob/master/src/MCP2515.cpp
    CAN_1_reg_write(REG_CNF1, 0x00);
    CAN_1_reg_write(REG_CNF2, 0x90);
    CAN_1_reg_write(REG_CNF3, 0x02);

    // set up interrupt handler
    gpio_init(CAN_1_INT);
    gpio_set_dir(CAN_1_INT,GPIO_IN);
    gpio_disable_pulls(CAN_1_INT);
    gpio_set_irq_enabled_with_callback(CAN_1_INT, GPIO_IRQ_LEVEL_LOW, true, &CAN_1_receive_callback);

    // Enable receive interrupts
    //CAN_reg_write(REG_CANINTE, 3);

    // Set normal operation mode
    CAN_1_reg_write(REG_CANCTRL, MODE_NORMAL);
}





int main() {
    // set up the serial port
    uart_init(UART_ID, BAUD_RATE);
    gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);

    // LED blinks when msg received
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);

    while(1) {
        // Send DEADBEEF from CAN 0
        CAN_0_transmit(0x01, (uint8_t[]){ 0xDE, 0xAD, 0xBE, 0xEF }, 4);
        // sleep for 1 second
        busy_wait_ms(1000);
    }
}
