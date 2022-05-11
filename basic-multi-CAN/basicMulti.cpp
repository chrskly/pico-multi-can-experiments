/*------------------------------------------------------------------------------

Connect two mcp2515 boards to a Pi Pico. Have them print out the id of any msg
received to the serial uart.

Pins
GPIO0  (1)  uart rx  - to ftdi tx
GPIO1  (2)  uart tx  - to ftdi rx
GPIO16 (21) SPI0 RX  - to MISO on both mcp2515 boards (via level converter)
GPIO18 (24) SPI0 SCK - to SCK on both mcp2515 boards (via level converter)
GPIO19 (25) SPI0 TX  - to MOSI on both mcp2515 boards (via level converter)
GPIO17 (22) SPI0 CS0 - to CS on mcp2515 board 0 (via level converter)
SPIO15 (20) SPI0 CS1 - to CS on mcp2515 board 1 (vai level converter)

------------------------------------------------------------------------------*/

#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "hardware/uart.h"
#include "hardware/gpio.h"
#include "mcp2515/mcp2515.h"

#define SPI_PORT  spi0

#define SPI_MISO   16 // RX
#define SPI_CLK    18 // 
#define SPI_MOSI   19 // TX

// CAN board 0
#define CAN_0_CS    17
#define CAN_0_INT   9


// CAN board 1
#define CAN_1_CS   15
#define CAN_1_INT  14

// Serial port
#define UART_ID uart0
#define BAUD_RATE 115200
#define UART_TX_PIN 0
#define UART_RX_PIN 1

const uint LED_PIN = PICO_DEFAULT_LED_PIN;

uint8_t can_data_buffer[16];

//MCP2515 can0;
MCP2515 can0(spi0, CAN_0_CS, SPI_MISO, SPI_MOSI, SPI_CLK, 10000000);
MCP2515 can1(spi0, CAN_1_CS, SPI_MISO, SPI_MOSI, SPI_CLK, 10000000);

struct can_frame rx;

int main() {
    stdio_init_all();

    // set up the serial port
    uart_init(UART_ID, BAUD_RATE);
    gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);

    uart_puts(UART_ID, "Starting up...\n");

    //Initialize interface
    can0.reset();
    can0.setBitrate(CAN_1000KBPS, MCP_8MHZ);
    can0.setNormalMode();

    can1.reset();
    can1.setBitrate(CAN_1000KBPS, MCP_8MHZ);
    can1.setNormalMode();

    //Listen loop
    while(true) {
        if(can1.readMessage(&rx) == MCP2515::ERROR_OK) {
            char str[200];
            sprintf(str, "CAN1 New frame from ID: %10x\n", rx.can_id);
            uart_puts(UART_ID, str);
        }
        if(can0.readMessage(&rx) == MCP2515::ERROR_OK) {
            char str[200];
            sprintf(str, "CAN0 New frame from ID: %10x\n", rx.can_id);
            uart_puts(UART_ID, str);
        }
    }

    return 0;
}


