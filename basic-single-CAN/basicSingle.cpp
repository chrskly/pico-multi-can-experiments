#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "hardware/uart.h"
#include "hardware/gpio.h"
#include "mcp2515/mcp2515.h"

#define SPI_PORT  spi0

#define SPI_MISO    4
#define SPI_CLK     2
#define SPI_MOSI    5

// CAN board 0
#define CAN_0_CS    5
#define CAN_0_INT   9


// CAN board 1
#define CAN_1_CS   11
#define CAN_1_INT  14

// Serial port
#define UART_ID uart0
#define BAUD_RATE 115200
#define UART_TX_PIN 0
#define UART_RX_PIN 1

const uint LED_PIN = PICO_DEFAULT_LED_PIN;

uint8_t can_data_buffer[16];

MCP2515 can0;
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

    //Listen loop
    while(true) {
        if(can0.readMessage(&rx) == MCP2515::ERROR_OK) {
            char str[200];
            sprintf(str, "New frame from ID: %10x\n", rx.can_id);
            uart_puts(UART_ID, str);
        }
    }

    return 0;
}


