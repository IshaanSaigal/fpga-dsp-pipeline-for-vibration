/*
 * This program is for configuring the ESP32 as an SPI slave.
 * Depending on the byte received from the SPI master on the FPGA,
 * this program gives control signals to the L293D motor driver.
 */

// --- Including Necessary Header Files ---

// Standard C headers:
#include <stdio.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>

// Needed for FreeRTOS tasks and delays:
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// Needed for logging (ESP_LOGI, ESP_LOGE, ESP_LOGW):
#include "esp_log.h"

// Needed for GPIO configuration:
#include "driver/gpio.h"

// Needed for Legacy SPI Slave driver:
#include "driver/spi_slave.h" // This also includes driver/spi_common.h where VSPI_HOST is defined


// --- Pin Definitions ---

// SPI Pins (defaults for VSPI / SPI3)
#define SPI_CS_PIN      GPIO_NUM_5
#define SPI_SCLK_PIN    GPIO_NUM_18
#define SPI_MOSI_PIN    GPIO_NUM_23
#define SPI_MISO_PIN    GPIO_NUM_19

// Motor Control Pins
#define L293D_IN1_PIN   GPIO_NUM_25
#define L293D_IN2_PIN   GPIO_NUM_26


// --- SPI Definitions ---
// Using VSPI_HOST (maps to SPI3), which is defined in driver/spi_common.h (included by driver/spi_slave.h)
#define SLAVE_SPI_HOST VSPI_HOST

// Logging Tag (the first argument in the ESP_LOGx functions;
// determines that the log message corresponds to the SPI slave)
static const char *TAG = "SPI_SLAVE";

// Buffer to store received data
#define SPI_BUF_SIZE 64 // Configuring the receive buffer size as the maximum value of 64-bytes (for eventual FFT)
WORD_ALIGNED_ATTR uint8_t recv_buf[SPI_BUF_SIZE]; // WORD_ALIGNED_ATTR is good practice for DMA-capable buffers

// NOTE: To send data back, we would similarly define a send_buf:
// WORD_ALIGNED_ATTR uint8_t send_buf[SPI_BUF_SIZE];


// --- Function Declarations for Motor Control ---

void motor_setup();
void motor_run_forward();
void motor_stop();

void motor_setup() {
    gpio_reset_pin(L293D_IN1_PIN);  // Resets the pin to its default state
    gpio_set_direction(L293D_IN1_PIN, GPIO_MODE_OUTPUT);    // Configures the pins as a GPIO output
    gpio_reset_pin(L293D_IN2_PIN);
    gpio_set_direction(L293D_IN2_PIN, GPIO_MODE_OUTPUT);

    ESP_LOGI(TAG, "Motor GPIOs initialized.");
    motor_stop(); // Ensures the motor is initially stopped
}

void motor_run_forward() {
    ESP_LOGI(TAG, "Command: Motor Run Forward");
    gpio_set_level(L293D_IN1_PIN, 1);   // Set pin to '1' / HIGH
    gpio_set_level(L293D_IN2_PIN, 0);
}

void motor_stop() {
    ESP_LOGI(TAG, "Command: Motor Stop");
    gpio_set_level(L293D_IN1_PIN, 0);   // Set pin to '0' / LOW
    gpio_set_level(L293D_IN2_PIN, 0);
}


// --- Main Function ---

void app_main(void)
{
    esp_err_t ret; // Special integer type variable used by many ESP-IDF functions to return error codes

    // Initialize Motor GPIOs:
    motor_setup();

    ESP_LOGI(TAG, "Initializing legacy SPI slave...");

    // Configuration for the SPI bus:
    spi_bus_config_t bus_cfg = {
        .mosi_io_num = SPI_MOSI_PIN,
        .miso_io_num = SPI_MISO_PIN,
        .sclk_io_num = SPI_SCLK_PIN,
        .quadwp_io_num = -1, // Quad Write Protect; -1 means this is not used
        .quadhd_io_num = -1, // Quad Hold; -1 means this is not used
    };

    // Configuration for the SPI slave interface:
    spi_slave_interface_config_t slv_cfg = {
        .spics_io_num = SPI_CS_PIN, // The CS GPIO pin for this device
        .flags = 0,                 // Default
        .queue_size = 3,            // Number of transactions that can be queued
        .mode = 0,                  // SPI mode 0 (CPOL=0, CPHA=0) - matches SPI Master on Basys 3
        .post_setup_cb = NULL,      // NULL means no callback function is used
        .post_trans_cb = NULL       // NULL means no callback function is used
    };

    // Initializing SPI slave peripheral:
    // For ESP32, dma_chan (DMA Channel) can be 1 or 2.
    // SPI_DMA_CH_AUTO means that we let the driver automatically select an available DMA channel.
    // NOTE: DMA will not be needed here; it will be needed when we are sending FFT data via SPI.
    ret = spi_slave_initialize(SLAVE_SPI_HOST, &bus_cfg, &slv_cfg, SPI_DMA_CH_AUTO);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize SPI slave: %s", esp_err_to_name(ret)); // Logging error to initialize
        return;     // Exit the app_main() function; the execution stops; ESP32 will halt
    }

    ESP_LOGI(TAG, "SPI Slave initialized. Waiting for data from master..."); // Logging successful initialization

    spi_slave_transaction_t spi_trans; // Structure to describe the SPI transaction

    while (1) {
        // Every iteration of the while loop processes 1 SPI transaction

        // Clear the transaction structure before every new transaction / iteration of loop:
        memset(&spi_trans, 0, sizeof(spi_trans));

        // Setting the length of the data buffer for reception (in bits):
        spi_trans.length = SPI_BUF_SIZE * 8;
        // The 'length' field specifies the maximum amount of data (in bits)
        // that the ESP32 slave is prepared to receive / send in 1 transaction.
        // While Basys 3 only sends 1 byte, the buffer can be larger;
        // the driver will use trans_len to report actual bits.

        // Assigning the buffer where received data will be stored:
        spi_trans.rx_buffer = recv_buf;

        // Assigning the buffer where transmitted data will be stored:
        spi_trans.tx_buffer = NULL;
        // tx_buffer is NULL since we are only receiving right now.

        // Blocking execution until the SPI master completes a transaction and releases the CS line:
        ret = spi_slave_transmit(SLAVE_SPI_HOST, &spi_trans, portMAX_DELAY);

        if (ret == ESP_OK) {
            // This runs when there are no errors in the SPI transaction

            // spi_trans.trans_len contains the actual number of bits transferred.
            // For a single byte from Basys 3, this should be 8.
            ESP_LOGI(TAG, "Received %u bits.",
                    (unsigned int)spi_trans.trans_len   // Type cast to unsigned int for ESP_LOGx
                    );
            
            if (spi_trans.trans_len == 8) {
                // This runs when a byte was successfully received and no bit was lost in transmission

                ESP_LOGI(TAG, "Received byte: 0x%02X (ASCII: %c)",
                        // 0x%02X prints a 2-digit hex number; %c prints the ASCII equivalent of the byte
                        recv_buf[0],    // The first (and only) byte in the receive buffer
                        (recv_buf[0] >= 32 && recv_buf[0] <= 126) ? recv_buf[0] : '.');
                        // Only printable ASCII characters (decimal 32-126) are displayed; else, '.' is printed.

                if (recv_buf[0] == 'Y') {           // ASCII 'Y' is 0x59
                    motor_run_forward();
                } else if (recv_buf[0] == 'N') {    // ASCII 'N' is 0x4E
                    motor_stop();
                } else {
                    ESP_LOGW(TAG, "Unknown command received: 0x%02X", recv_buf[0]); // Logging warning of unknown byte
                    // Stop the motor on unknown / erroneous byte:
                    motor_stop();
                }
            }

        } else {
            ESP_LOGE(TAG, "SPI slave receive failed: %s", esp_err_to_name(ret));
        }

    }
    // To release the SPI peripheral and associated resources (like DMA channel, GPIO pins):
    // spi_slave_free(SLAVE_SPI_HOST);
    // However, since we want to continuously receive data, we will not run this line.
}
