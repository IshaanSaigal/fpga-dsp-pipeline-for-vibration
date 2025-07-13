/*
 * This program is for configuring the ESP32 as an SPI slave.
 * It is designed to receive a 6kB data frame (8kB with padding).
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
WORD_ALIGNED_ATTR uint8_t recv_buf[SPI_BUF_SIZE]; // WORD_ALIGNED_ATTR is a requirement for DMA-capable buffers

// NOTE: To send data back, we would similarly define a send_buf:
// WORD_ALIGNED_ATTR uint8_t send_buf[SPI_BUF_SIZE];


// --- SPI Frame and Transaction Definitions ---
#define BYTES_PER_TRANSACTION_DATA      6       // The actual useful data in each transaction
#define BYTES_PER_TRANSACTION_PADDED    8       // The padded size for DMA (6 data + 2 padding bytes)
#define NUM_TRANSACTIONS_PER_FRAME      1024    // Number of transactions to make a full frame
#define TOTAL_FRAME_SIZE                (BYTES_PER_TRANSACTION_DATA * NUM_TRANSACTIONS_PER_FRAME) // 6144 bytes or 6kB

// Buffer to store the entire 6kB frame
static uint8_t full_frame_buffer[TOTAL_FRAME_SIZE];
// NOTE: We make this 'static' so it's not allocated on the small task stack.

// Counter to track received transactions
static int transaction_count = 0;       // Initialized to 0


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

    ESP_LOGI(TAG, "Initializing legacy SPI Slave...");

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

    // Initializing SPI slave peripheral with DMA enabled (for high-volume transfer):
    // For ESP32, dma_chan (DMA Channel) can be 1 or 2.
    // SPI_DMA_CH_AUTO means that we let the driver automatically select an available DMA channel.
    // This limits our SPI Slave to ONLY being able to process transactions that are multiples of 4 bytes.
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
        // While Basys 3 only sends 8 bytes, the buffer can be larger;
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
            
            size_t num_bytes_received = (spi_trans.trans_len + 7) / 8;
            
            // Creating a string to print all received bytes in one line.
            // Max size: 64 bytes * 5 chars/byte ("0xXX ") + null terminator = 321
            char hex_string_buffer[321];
            char *ptr = hex_string_buffer;
            ptr[0] = '\0'; // Start with an empty string
            for (int i = 0; i < num_bytes_received; i++) {
                // Appending "0xXX " to the string for each byte
                ptr += sprintf(ptr, "0x%02X ", recv_buf[i]);
            }
            ESP_LOGI(TAG, "Received data: %s", hex_string_buffer);
            

            if (spi_trans.trans_len == 64) {
                // Runs when all 8 bytes are correctly received. We enter the data into full_frame_buffer.
                // NOTE: Frame won't store the 0-bit transactions sometimes triggered by the 
                // CS line being pulled low on FPGA start-up.

                // Copying the useful 6 bytes from the recv_buf into our large frame buffer
                memcpy(&full_frame_buffer[transaction_count * BYTES_PER_TRANSACTION_DATA], 
                    recv_buf, 
                    BYTES_PER_TRANSACTION_DATA);
                // NOTE: The destination address is calculated based on the current transaction count.
                transaction_count++;    // Incrementing the transaction_count
            }
            
            
            // Checking if we have received the complete frame
            if (transaction_count == NUM_TRANSACTIONS_PER_FRAME) {
                
                ESP_LOGI(TAG, "======================================================");
                ESP_LOGI(TAG, "COMPLETE FRAME RECEIVED: %d bytes of data.", TOTAL_FRAME_SIZE);
                ESP_LOGI(TAG, "======================================================");

                // For verification, we print the first and last few bytes of the frame
                ESP_LOGI(TAG, "First 6 bytes of frame: 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X",
                         full_frame_buffer[0], full_frame_buffer[1], full_frame_buffer[2],
                         full_frame_buffer[3], full_frame_buffer[4], full_frame_buffer[5]);

                ESP_LOGI(TAG, "Last 6 bytes of frame: 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X",
                         full_frame_buffer[TOTAL_FRAME_SIZE - 6], full_frame_buffer[TOTAL_FRAME_SIZE - 5],
                         full_frame_buffer[TOTAL_FRAME_SIZE - 4], full_frame_buffer[TOTAL_FRAME_SIZE - 3],
                         full_frame_buffer[TOTAL_FRAME_SIZE - 2], full_frame_buffer[TOTAL_FRAME_SIZE - 1]);
                
                
                ESP_LOGI(TAG, "Resetting state to receive next frame...");
                transaction_count = 0; // Reset the counter
                memset(full_frame_buffer, 0, TOTAL_FRAME_SIZE); // Clear the frame buffer for the next frame
            }


        } else {
            ESP_LOGE(TAG, "SPI slave receive failed: %s", esp_err_to_name(ret));
        }

        vTaskDelay(pdMS_TO_TICKS(2)); // Delay for 2 ms to reset IDLE0 TWDT

    }
    // To release the SPI peripheral and associated resources (like DMA channel, GPIO pins):
    // spi_slave_free(SLAVE_SPI_HOST);
    // However, since we want to continuously receive data, we will not run this line.
}
