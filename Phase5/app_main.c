/* 
 * This program is for configuring the ESP32 as an SPI slave, motor driver, and MQTT client (over TCP).
 * It is designed to:
 * - Receive a 6 kB (8 kB with padding) FFT frame from FPGA via SPI.
 * - Publish the complete FFT frame to an MQTT topic.
 * - Publish the state of FPGA's motor control pin to another MQTT topic.
 * - Subscribe to an MQTT command topic to receive ON/OFF override commands for the motor.
 */


// --- Including Necessary Header Files ---

// Standard C headers:
#include <stdio.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <stdlib.h>
#include <inttypes.h>

// Needed for FreeRTOS tasks and delays:
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// Needed for logging (ESP_LOGI, ESP_LOGE, ESP_LOGW):
#include "esp_log.h"

// Needed for GPIO configuration:
#include "driver/gpio.h"

// Needed for Legacy SPI Slave driver:
#include "driver/spi_slave.h" // This also includes driver/spi_common.h where VSPI_HOST is defined

// Needed for error handling (ESP_ERROR_CHECK)
#include "esp_system.h"

// Needed for saving WiFi details using NVS
#include "nvs_flash.h" // This is the API for the Non-Volatile Storage (NVS) library
// NOTE: NVS is a key-value storage system on the ESP32's flash memory that persists even after the device is turned off.
// The underlying Wi-Fi driver uses NVS to save the network's SSID and password.
// This allows the ESP32 to automatically reconnect to the Wi-Fi after a reboot without having to re-configure it.

// Needed for the system-wide event handling loop
#include "esp_event.h"
// NOTE: The mqtt_event_handler() function creates MQTT events which our system needs to react to.

// Needed for handling Network Interface (netif) abstraction layer
#include "esp_netif.h"
// The netif layer provides a unified way for the TCP/IP stack 
// to interact with different physical network hardware (like Wi-Fi or Ethernet).

// Needed for handling WiFi connection
#include "protocol_examples_common.h"

// Needed for MQTT Client
#include "mqtt_client.h"


// --- Pin Definitions ---

// SPI Pins (defaults for VSPI / SPI3)
#define SPI_CS_PIN      GPIO_NUM_5
#define SPI_SCLK_PIN    GPIO_NUM_18
#define SPI_MOSI_PIN    GPIO_NUM_23
#define SPI_MISO_PIN    GPIO_NUM_19

// Motor Control Pins
#define MOTOR_CTRL_PIN  GPIO_NUM_21     // FPGA output indicating ab/normal vibration
#define L293D_IN1_PIN   GPIO_NUM_25
#define L293D_IN2_PIN   GPIO_NUM_26


// --- Logging Tags ---
// (the first argument in the ESP_LOGx functions;
// determines that the log message corresponds to these modules)
static const char *TAG_SPI = "SPI_SLAVE";
static const char *TAG_MOTOR = "MOTOR";
static const char *TAG_MQTT = "MQTT_APP";


// --- SPI Definitions ---
// Using VSPI_HOST (maps to SPI3), which is defined in driver/spi_common.h (included by driver/spi_slave.h)
#define SLAVE_SPI_HOST VSPI_HOST

// --- SPI Frame and Transaction Definitions ---
#define BYTES_PER_TRANSACTION_DATA      6       // The actual useful data in each transaction
#define BYTES_PER_TRANSACTION_PADDED    8       // The padded size for DMA (6 data + 2 padding bytes)
#define NUM_TRANSACTIONS_PER_FRAME      1024    // Number of transactions to make a full frame
#define TOTAL_FRAME_SIZE                (BYTES_PER_TRANSACTION_DATA * NUM_TRANSACTIONS_PER_FRAME) // 6144 bytes or 6kB

// Buffer to store the entire 6kB frame
static uint8_t full_frame_buffer[TOTAL_FRAME_SIZE];
// NOTE: We make this 'static' so it's not allocated on the small task stack.


// --- MQTT Definitions ---
#define MQTT_TOPIC_FFT_DATA        "esp32/fft_data"
#define MQTT_TOPIC_MOTOR_STATE     "esp32/motor_state"
#define MQTT_TOPIC_MOTOR_COMMAND   "rpi5/motor_override"

// Global MQTT client handle so it can be accessed by different tasks
esp_mqtt_client_handle_t client; 


// --- Function Declarations for Motor Control ---

void motor_setup();
void motor_run_forward();
void motor_stop();

void motor_setup() {
    gpio_reset_pin(MOTOR_CTRL_PIN);                         // Resets the pin to its default state
    gpio_set_direction(MOTOR_CTRL_PIN, GPIO_MODE_INPUT);    // Configures the pin as a GPIO input

    gpio_reset_pin(L293D_IN1_PIN);
    gpio_set_direction(L293D_IN1_PIN, GPIO_MODE_OUTPUT);    // Configures the pins as a GPIO output
    gpio_reset_pin(L293D_IN2_PIN);
    gpio_set_direction(L293D_IN2_PIN, GPIO_MODE_OUTPUT);

    ESP_LOGI(TAG_MOTOR, "Motor GPIOs initialized."); // Logging successful initialization
    motor_run_forward();    // Ensures the motor is initially running for FPGA processing
}

void motor_run_forward() {
    ESP_LOGI(TAG_SPI, "Command: Motor Run Forward");
    gpio_set_level(L293D_IN1_PIN, 1);   // Set pin to '1' / HIGH
    gpio_set_level(L293D_IN2_PIN, 0);
}

void motor_stop() {
    ESP_LOGI(TAG_SPI, "Command: Motor Stop");
    gpio_set_level(L293D_IN1_PIN, 0);   // Set pin to '0' / LOW
    gpio_set_level(L293D_IN2_PIN, 0);
}


// --- SPI Setup Function ---

void spi_slave_setup() {
    /* This function is for setting up the Legacy SPI Slave.*/

    esp_err_t ret; // Special integer type variable used by many ESP-IDF functions to return error codes

    ESP_LOGI(TAG_SPI, "Initializing legacy SPI Slave...");

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
        ESP_LOGE(TAG_SPI, "Failed to initialize SPI slave: %s", esp_err_to_name(ret)); // Logging error to initialize
        return;     // Exit the app_main() function; the execution stops; ESP32 will halt
    }

    ESP_LOGI(TAG_SPI, "SPI Slave initialized. Waiting for data from master..."); // Logging successful initialization
}


// --- SPI Receiver FreeRTOS Task ---

void spi_receiver_task(void *pvParameters) {
    /*This task’s job is to wait and collect SPI data, and forward it for publishing.
    When this operation is organized in a task, it would not hold up anything else.*/

    // Buffer to store received data
    WORD_ALIGNED_ATTR uint8_t recv_buf[BYTES_PER_TRANSACTION_PADDED]; // WORD_ALIGNED_ATTR is a requirement for DMA-capable buffers
    // NOTE: To send data back, we would similarly define a send_buf:
    // WORD_ALIGNED_ATTR uint8_t send_buf[SPI_BUF_SIZE];

    // Counter to track received transactions
    int transaction_count = 0;          // Initialized to 0

    spi_slave_transaction_t spi_trans;  // Structure to describe the SPI transaction

    while (1) {
        // Every iteration of the while loop processes 1 SPI transaction

        // Clear the transaction structure before every new transaction / iteration of loop:
        memset(&spi_trans, 0, sizeof(spi_trans));

        // Setting the length of the data buffer for reception (in bits):
        spi_trans.length = BYTES_PER_TRANSACTION_PADDED * 8;
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
        esp_err_t ret = spi_slave_transmit(SLAVE_SPI_HOST, &spi_trans, portMAX_DELAY);

        if (ret == ESP_OK) {
            // This runs when there are no errors in the SPI transaction

            // spi_trans.trans_len contains the actual number of bits transferred.
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
            

            if (spi_trans.trans_len == 64) {
                // Runs when all 8 bytes are correctly received. We enter the data into full_frame_buffer.
                // NOTE: Frame won't store the 0-bit transactions sometimes triggered by the 
                // CS line being pulled low on FPGA start-up.

                // Logging the correct transaction
                /*ESP_LOGI(TAG, "RX'd %u bits: %s",
                        (unsigned int)spi_trans.trans_len,  // Type cast to unsigned int for ESP_LOGx
                        hex_string_buffer);*/
                // NOTE: Commenting this out, since it massively increases latency

                // Copying the useful 6 bytes from the recv_buf into our large frame buffer
                memcpy(&full_frame_buffer[transaction_count * BYTES_PER_TRANSACTION_DATA], 
                    recv_buf, 
                    BYTES_PER_TRANSACTION_DATA);
                // NOTE: The destination address is calculated based on the current transaction count.
                transaction_count++;    // Incrementing the transaction_count
            } else {
                // Logging the erroneous transaction
                ESP_LOGE(TAG_SPI, "RX'd %u bits: %s",
                        (unsigned int)spi_trans.trans_len,  // Type cast to unsigned int for ESP_LOGx
                        hex_string_buffer);
            }
            
            
            // Checking if we have received the complete frame
            if (transaction_count == NUM_TRANSACTIONS_PER_FRAME) {
                
                ESP_LOGI(TAG_SPI, "======================================================");
                ESP_LOGI(TAG_SPI, "COMPLETE FRAME RECEIVED: %d bytes of data.", TOTAL_FRAME_SIZE);
                ESP_LOGI(TAG_SPI, "======================================================");

                // For verification, we print the first and last few bytes of the frame
                ESP_LOGI(TAG_SPI, "First 6 bytes of frame: 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X",
                         full_frame_buffer[0], full_frame_buffer[1], full_frame_buffer[2],
                         full_frame_buffer[3], full_frame_buffer[4], full_frame_buffer[5]);

                ESP_LOGI(TAG_SPI, "Last 6 bytes of frame: 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X",
                         full_frame_buffer[TOTAL_FRAME_SIZE - 6], full_frame_buffer[TOTAL_FRAME_SIZE - 5],
                         full_frame_buffer[TOTAL_FRAME_SIZE - 4], full_frame_buffer[TOTAL_FRAME_SIZE - 3],
                         full_frame_buffer[TOTAL_FRAME_SIZE - 2], full_frame_buffer[TOTAL_FRAME_SIZE - 1]);
                
                
                // Publish the frame via MQTT
                if (client) { // Make sure MQTT client is initialized
                    ESP_LOGI(TAG_MQTT, "Publishing complete frame via MQTT...");
                    esp_mqtt_client_publish(
                        client, 
                        MQTT_TOPIC_FFT_DATA, 
                        (const char*)full_frame_buffer, 
                        TOTAL_FRAME_SIZE, 
                        0, // QoS 0 to ensure that packets are received at most once
                        0  // Retain = 0
                    );
                }
                

                // Reset the state for next frame
                ESP_LOGI(TAG_SPI, "Resetting state to receive next frame...");
                transaction_count = 0; // Reset the counter
                memset(full_frame_buffer, 0, TOTAL_FRAME_SIZE); // Clear the frame buffer for the next frame
            }


        } else {
            ESP_LOGE(TAG_SPI, "SPI slave receive failed: %s", esp_err_to_name(ret));
        }


        // Delay for 2 ms to reset IDLE0 TWDT:
        vTaskDelay(pdMS_TO_TICKS(2));
    }
    // To release the SPI peripheral and associated resources (like DMA channel, GPIO pins):
    // spi_slave_free(SLAVE_SPI_HOST);
    // However, since we want to continuously receive data, we will not run this line.
}


// --- Motor Status FreeRTOS Task ---

void motor_status_publisher_task(void *pvParameters) {
    /*This task’s job is to check the MOTOR_CTRL pin,
    and forward the motor state for publishing if it changes.*/

    // Flag for last known state
    int last_known_state = -1;  // Initialize to garbage value

    while (1) {
        vTaskDelay(200 / portTICK_PERIOD_MS); // Check every 200ms
        int current_state = gpio_get_level(MOTOR_CTRL_PIN);
        
        if (current_state != last_known_state) {
            last_known_state = current_state;
            const char *state_str = (current_state == 1) ? "OK" : "ERR";

            // Publish the status via MQTT
            if (client) { // Make sure MQTT client is initialized
                esp_mqtt_client_publish(client, MQTT_TOPIC_MOTOR_STATE, state_str, 0, 1, 0); // QoS 1 to ensure packets are received at least once
                ESP_LOGI(TAG_MQTT, "Sent motor state: %s", state_str);
            }
        }
    }
}


// NOTE: The MQTT FreeRTOS Task runs the background and is handled by the mqtt_client library


// --- MQTT Event Handler ---

static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data) {
    /* This is the event handler for unexpected, asynchronous MQTT events.
     *
     * @brief Event handler registered to receive MQTT events
     *
     *  This function is called by the MQTT client event loop.
     *
     * @param handler_args user data registered to the event.
     * @param base Event base for the handler(always MQTT Base in this example).
     * @param event_id The id for the received event.
     * @param event_data The data for the event, esp_mqtt_event_handle_t.
     */
    
    esp_mqtt_event_handle_t event = event_data;

    switch ((esp_mqtt_event_id_t)event_id) {

        case MQTT_EVENT_CONNECTED:
            ESP_LOGI(TAG_MQTT, "MQTT_EVENT_CONNECTED");
            // Subscribe to the rpi/motor_override topic:
            esp_mqtt_client_subscribe(client, MQTT_TOPIC_MOTOR_COMMAND, 1);
            break;
        
        case MQTT_EVENT_DISCONNECTED:
            ESP_LOGI(TAG_MQTT, "MQTT_EVENT_DISCONNECTED");
            break;
        
        case MQTT_EVENT_DATA:
            ESP_LOGI(TAG_MQTT, "MQTT_EVENT_DATA");
            if (strncmp(event->topic, MQTT_TOPIC_MOTOR_COMMAND, event->topic_len) == 0) {
                if (strncmp(event->data, "ON", event->data_len) == 0) {
                    motor_run_forward();
                } else if (strncmp(event->data, "OFF", event->data_len) == 0) {
                    motor_stop();
                }
            }
            break;
        
        case MQTT_EVENT_ERROR:
            ESP_LOGI(TAG_MQTT, "MQTT_EVENT_ERROR");
            break;
        
        default:
            // Other events like SUBSCRIBED, PUBLISHED are logged by default if log level is verbose
            break;
    }
}


// --- Main Function ---

void app_main(void)
{
    
    ESP_LOGI(TAG_MQTT, "[APP] Startup...");
    
    // Initialize NVS Flash - required for WiFi
    ESP_ERROR_CHECK(nvs_flash_init());
    
    // Initialize networking stack
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    // Setup all peripherals
    motor_setup();
    spi_slave_setup();

    // Connect to Wi-Fi (from protocol_examples_common)
    ESP_ERROR_CHECK(example_connect());

    // Configure and start MQTT client
    esp_mqtt_client_config_t mqtt_cfg = {
        .broker.address.uri = CONFIG_BROKER_URL,
    };
    client = esp_mqtt_client_init(&mqtt_cfg);
    esp_mqtt_client_register_event(client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
    esp_mqtt_client_start(client);

    // Give MQTT time to connect before starting publisher tasks.
    // This prevents trying to publish before connected.
    vTaskDelay(2000 / portTICK_PERIOD_MS);

    ESP_LOGI(TAG_MQTT, "[APP] Initializations complete. Starting tasks.");
    
    // Create the application tasks
    xTaskCreate(spi_receiver_task, "spi_receiver_task", 4096, NULL, 10, NULL);
    xTaskCreate(motor_status_publisher_task, "motor_status_task", 2048, NULL, 5, NULL);

}
