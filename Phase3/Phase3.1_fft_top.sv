`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 26.06.2025 01:28:31
// Design Name: 
// Module Name: fft_top
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////


module fft_top (
    input wire CLK100MHZ,       // 100MHz clock pin, Pin W5
    input wire RESET_N,         // Active-low reset switch, Pin R2
    
    inout wire MPU_SCL,         // PMOD JA, Pin J1
    inout wire MPU_SDA,         // PMOD JA, Pin L2
    // NOTE: Since both SDA and SCL (clock stretching) can be controlled by both i2c_master and slave (MPU-6050),
    // these must be modelled as inout ports.
    
    output wire UART_TXD        // UART Transmit Data line, Pin A18
);
    
    // --- Internal Signals ---
    logic   rst_n;                          // Internal active-low reset signal
    assign  rst_n = RESET_N;
    // The above code seems redundant for modules that have active-low resets.
    // However, in case of modules (like the FIFO IP cores) requiring an active-high reset signal, we need to have the
    // wire rst_n which we can alter as per our requirements (active-high vs active-low), while still using the same external switch.
    
    
    // --- Internal Signals for I2C ---
    logic i2c_enable_to_master_reg;
    logic i2c_read_write_to_master_reg;
    logic [7:0] i2c_mosi_data_to_master_reg;
    logic [7:0] i2c_register_address_to_master_reg;
    localparam IC_MPU_DEVICE_ADDRESS = 7'h68;       // MPU-6050's I2C address. Since this is constant, we use localparam
    localparam I2C_CLK_DIVIDER = 16'd249;           // For 100kHz Standard Mode I2C. 16-bit to match i2c_master. Since this is constant, we use localparam
    // For 100MHz system clk, 100kHz SCL clk:
    // f_SCL = f_CLK / (4 * (divider + 1))
    // => 100kHz = 100MHz / (4 * (divider + 1))
    // => divider = 249
    
    logic [7:0] i2c_miso_data_from_master;
    logic i2c_busy_from_master;
    
    
    // --- Internal Signals for I2C burst read ---
    // NOTE: Having separate signals for the burst instance is recommended to avoid contention.
    // For example, we don't want the same enable signal for both the instances, otherwise they both might try to seize the bus at the same time.
    logic i2c_enable_burst_to_master_reg;
    logic i2c_read_write_burst_to_master_reg;
    logic [15:0] i2c_mosi_data_burst_to_master_reg;     // Only needed for initialization; never actually used since we never do burst writes
    logic [7:0] i2c_register_address_burst_to_master_reg;
    // NOTE: The localparams remain same as the single-byte i2c_master instance
    
    logic [15:0] i2c_miso_data_burst_from_master;       // For 16-bit accelerometer readings
    logic i2c_busy_burst_from_master;
    
    
    // --- Internal Signals for UART ---
    logic uart_tx_en_pulse_sig;         // 1-cycle enable pulse for UART TX module
    logic [7:0] data_to_transmit_reg;   // Register to hold the data to be sent
    
    logic uart_busy_sig;                // Busy signal from UART TX module
    
    
    // NOTE: The FIFO IP cores are configured in First Word Fall Through (FWFT) mode instead of Standard mode. This reduces latency by 
    // eliminating the initial one-cycle delay on the first read. Although throughput remains the SAME in both modes after the initial read,
    // single-cycle AXIS streaming is much simpler in FWFT mode. Standard mode can only achieve single-cycle streaming using a 
    // complex pipelined look-ahead logic.
    
    // --- Internal Signals for Input FIFO Buffer ---
    logic           fifo_in_write_en_reg;       // 1-cycle enable pulse for writing into the buffer; register for sequential I2C logic
    logic [15:0]    fifo_in_write_data_reg;     // Register bus for sequential I2C logic. Only 16-bit real input values are stored to save resources; imag=0
    
    logic           fifo_in_read_en;            // 1-cycle enable pulse for reading from the buffer; wire for combinational AXIS logic
    logic [15:0]    fifo_in_read_data;
    
    logic           fifo_in_full;               // Output wire indicating FIFO is full (1025 samples); should never be HIGH as we only store 1024 samples
    logic           fifo_in_empty;              // Output wire indicating FIFO is empty
    
    // Counter for inputting exactly 1024 samples to the FIFO buffer (1025 is the actual depth):
    logic [10:0]    fifo_in_write_counter_reg;  // Incremented every time a sample is written. 11-bit, since we need to represent values till 1024
                                                // (10-bit is 0-1023)
    
    // NOTE: This buffer's input logic is sequential for slow I2C, but output logic is combinational for fast AXI4-Stream.
    // Combinational logic is needed in AXI4-Stream for reading in a single clock cycle.
    
    
    // --- Instantiating I2C Master module ---
    i2c_master i2c_master_inst (
        .clock(CLK100MHZ),
        .reset_n(rst_n),
        .enable(i2c_enable_to_master_reg),
        .read_write(i2c_read_write_to_master_reg),
        .mosi_data(i2c_mosi_data_to_master_reg),
        .register_address(i2c_register_address_to_master_reg),
        .device_address(IC_MPU_DEVICE_ADDRESS),
        .divider(I2C_CLK_DIVIDER),
        
        .miso_data(i2c_miso_data_from_master),
        .busy(i2c_busy_from_master),
        
        .external_serial_data(MPU_SDA),
        .external_serial_clock(MPU_SCL)
    );
    
    
    // --- Instantiating I2C Master module for burst reads (for accelerometer readings) ---
    i2c_master #(
        .NUMBER_OF_DATA_BYTES(2)    // This parameter allows this instance to perform both burst reads and writes upto 2 bytes
    )
    i2c_master_burst_inst (
        .clock(CLK100MHZ),
        .reset_n(rst_n),
        .enable(i2c_enable_burst_to_master_reg),
        .read_write(i2c_read_write_burst_to_master_reg),
        .mosi_data(i2c_mosi_data_burst_to_master_reg),
        .register_address(i2c_register_address_burst_to_master_reg),
        .device_address(IC_MPU_DEVICE_ADDRESS),
        .divider(I2C_CLK_DIVIDER),
        
        .miso_data(i2c_miso_data_burst_from_master),
        .busy(i2c_busy_burst_from_master),
        
        .external_serial_data(MPU_SDA),
        .external_serial_clock(MPU_SCL)
    );
    
    
    // --- Instantiating UART TX Module ---
    uart_tx #(
        .BIT_RATE(115200),                  // Baud rate
        .CLK_HZ(100_000_000),               // System clock frequency
        .PAYLOAD_BITS(8),                   // Number of data bits
        .STOP_BITS(1)                       // Number of stop bits
    ) uart_transmitter_inst (
        .clk(CLK100MHZ),                    // System clock
        .resetn(rst_n),                     // Active-low reset
        .uart_txd(UART_TXD),                // UART transmit data output pin
        .uart_tx_busy(uart_busy_sig),       // UART busy status output
        .uart_tx_en(uart_tx_en_pulse_sig),  // UART transmit enable (1-cycle pulse)
        .uart_tx_data(data_to_transmit_reg) // Data to be transmitted
    );
    
    
    // --- Instantiating the FIFO IP for Input Buffer ---
    in_fifo_buffer in_fifo_buffer_inst (
        .clk(CLK100MHZ),                    // (input wire) System clock
        .rst(!rst_n),                       // (input wire) Asynchronous active-high reset; negate rst_n to handle active-high
        
        .din(fifo_in_write_data_reg),       // (input wire [15:0])
        .wr_en(fifo_in_write_en_reg),       // (input wire)
        
        .rd_en(fifo_in_read_en),            // (input wire)
        .dout(fifo_in_read_data),           // (output wire [15:0])
        
        .full(fifo_in_full),                // (output wire)
        .empty(fifo_in_empty)               // (output wire)
    );
    
    
    // --- Defining the FSM states ---
    typedef enum logic [5:0] {
        S_IDLE,
        
        // 1. Writing to PWR_MGMT_1 register
        S_I2C_PWR_MGMT_1_CONFIG,                    // Configure the inputs to the i2c_master module
        S_I2C_PWR_MGMT_1_WAIT_BEFORE_ENABLE,        // Ensure that the inputs are stable for 1 clock cycle
        S_I2C_PWR_MGMT_1_ENABLE,                    // Enable the i2c_master module
        S_I2C_PWR_MGMT_1_BUSY_HIGH_ENABLE_LOW,      // If busy is HIGH, pull enable LOW
        S_I2C_PWR_MGMT_1_WAIT_BUSY_LOW,             // Wait till i2c_master busy wire is LOW
        
        // 2. Writing to ACCEL_CONFIG register
        S_I2C_ACCEL_CONFIG_CONFIG,                  // Configure the inputs to the i2c_master module
        S_I2C_ACCEL_CONFIG_WAIT_BEFORE_ENABLE,      // Ensure that the inputs are stable for 1 clock cycle
        S_I2C_ACCEL_CONFIG_ENABLE,                  // Enable the i2c_master module
        S_I2C_ACCEL_CONFIG_BUSY_HIGH_ENABLE_LOW,    // If busy is HIGH, pull enable LOW
        S_I2C_ACCEL_CONFIG_WAIT_BUSY_LOW,           // Wait till i2c_master busy wire is LOW
        
        // 3. Reading ACCEL_XOUT_H and ACCEL_XOUT_L registers (burst read)
        S_I2C_ACCEL_XOUT_CONFIG,                    // Configure the inputs to the burst instance of the i2c_master module
        S_I2C_ACCEL_XOUT_WAIT_BEFORE_ENABLE,        // Ensure that the inputs are stable for 1 clock cycle
        S_I2C_ACCEL_XOUT_ENABLE,                    // Enable the burst instance of the i2c_master module
        S_I2C_ACCEL_XOUT_BUSY_HIGH_ENABLE_LOW,      // If busy is HIGH, pull enable LOW
        S_I2C_ACCEL_XOUT_WAIT_BUSY_LOW,             // Wait till i2c_master busy wire is LOW
        S_I2C_ACCEL_XOUT_READ_OUTPUT,               // Read the ACCEL_XOUT_H and ACCEL_XOUT_L registers
        
        // 4. Writing accelerometer readings to input FIFO buffer
        S_FIFO_IN_WRITE_ENABLE,                 // Pulse fifo_in_write_en_reg for 1 system clock cycle
        
        // 5. Sending ACCEL_XOUT_H over UART
        S_UART_ACCEL_XOUT_H_PULSE_ENABLE,       // Pulse the uart_tx enable wire for 1 system clock cycle
        S_UART_ACCEL_XOUT_H_WAIT_BUSY_LOW,      // Wait till the uart_tx busy wire is LOW
        
        // 6. Sending ACCEL_XOUT_L over UART
        S_UART_ACCEL_XOUT_L_LATCH,              // Latch the ACCEL_XOUT_L value to the uart_tx input data_to_transmit_reg (previously stored ACCEL_XOUT_H)
        S_UART_ACCEL_XOUT_L_PULSE_ENABLE,       // Pulse the uart_tx enable wire for 1 system clock cycle
        S_UART_ACCEL_XOUT_L_WAIT_BUSY_LOW,      // Wait till the uart_tx busy wire is LOW
        
        // 7. Reading WHO_AM_I register
        S_I2C_WHO_AM_I_CONFIG,                  // Configure the inputs to the i2c_master module
        S_I2C_WHO_AM_I_WAIT_BEFORE_ENABLE,      // Ensure that the inputs are stable for 1 clock cycle
        S_I2C_WHO_AM_I_ENABLE,                  // Enable the i2c_master module
        S_I2C_WHO_AM_I_BUSY_HIGH_ENABLE_LOW,    // If busy is HIGH, pull enable LOW
        S_I2C_WHO_AM_I_WAIT_BUSY_LOW,           // Wait till i2c_master busy wire is LOW
        S_I2C_WHO_AM_I_READ_OUTPUT,             // Read the WHO_AM_I register's value
        
        // 8. Sending WHO_AM_I over UART
        S_UART_WHO_AM_I_PULSE_ENABLE,           // Pulse the uart_tx enable wire for 1 system clock cycle
        S_UART_WHO_AM_I_WAIT_BUSY_LOW,          // Wait till the uart_tx busy wire is LOW
        
        // 9. Sending ACCEL_XOUT_H in input FIFO buffer over UART
        S_UART_FIFO_OUT_H_LATCH,                // Latch the FIFO's ACCEL_XOUT_H to the uart_tx input data_to_transmit_reg
        S_UART_FIFO_OUT_H_PULSE_ENABLE,         // Pulse the uart_tx enable wire for 1 system clock cycle
        S_UART_FIFO_OUT_H_WAIT_BUSY_LOW,        // Wait till the uart_tx busy wire is LOW
        
        // 10. Sending ACCEL_XOUT_L in input FIFO buffer over UART
        S_UART_FIFO_OUT_L_LATCH,                // Latch the FIFO's ACCEL_XOUT_L to the uart_tx input data_to_transmit_reg
        S_UART_FIFO_OUT_L_PULSE_ENABLE,         // Pulse the uart_tx enable wire for 1 system clock cycle
        S_UART_FIFO_OUT_L_WAIT_BUSY_LOW,        // Wait till the uart_tx busy wire is LOW
        
        S_DONE                                  // Reset all the counters used in the FSM
    } main_state_t;
    
    main_state_t current_state, next_state;
    
    // NOTE: We need a separate always_ff block to update current_state. Otherwise, if we used the same always_ff block as the
    // rest of the sequential logic, the case(current_state) won't know which current_state to use: the one that just updated
    // vs one that was used in the previous cycle. This causes a Synthesis Error.
    always_ff @(posedge CLK100MHZ or negedge rst_n) begin
        if (!rst_n) begin
            current_state <= S_IDLE;
        end else begin
            current_state <= next_state;
        end
    end
    
    
    // --- FSM Sequential Logic ---
    // NOTE: The I2C logic is both sequential and combinational
    always_ff @(posedge CLK100MHZ or negedge rst_n) begin
        if (!rst_n) begin
            // Reset values for all registers controlled by this FSM
            i2c_enable_to_master_reg <= 1'b0;
            i2c_read_write_to_master_reg <= 1'b0;
            i2c_mosi_data_to_master_reg <= 8'h00;
            i2c_register_address_to_master_reg <= 8'h00;
            
            i2c_enable_burst_to_master_reg <= 1'b0;
            i2c_read_write_burst_to_master_reg <= 1'b0;
            i2c_mosi_data_burst_to_master_reg <= 16'b0;     // Always remains 0, since we never need to burst write
            i2c_register_address_burst_to_master_reg <= 8'h00;
            
            data_to_transmit_reg <= 8'h00;
            
            fifo_in_write_en_reg <= 1'b0;
            fifo_in_write_data_reg <= 16'b0;
            fifo_in_write_counter_reg <= 11'b0;
            
        end else begin
            // Defaults:
            i2c_enable_to_master_reg <= 1'b0;
            i2c_enable_burst_to_master_reg <= 1'b0;
            // NOTE: The rest of the I2C registers are held undefined/latched right now
            
            fifo_in_write_en_reg <= 1'b0;
            // NOTE: The rest of the input FIFO core registers are held undefined/latched right now
            
            case (current_state)
                
                // 1. Writing to PWR_MGMT_1 register
                
                S_I2C_PWR_MGMT_1_CONFIG: begin
                    i2c_read_write_to_master_reg <= 1'b0;           // For writing
                    i2c_mosi_data_to_master_reg <= 8'h01;           // 1) Wake up; 2) Gyro-referenced PLL as clock; 3). Leave temperature sensing enabled 
                    i2c_register_address_to_master_reg <= 8'h6B;    // Address of the PWR_MGMT_1 register
                end
                
                S_I2C_PWR_MGMT_1_ENABLE: begin
                    i2c_enable_to_master_reg <= 1'b1;
                end
                
                S_I2C_PWR_MGMT_1_BUSY_HIGH_ENABLE_LOW: begin
                    if (!i2c_busy_from_master)
                        i2c_enable_to_master_reg <= 1'b1;   // Keep enable high if busy is low
                    else
                        i2c_enable_to_master_reg <= 1'b0;   // Keep enable low if busy is high
                end
                
                
                // 2. Writing to ACCEL_CONFIG register
                
                S_I2C_ACCEL_CONFIG_CONFIG: begin
                    i2c_read_write_to_master_reg <= 1'b0;           // For writing
                    i2c_mosi_data_to_master_reg <= 8'h10;           // For configuring full-scale range of accelerometer as +-8g
                    i2c_register_address_to_master_reg <= 8'h1C;    // Address of the ACCEL_CONFIG register
                end
                
                S_I2C_ACCEL_CONFIG_ENABLE: begin
                    i2c_enable_to_master_reg <= 1'b1;
                end
                
                S_I2C_ACCEL_CONFIG_BUSY_HIGH_ENABLE_LOW: begin
                    if (!i2c_busy_from_master)
                        i2c_enable_to_master_reg <= 1'b1;   // Keep enable high if busy is low
                    else
                        i2c_enable_to_master_reg <= 1'b0;   // Keep enable low if busy is high
                end
                
                
                // 3. Reading ACCEL_XOUT_H and ACCEL_XOUT_L (burst read)
                
                S_I2C_ACCEL_XOUT_CONFIG: begin
                    i2c_read_write_burst_to_master_reg <= 1'b1;         // For reading (burst read 2 bytes)
                    i2c_mosi_data_burst_to_master_reg <= 16'h00;        // We want to read, not write. But we should still define this with a fixed value
                    i2c_register_address_burst_to_master_reg <= 8'h3B;  // Address of the ACCEL_XOUT_H register. Both this and ACCEL_XOUT_L are burst read
                end
                
                S_I2C_ACCEL_XOUT_ENABLE: begin
                    i2c_enable_burst_to_master_reg <= 1'b1;
                end
                
                S_I2C_ACCEL_XOUT_BUSY_HIGH_ENABLE_LOW: begin
                    if (!i2c_busy_burst_from_master)
                        i2c_enable_burst_to_master_reg <= 1'b1;     // Keep enable high if busy is low
                    else
                        i2c_enable_burst_to_master_reg <= 1'b0;     // Keep enable low if busy is high
                end
                
                S_I2C_ACCEL_XOUT_READ_OUTPUT: begin
                    // Actually read the i2c_master module's output:
                    fifo_in_write_data_reg <= i2c_miso_data_burst_from_master;      // Writing to input FIFO buffer
                    
                    // For UART:
                    data_to_transmit_reg <= i2c_miso_data_burst_from_master[15:8];
                    // We read the ACCEL_XOUT_H byte right now.
                    // The ACCEL_XOUT_L still remains on the i2c_miso_data_burst_from_master line, which we read later.
                end
                
                
                // 4. Writing accelerometer readings to input FIFO buffer
                
                S_FIFO_IN_WRITE_ENABLE: begin
                    if (!fifo_in_full) begin
                        fifo_in_write_en_reg <= 1'b1;   // Pulse write enable (check for 1024 samples is handled combinationally)
                        
                        // Increment counter indicating a write to the FIFO buffer:
                        fifo_in_write_counter_reg <= fifo_in_write_counter_reg + 1;
                    end
                end
                
                
                // 6. Sending ACCEL_XOUT_L over UART
                
                S_UART_ACCEL_XOUT_L_LATCH: begin
                    // Latch the ACCEL_XOUT_L to uart_tx's data_to_transmit_reg (previously ACCEL_XOUT_H) for transmission
                    data_to_transmit_reg <= i2c_miso_data_burst_from_master[7:0];
                end
                
                
                // 7. Reading WHO_AM_I register
                
                S_I2C_WHO_AM_I_CONFIG: begin
                    i2c_read_write_to_master_reg <= 1'b1;           // For reading
                    i2c_mosi_data_to_master_reg <= 8'h00;           // We want to read, not write. But we should still define this with a fixed value
                    i2c_register_address_to_master_reg <= 8'h75;    // Address of the WHO_AM_I register
                end
                
                S_I2C_WHO_AM_I_ENABLE: begin
                    i2c_enable_to_master_reg <= 1'b1;
                end
                
                S_I2C_WHO_AM_I_BUSY_HIGH_ENABLE_LOW: begin
                    if (!i2c_busy_from_master)
                        i2c_enable_to_master_reg <= 1'b1;   // Keep enable high if busy is low
                    else
                        i2c_enable_to_master_reg <= 1'b0;   // Keep enable low if busy is high
                end
                
                S_I2C_WHO_AM_I_READ_OUTPUT: begin
                    // Actually read the i2c_master module's output:
                    data_to_transmit_reg <= i2c_miso_data_from_master;      // 8'h68 (ASCII 'h') is the ideal return from WHO_AM_I
                end
                
                
                // 9. Sending ACCEL_XOUT_H in input FIFO buffer over UART
                
                S_UART_FIFO_OUT_H_LATCH: begin
                    if (!fifo_in_empty) begin
                        // Latch the input FIFO's ACCEL_XOUT_H to uart_tx's data_to_transmit_reg for transmission
                        data_to_transmit_reg <= fifo_in_read_data[15:8];
                    end
                end
                
                
                // 10. Sending ACCEL_XOUT_L in input FIFO buffer over UART
                
                S_UART_FIFO_OUT_L_LATCH: begin
                    if (!fifo_in_empty) begin
                        // Latch the input FIFO's ACCEL_XOUT_L to uart_tx's data_to_transmit_reg for transmission
                        data_to_transmit_reg <= fifo_in_read_data[7:0];
                    end
                end
                
                
                S_DONE: begin
                    // Resetting all the counters used in the FSM:
                    fifo_in_write_counter_reg <= 11'b0;
                end
                
            endcase
        end
    end
    
    
    // --- FSM Combinational Logic ---
    // NOTE: The AXI4-Stream and UART logic is purely combinational
    always_comb begin
        // Defaults:
        uart_tx_en_pulse_sig = 1'b0;
        
        next_state = current_state;     // By default, stay in current_state, unless explicitly mentioned in a case
        
        fifo_in_read_en = 1'b0;
        
        case (current_state)
            
            S_IDLE: begin
                next_state = S_I2C_PWR_MGMT_1_CONFIG;
            end
            
            
            // 1. Writing to PWR_MGMT_1 register
            
            S_I2C_PWR_MGMT_1_CONFIG: begin
                next_state = S_I2C_PWR_MGMT_1_WAIT_BEFORE_ENABLE;
            end
            
            S_I2C_PWR_MGMT_1_WAIT_BEFORE_ENABLE: begin
                // Wait 1 clock cycle for the configuration values to stabilize
                if (!i2c_busy_from_master) begin
                    next_state = S_I2C_PWR_MGMT_1_ENABLE;               // If I2C core is idle, proceed to enable it
                end else begin
                    next_state = S_I2C_PWR_MGMT_1_WAIT_BEFORE_ENABLE;   // Else, keep waiting
                end
            end
            
            S_I2C_PWR_MGMT_1_ENABLE: begin
                next_state = S_I2C_PWR_MGMT_1_BUSY_HIGH_ENABLE_LOW;
            end
            
            S_I2C_PWR_MGMT_1_BUSY_HIGH_ENABLE_LOW: begin
                // Wait till busy signal is HIGH to set enable signal LOW
                if (!i2c_enable_to_master_reg && i2c_busy_from_master)
                    next_state = S_I2C_PWR_MGMT_1_WAIT_BUSY_LOW;
                else
                    next_state = S_I2C_PWR_MGMT_1_BUSY_HIGH_ENABLE_LOW;
            end
            
            S_I2C_PWR_MGMT_1_WAIT_BUSY_LOW: begin
                // Wait till busy is LOW, which means I2C transaction is complete
                if (!i2c_busy_from_master)
                    next_state = S_I2C_ACCEL_CONFIG_CONFIG;
                else
                    next_state = S_I2C_PWR_MGMT_1_WAIT_BUSY_LOW;
            end
            
            
            // 2. Writing to ACCEL_CONFIG register
            
            S_I2C_ACCEL_CONFIG_CONFIG: begin
                next_state = S_I2C_ACCEL_CONFIG_WAIT_BEFORE_ENABLE;
            end
            
            S_I2C_ACCEL_CONFIG_WAIT_BEFORE_ENABLE: begin
                // Wait 1 clock cycle for the configuration values to stabilize
                if (!i2c_busy_from_master) begin
                    next_state = S_I2C_ACCEL_CONFIG_ENABLE;             // If I2C core is idle, proceed to enable it
                end else begin
                    next_state = S_I2C_ACCEL_CONFIG_WAIT_BEFORE_ENABLE; // Else, keep waiting
                end
            end
            
            S_I2C_ACCEL_CONFIG_ENABLE: begin
                next_state = S_I2C_ACCEL_CONFIG_BUSY_HIGH_ENABLE_LOW;
            end
            
            S_I2C_ACCEL_CONFIG_BUSY_HIGH_ENABLE_LOW: begin
                // Wait till busy signal is HIGH to set enable signal LOW
                if (!i2c_enable_to_master_reg && i2c_busy_from_master)
                    next_state = S_I2C_ACCEL_CONFIG_WAIT_BUSY_LOW;
                else
                    next_state = S_I2C_ACCEL_CONFIG_BUSY_HIGH_ENABLE_LOW;
            end
            
            S_I2C_ACCEL_CONFIG_WAIT_BUSY_LOW: begin
                // Wait till busy is LOW, which means I2C transaction is complete
                if (!i2c_busy_from_master)
                    next_state = S_I2C_ACCEL_XOUT_CONFIG;
                else
                    next_state = S_I2C_ACCEL_CONFIG_WAIT_BUSY_LOW;
            end
            
            
            // 3. Reading ACCEL_XOUT_H and ACCEL_XOUT_L (burst read)
            
            S_I2C_ACCEL_XOUT_CONFIG: begin
                next_state = S_I2C_ACCEL_XOUT_WAIT_BEFORE_ENABLE;
            end
            
            S_I2C_ACCEL_XOUT_WAIT_BEFORE_ENABLE: begin
                // Wait 1 clock cycle for the configuration values to stabilize
                if (!i2c_busy_burst_from_master) begin
                    next_state = S_I2C_ACCEL_XOUT_ENABLE;               // If I2C burst instance is idle, proceed to enable it
                end else begin
                    next_state = S_I2C_ACCEL_XOUT_WAIT_BEFORE_ENABLE;   // Else, keep waiting
                end
            end
            
            S_I2C_ACCEL_XOUT_ENABLE: begin
                next_state = S_I2C_ACCEL_XOUT_BUSY_HIGH_ENABLE_LOW;
            end
            
            S_I2C_ACCEL_XOUT_BUSY_HIGH_ENABLE_LOW: begin
                // Wait till busy signal is HIGH to set enable signal LOW
                if (!i2c_enable_burst_to_master_reg && i2c_busy_burst_from_master)
                    next_state = S_I2C_ACCEL_XOUT_WAIT_BUSY_LOW;
                else
                    next_state = S_I2C_ACCEL_XOUT_BUSY_HIGH_ENABLE_LOW;
            end
            
            S_I2C_ACCEL_XOUT_WAIT_BUSY_LOW: begin
                // Wait till busy is LOW, which means I2C transaction is complete
                if (!i2c_busy_burst_from_master)
                    next_state = S_I2C_ACCEL_XOUT_READ_OUTPUT;
                else
                    next_state = S_I2C_ACCEL_XOUT_WAIT_BUSY_LOW;
            end
            
            S_I2C_ACCEL_XOUT_READ_OUTPUT: begin
                next_state = S_FIFO_IN_WRITE_ENABLE;
            end
            
            
            // 4. Writing accelerometer readings to input FIFO buffer
            
            S_FIFO_IN_WRITE_ENABLE: begin
                next_state = S_UART_ACCEL_XOUT_H_PULSE_ENABLE;
            end
            
            
            // 5. Sending ACCEL_XOUT_H over UART
            
            S_UART_ACCEL_XOUT_H_PULSE_ENABLE: begin
                uart_tx_en_pulse_sig = !uart_busy_sig && rst_n;         // HIGH only when the UART TX module is NOT busy and NOT in reset
                // uart_tx_en_pulse_sig is HIGH for 1 cycle, since the FSM states last 1 clock cycle each
                
                if (uart_tx_en_pulse_sig)
                    next_state = S_UART_ACCEL_XOUT_H_WAIT_BUSY_LOW;     // Only transition to the next state once enable signal is HIGH
                else
                    next_state = S_UART_ACCEL_XOUT_H_PULSE_ENABLE;
            end
            
            S_UART_ACCEL_XOUT_H_WAIT_BUSY_LOW: begin
                // Transition when UART TX module is not busy, else wait here till communication is finished
                if (!uart_busy_sig)
                    next_state = S_UART_ACCEL_XOUT_L_LATCH;
                else
                    next_state = S_UART_ACCEL_XOUT_H_WAIT_BUSY_LOW;
            end
            
            
            // 6. Sending ACCEL_XOUT_L over UART
            
            S_UART_ACCEL_XOUT_L_LATCH: begin
                // Just wait 1 clock cycle to latch the ACCEL_XOUT_L to uart_tx's data_to_transmit_reg
                next_state = S_UART_ACCEL_XOUT_L_PULSE_ENABLE;
            end
            
            S_UART_ACCEL_XOUT_L_PULSE_ENABLE: begin
                uart_tx_en_pulse_sig = !uart_busy_sig && rst_n;         // HIGH only when the UART TX module is NOT busy and NOT in reset
                // uart_tx_en_pulse_sig is HIGH for 1 cycle, since the FSM states last 1 clock cycle each
                
                if (uart_tx_en_pulse_sig)
                    next_state = S_UART_ACCEL_XOUT_L_WAIT_BUSY_LOW;     // Only transition to the next state once enable signal is HIGH
                else
                    next_state = S_UART_ACCEL_XOUT_L_PULSE_ENABLE;
            end
            
            S_UART_ACCEL_XOUT_L_WAIT_BUSY_LOW: begin
                // Transition when UART TX module is not busy, else wait here till communication is finished
                if (!uart_busy_sig)
                    next_state = S_I2C_WHO_AM_I_CONFIG;
                else
                    next_state = S_UART_ACCEL_XOUT_L_WAIT_BUSY_LOW;
            end
            
            
            // 7. Reading WHO_AM_I register
            
            S_I2C_WHO_AM_I_CONFIG: begin
                next_state = S_I2C_WHO_AM_I_WAIT_BEFORE_ENABLE;
            end
            
            S_I2C_WHO_AM_I_WAIT_BEFORE_ENABLE: begin
                // Wait 1 clock cycle for the configuration values to stabilize
                if (!i2c_busy_from_master) begin
                    next_state = S_I2C_WHO_AM_I_ENABLE;             // If I2C core is idle, proceed to enable it
                end else begin
                    next_state = S_I2C_WHO_AM_I_WAIT_BEFORE_ENABLE; // Else, keep waiting
                end
            end
            
            S_I2C_WHO_AM_I_ENABLE: begin
                next_state = S_I2C_WHO_AM_I_BUSY_HIGH_ENABLE_LOW;
            end
            
            S_I2C_WHO_AM_I_BUSY_HIGH_ENABLE_LOW: begin
                // Wait till busy signal is HIGH to set enable signal LOW
                if (!i2c_enable_to_master_reg && i2c_busy_from_master)
                    next_state = S_I2C_WHO_AM_I_WAIT_BUSY_LOW;
                else
                    next_state = S_I2C_WHO_AM_I_BUSY_HIGH_ENABLE_LOW;
            end
            
            S_I2C_WHO_AM_I_WAIT_BUSY_LOW: begin
                // Wait till busy is LOW, which means I2C transaction is complete
                if (!i2c_busy_from_master)
                    next_state = S_I2C_WHO_AM_I_READ_OUTPUT;
                else
                    next_state = S_I2C_WHO_AM_I_WAIT_BUSY_LOW;
            end
            
            S_I2C_WHO_AM_I_READ_OUTPUT: begin
                next_state = S_UART_WHO_AM_I_PULSE_ENABLE;
            end
            
            
            // 8. Sending WHO_AM_I over UART
            
            S_UART_WHO_AM_I_PULSE_ENABLE: begin
                uart_tx_en_pulse_sig = !uart_busy_sig && rst_n;     // HIGH only when the UART TX module is NOT busy and NOT in reset
                // uart_tx_en_pulse_sig is HIGH for 1 cycle, since the FSM states last 1 clock cycle each
                
                if (uart_tx_en_pulse_sig)
                    next_state = S_UART_WHO_AM_I_WAIT_BUSY_LOW;     // Only transition to the next state once enable signal is HIGH
                else
                    next_state = S_UART_WHO_AM_I_PULSE_ENABLE;
            end
            
            S_UART_WHO_AM_I_WAIT_BUSY_LOW: begin
                // Transition when UART TX module is not busy, else wait here till communication is finished
                if (!uart_busy_sig) begin
                    if (fifo_in_write_counter_reg >= 1024 || fifo_in_full)  // If 1024 samples written to FIFO buffer, transition to FFT AXIS logic
                        next_state = S_UART_FIFO_OUT_H_LATCH;
                    else
                        next_state = S_I2C_ACCEL_XOUT_CONFIG;               // Loop back to burst reading accelerometer values
                end else
                    next_state = S_UART_WHO_AM_I_WAIT_BUSY_LOW;
            end
            
            
            // 9. Sending ACCEL_XOUT_H in input FIFO buffer over UART
            
            S_UART_FIFO_OUT_H_LATCH: begin
                // Just wait 1 clock cycle to latch the higher byte to uart_tx's data_to_transmit_reg
                next_state = S_UART_FIFO_OUT_H_PULSE_ENABLE;
            end
            
            S_UART_FIFO_OUT_H_PULSE_ENABLE: begin
                uart_tx_en_pulse_sig = !uart_busy_sig && rst_n;         // HIGH only when the UART TX module is NOT busy and NOT in reset
                // uart_tx_en_pulse_sig is HIGH for 1 cycle, since the FSM states last 1 clock cycle each
                
                if (uart_tx_en_pulse_sig)
                    next_state = S_UART_FIFO_OUT_H_WAIT_BUSY_LOW;       // Only transition to the next state once enable signal is HIGH
                else
                    next_state = S_UART_FIFO_OUT_H_PULSE_ENABLE;
            end
            
            S_UART_FIFO_OUT_H_WAIT_BUSY_LOW: begin
                // Transition when UART TX module is not busy, else wait here till communication is finished
                if (!uart_busy_sig)
                    next_state = S_UART_FIFO_OUT_L_LATCH;
                else
                    next_state = S_UART_FIFO_OUT_H_WAIT_BUSY_LOW;
            end
            
            
            // 10. Sending ACCEL_XOUT_L in input FIFO buffer over UART
            
            S_UART_FIFO_OUT_L_LATCH: begin
                // Just wait 1 clock cycle to latch the lower byte to uart_tx's data_to_transmit_reg
                next_state = S_UART_FIFO_OUT_L_PULSE_ENABLE;
                
                // Assert read enable to request the next word from the input FIFO buffer:
                if (!fifo_in_empty)
                    fifo_in_read_en = 1'b1;    // Pulse enable for NEXT word due to FWFT mode
            end
            
            S_UART_FIFO_OUT_L_PULSE_ENABLE: begin
                uart_tx_en_pulse_sig = !uart_busy_sig && rst_n;         // HIGH only when the UART TX module is NOT busy and NOT in reset
                // uart_tx_en_pulse_sig is HIGH for 1 cycle, since the FSM states last 1 clock cycle each
                
                if (uart_tx_en_pulse_sig)
                    next_state = S_UART_FIFO_OUT_L_WAIT_BUSY_LOW;       // Only transition to the next state once enable signal is HIGH
                else
                    next_state = S_UART_FIFO_OUT_L_PULSE_ENABLE;
            end
            
            S_UART_FIFO_OUT_L_WAIT_BUSY_LOW: begin
                // Transition when UART TX module is not busy, else wait here till communication is finished
                if (!uart_busy_sig) begin
                    if (fifo_in_empty)
                        next_state = S_DONE;
                    else
                        next_state = S_UART_FIFO_OUT_H_LATCH;
                end else
                    next_state = S_UART_FIFO_OUT_L_WAIT_BUSY_LOW;
            end
            
            
            S_DONE: begin
                next_state = S_I2C_ACCEL_XOUT_CONFIG;       // Loop back to burst reading accelerometer values
            end
            
        endcase
    end
    
endmodule
