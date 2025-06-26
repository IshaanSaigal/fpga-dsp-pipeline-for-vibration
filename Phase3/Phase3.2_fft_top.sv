`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: Ishaan Saigal
// 
// Create Date: 10.06.2025 17:47:01
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
    logic rst_n;                        // Internal active-low reset signal
    assign rst_n = RESET_N;
    // The above code seems redundant for the current modules, which both have active-low resets.
    // However, in case any other module in the future requires a reset signal, which may be active-high, we need to have the
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
    
    
    // NOTE: Both the FIFO IP cores are configured in First Word Fall Through (FWFT) mode instead of Standard mode. This reduces latency by 
    // eliminating the initial one-cycle delay on the first read. Although throughput remains the SAME in both modes after the initial read,
    // single-cycle AXIS streaming is much simpler in FWFT mode. Standard mode can only achieve single-cycle streaming using a 
    // complex pipelined look-ahead logic.
    
    // --- Internal Signals for Input FIFO Buffer ---
    logic           fifo_sync_reset;                    // (wire) Unnecessary; just used for better readability
    logic           reset_sync_reg1, reset_sync_reg2;   // 1-bit registers are equivalent to FFs
    // The "srst" port of the FIFO Generator IP is a synchronous reset. "Synchronous" means that the reset only happens on the rising edge of the clk.
    // We cannot directly connect to RESET_N switch since that is asynchronous, and could cause metastability in the FIFO core.
    // Thus, we update this in a separate always_ff block, synchronised with the clk. For this, we will use a two-flop synchronizer chain:
    always_ff @(posedge CLK100MHZ) begin
        // First FF samples the asynchronous switch input. This FF is allowed to go metastable.
        reset_sync_reg1 <= !rst_n;              // This FF samples the asynchronous rst_n signal on the clk edge.
        // Setup time is the minimum time BEFORE the clk edge that the input must be stable.
        // Hold time is the minimum time AFTER the clk edge that the input must be stable.
        // If the rst_n signal changes in this setup/hold window, the reset_sync_reg1 would become metastable.
        
        // Second FF samples the synchronous first FF. It has a full clock cycle for the first FF to settle.
        reset_sync_reg2 <= reset_sync_reg1;     // The output of this will be stable.
        
        // Best-case scenario for de/asserting RESET_N (2 clk cycles):
        // posedge 0: Both registers are stable.
        // posedge 1: RESET_N changes outside the setup/hold window, not causing any metastability in FF1.
        //            FF1 samples the new !rst_n correctly. FF2 samples the old value of FF1 (from before posedge 1).
        // posedge 2: FF2 samples the stable output of FF1.
        
        // Worst-case scenario for de/asserting RESET_N (3 clk cycles):
        // posedge 0: Both registers are stable.
        // posedge 1: RESET_N changes in the setup/hold window, which may or may not cause metastability in FF1.
        //            FF2 samples FF1's output, which may be valid, invalid, or metastable depending on FF1's state.
        // Between posedge 1 and 2: Metastability decays exponentially in FF1, it resolves to a stable 0 or 1.
        //                          This MAY NOT match !rst_n. Let's say it doesn't.
        // posedge 2: FF2 samples the stable output of FF1, without any metastability. However, if FF1 resolved to
        //            the incorrect value, FF2 would sample that incorrect value.
        //            FF1 samples !rst_n again. Since RESET_N doesn't change, FF1 latches the correct value.
        // posedge 3: FF2 samples the stable output of FF1, which is now the correct value.
    end
    assign fifo_sync_reset = reset_sync_reg2;
    
    logic           fifo_in_write_en_reg;       // 1-cycle enable pulse for writing into the buffer; register for sequential I2C logic
    logic [15:0]    fifo_in_write_data_reg;     // Register bus for sequential I2C logic. Only 16-bit real input values are stored to save resources; imag=0
    
    logic           fifo_in_read_en;            // 1-cycle enable pulse for reading from the buffer; wire for combinational AXIS logic
    logic [15:0]    fifo_in_read_data;
    
    logic           fifo_in_full;               // Output wire indicating FIFO is full (1026 samples); should never be HIGH as we only store 1024 samples
    logic           fifo_in_empty;              // Output wire indicating FIFO is empty
    
    // Counter for inputting exactly 1024 samples to the FIFO buffer (1026 is the actual depth):
    logic [10:0]    fifo_in_write_counter_reg;  // Incremented every time a sample is written. 11-bit, since we need to represent values till 1024
                                                // (10-bit is 0-1023)
    
    // NOTE: This buffer's input logic is sequential for slow I2C, but output logic is combinational for fast AXI4-Stream.
    // Combinational logic is needed in AXI4-Stream for reading in a single clock cycle.
    
    
    // --- Internal Signals for the FFT IP ---
    // Signals for AXI4-Stream Master logic (for inputting data to the AXI4-Stream Slave in FFT core):
    logic [31:0]    m_axis_data_tdata;
    logic           m_axis_data_tvalid;
    logic           m_axis_data_tready;
    logic           m_axis_data_tlast;
    
    // Signals for AXI4-Stream Slave logic (for outputting data from the AXI4-Stream Master in FFT core):
    logic [31:0]    s_axis_data_tdata;
    logic           s_axis_data_tvalid;
    logic           s_axis_data_tready;
    logic           s_axis_data_tlast;
    
    // Counter for inputting exactly 1024 samples to the FFT core:
    logic [10:0]    flit_transfer_counter_reg;  // 11-bit, since we need to represent values till 1024
                                                // NOTE: Cannot use fifo_in_empty to assert m_axis_data_tlast, since it must be asserted BEFORE the
                                                // 1024th flit transfer, not AFTER it
    
    
    // --- Internal Signals for Output FIFO Buffer ---
    logic           fifo_out_write_en;          // 1-cycle enable pulse for writing into the buffer; wire for combinational AXIS logic
    // NOTE: Initializing a register fifo_out_write_data_reg would waste a clock cycle; we can directly connect FFT core's output for higher throughput
    
    logic           fifo_out_read_en_reg;       // 1-cycle enable pulse for reading from the buffer; register for sequential SPI logic
    logic [31:0]    fifo_out_read_data;
    
    logic           fifo_out_full;              // Output wire indicating FIFO is full (1026 samples); should never be HIGH as we only store 1024 samples
    logic           fifo_out_empty;             // Output wire indicating FIFO is empty
    
    // NOTE: Unlike the Input FIFO Buffer core, we won't need a counter for inputting exactly 1024 samples here,
    // since we can use the s_axis_data_tlast output signal from the FFT core for indicating the last sample.
    
    // NOTE: This buffer's input logic is combinational for fast AXI4-Stream, but output logic is sequential for slow SPI.
    // Combinational logic is needed in AXI4-Stream for writing in a single clock cycle.
    
    
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
    fifo_buffer in_fifo_buffer_inst (
        .clk(CLK100MHZ),                    // (input wire) System clock
        .srst(fifo_sync_reset),             // (input wire) Synchronous reset
        
        .din(fifo_in_write_data_reg),       // (input wire [15:0])
        .wr_en(fifo_in_write_en_reg),       // (input wire)
        
        .rd_en(fifo_in_read_en),            // (input wire)
        .dout(fifo_in_read_data),           // (output wire [15:0])
        
        .full(fifo_in_full),                // (output wire)
        .empty(fifo_in_empty)               // (output wire)
    );
    
    
    // --- Instantiating the FFT IP ---
    fft_ip fft_ip_inst (
        .aclk(CLK100MHZ),                                           // (input wire) AXI clock
        .aclken(1'b1),                                              // (input wire) AXI clock enable (for clock gating). Tied to '1' / always enabled
        .aresetn(rst_n),                                            // (input wire) AXI active-low reset; asynchronous
                                                                    // NOTE: MUST be asserted for at least 2 clk cycles; handled by RESET_N debouncing logic
        
        // Signals for sending metadata. Ports left unconnected; still need to write connections to avoid Critical Warnings:
        .s_axis_config_tdata(16'b0),                                // (input wire [15:0])
        .s_axis_config_tvalid(1'b0),                                // (input wire) Tied to '0' / no transfer occurs
        .s_axis_config_tready(s_axis_config_tready),                // (output wire)
        
        // AXI4-Stream Slave ports (for input):
        .s_axis_data_tdata(m_axis_data_tdata),                      // (input wire [31:0])
        .s_axis_data_tvalid(m_axis_data_tvalid),                    // (input wire)
        .s_axis_data_tready(m_axis_data_tready),                    // (output wire)
        .s_axis_data_tlast(m_axis_data_tlast),                      // (input wire)
        
        // AXI4-Stream Master ports (for output):
        .m_axis_data_tdata(s_axis_data_tdata),                      // (output wire [31:0])
        .m_axis_data_tvalid(s_axis_data_tvalid),                    // (output wire)
        .m_axis_data_tready(s_axis_data_tready),                    // (input wire)
        .m_axis_data_tlast(s_axis_data_tlast),                      // (output wire)
        
        // Signals for debugging. Ports left unconnected; still need to write connections to avoid Critical Warnings:
        .event_frame_started(event_frame_started),                  // (output wire)
        .event_tlast_unexpected(event_tlast_unexpected),            // (output wire)
        .event_tlast_missing(event_tlast_missing),                  // (output wire)
        .event_status_channel_halt(event_status_channel_halt),      // (output wire)
        .event_data_in_channel_halt(event_data_in_channel_halt),    // (output wire)
        .event_data_out_channel_halt(event_data_out_channel_halt)   // (output wire)
    );
    
    
    // --- Instantiating the FIFO IP for Output Buffer ---
    fifo_generator_0 out_fifo_buffer_inst (
        .clk(CLK100MHZ),                    // (input wire) System clock
        .srst(fifo_sync_reset),             // (input wire) Synchronous reset (using the same signal as the input FIFO buffer)
        
        .din(s_axis_data_tdata),            // (input wire [31:0])
        .wr_en(fifo_out_write_en),          // (input wire)
        
        .rd_en(fifo_out_read_en_reg),       // (input wire)
        .dout(fifo_out_read_data),          // (output wire [31:0])
        
        .full(fifo_out_full),               // (output wire)
        .empty(fifo_out_empty)              // (output wire)
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
        
        // 9. Streaming the 1024-sample input frame from the input FIFO buffer to the FFT core
        S_FFT_INPUT_FRAME,                      // High-speed combinational logic for AXIS flit transfer on every clock cycle
        
        // 10. Streaming the 1024-sample output frame from the FFT core to the output FIFO buffer
        S_FFT_OUTPUT_FRAME,                     // High-speed combinational logic for AXIS flit transfer on every clock cycle
        
        // 11. Sending higher real byte in output FIFO buffer over UART
        S_UART_FIFO_OUT_H_LATCH,                // Latch the higher real byte to the uart_tx input data_to_transmit_reg
        S_UART_FIFO_OUT_H_PULSE_ENABLE,         // Pulse the uart_tx enable wire for 1 system clock cycle
        S_UART_FIFO_OUT_H_WAIT_BUSY_LOW,        // Wait till the uart_tx busy wire is LOW
        
        // 12. Sending lower real byte in output FIFO buffer over UART
        S_UART_FIFO_OUT_L_LATCH,                // Latch the lower real byte to the uart_tx input data_to_transmit_reg
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
            
            flit_transfer_counter_reg <= 11'b0;
            
            fifo_out_read_en_reg <= 1'b0;
            
        end else begin
            // Defaults:
            i2c_enable_to_master_reg <= 1'b0;
            i2c_enable_burst_to_master_reg <= 1'b0;
            // NOTE: The rest of the I2C registers are held undefined/latched right now
            
            fifo_in_write_en_reg <= 1'b0;
            
            fifo_out_read_en_reg <= 1'b0;
            
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
                
                
                // 9. Streaming the 1024-sample input frame from the input FIFO buffer to the FFT core
                
                S_FFT_INPUT_FRAME: begin
                    // Increment counter indicating a flit transfer:
                    if (m_axis_data_tvalid && m_axis_data_tready && !fifo_in_empty) // AXIS handshake: indicates a flit transfer to the FFT core
                        flit_transfer_counter_reg <= flit_transfer_counter_reg + 1; // Increment for every AXIS handshake
                end
                
                
                // 11. Sending higher real byte in output FIFO buffer over UART
                
                S_UART_FIFO_OUT_H_LATCH: begin
                    if (!fifo_out_empty) begin
                        // Latch the higher real byte to uart_tx's data_to_transmit_reg for transmission
                        data_to_transmit_reg <= fifo_out_read_data[15:8];
                    end
                end
                
                
                // 12. Sending lower real byte in output FIFO buffer over UART
                
                S_UART_FIFO_OUT_L_LATCH: begin
                    if (!fifo_out_empty) begin
                        // Latch the lower real byte to uart_tx's data_to_transmit_reg for transmission
                        data_to_transmit_reg <= fifo_out_read_data[7:0];
                        
                        // Assert read enable to request the next word from the output FIFO buffer:
                        fifo_out_read_en_reg <= 1'b1;   // Pulse enable for NEXT word due to FWFT mode
                    end
                end
                
                
                S_DONE: begin
                    // Resetting all the counters used in the FSM:
                    fifo_in_write_counter_reg <= 11'b0;
                    flit_transfer_counter_reg <= 11'b0;
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
        
        // AXIS Master signals:
        m_axis_data_tdata = 32'b0;
        m_axis_data_tvalid = 1'b0;
        m_axis_data_tlast = 1'b0;
        fifo_in_read_en = 1'b0;
        
        // AXIS Slave signals:
        s_axis_data_tready = 1'b0;
        fifo_out_write_en = 1'b0;
        
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
                        next_state = S_FFT_INPUT_FRAME;
                    else
                        next_state = S_I2C_ACCEL_XOUT_CONFIG;               // Loop back to burst reading accelerometer values
                end else
                    next_state = S_UART_WHO_AM_I_WAIT_BUSY_LOW;
            end
            
            
            // 9. Streaming the 1024-sample input frame from the input FIFO buffer to the FFT core
            
            S_FFT_INPUT_FRAME: begin
                if (!fifo_in_empty) begin
                    m_axis_data_tdata = {16'b0, fifo_in_read_data}; // Concatenating 16-bit real with 16-bit imag 0 (real-valued signal)
                    // NOTE: Since the FIFO IP core is configured in FWFT mode, the fifo_in_read_data is already STABLE in this clk cycle.
                    m_axis_data_tvalid = 1'b1;                      // Since above signal is stable, we can pulse m_axis_data_tvalid in this cycle itself
                end
                
                if (m_axis_data_tvalid && m_axis_data_tready && !fifo_in_empty) // AXIS handshake: indicates a flit transfer to the FFT core
                    // NOTE: Flit transfer occurs 1 clk cycle AFTER BOTH m_axis_data_tvalid and m_axis_data_tready are HIGH.
                    fifo_in_read_en = 1'b1; // Current sample inputted to the FFT core; asserting read enable to request the next sample
                                            // NOTE: fifo_in_read_data will be stable by the next clk cycle. This enables single-cycle streaming.
                else
                    fifo_in_read_en = 1'b0;
                
                // Logic for asserting m_axis_data_tlast:
                if (flit_transfer_counter_reg == 1023)  // Indicates 1023 flits have been transferred; the last 1024th flit is remaining
                    m_axis_data_tlast = 1'b1;
                else
                    m_axis_data_tlast = 1'b0;
                
                // Logic for next state:
                if (flit_transfer_counter_reg >= 1024)  // Indicates ALL 1024 flits have been transferred
                    next_state = S_FFT_OUTPUT_FRAME;
                else
                    next_state = S_FFT_INPUT_FRAME;
            end
            
            
            // 10. Streaming the 1024-sample output frame from the FFT core to the output FIFO buffer
            
            S_FFT_OUTPUT_FRAME: begin
                s_axis_data_tready = !fifo_out_full;                                // Output FIFO buffer can accept flit ONLY if it is not full
                
                if (s_axis_data_tvalid && s_axis_data_tready && !fifo_out_full)     // AXIS handshake: indicates a flit transfer from the FFT core
                    fifo_out_write_en = 1'b1;
                else
                    fifo_out_write_en = 1'b0;
                
                // Logic for next state:
                if (s_axis_data_tlast && s_axis_data_tvalid && s_axis_data_tready)  // Transition ONLY after AXIS handshake for the LAST sample
                    next_state = S_UART_FIFO_OUT_H_LATCH;
                else
                    next_state = S_FFT_OUTPUT_FRAME;
            end
            
            
            // 11. Sending higher real byte in output FIFO buffer over UART
            
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
            
            
            // 12. Sending lower real byte in output FIFO buffer over UART
            
            S_UART_FIFO_OUT_L_LATCH: begin
                // Just wait 1 clock cycle to latch the lower byte to uart_tx's data_to_transmit_reg
                next_state = S_UART_FIFO_OUT_L_PULSE_ENABLE;
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
                    if (fifo_out_empty)
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
