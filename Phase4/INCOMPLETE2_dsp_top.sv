`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
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


module dsp_top #(
    parameter THRESHOLD_SENSITIVITY = 50_000       // threshold for comparison above avg values in ROM
) (
    input wire CLK100MHZ,       // 100MHz clock pin, Pin W5
    input wire RESET_N,         // Active-low reset switch, Pin R2
    
    inout wire MPU_SCL,         // PMOD JA, Pin J1
    inout wire MPU_SDA,         // PMOD JA, Pin L2
    // NOTE: Since both SDA and SCL (clock stretching) can be controlled by both i2c_master and slave (MPU-6050),
    // these must be modelled as inout ports.
    
    output wire UART_TXD,       // UART Transmit Data line, Pin A18
    
    output reg MOTOR_CTRL,      // PMOD JB, Pin A15; registered to continuously hold value
    
    output wire SPI_CS_N,       // PMOD JB, Pin A14
    output wire SPI_SCLK,       // PMOD JB, Pin A16
    output wire SPI_MOSI,       // PMOD JB, Pin B15
    input  wire SPI_MISO        // PMOD JB, Pin B16
);
    
    // --- Internal Signals ---
    logic   rst_n;                          // Internal active-low reset signal (synchronous due to debouncer module)
    // The above code seems redundant for modules that have active-low resets.
    // However, in case of modules (like the FIFO IP cores) requiring an active-high reset signal, we need to have the
    // wire rst_n which we can alter as per our requirements (active-high vs active-low), while still using the same external switch.
    
    
    // --- Internal Signals for I2C ---
    logic i2c_enable_to_master_reg;
    logic i2c_read_write_to_master_reg;
    logic [7:0] i2c_mosi_data_to_master_reg;
    logic [7:0] i2c_register_address_to_master_reg;
    localparam IC_MPU_DEVICE_ADDRESS = 7'h68;       // MPU-6050's I2C address. Since this is constant, we use localparam
    localparam I2C_CLK_DIVIDER = 16'd62;           // For 100kHz Standard Mode I2C. 16-bit to match i2c_master. Since this is constant, we use localparam
    // For 100MHz system clk, 100kHz SCL clk:
    // f_SCL = f_CLK / (4 * (divider + 1))
    // => 100kHz = 100MHz / (4 * (divider + 1))
    // => divider = 249
    
    // For 100MHz system clk, 400kHz SCL clk:
    // f_SCL = f_CLK / (4 * (divider + 1))
    // => 400kHz = 100MHz / (4 * (divider + 1))
    // => divider = 61.5 = 62       // Too fast; repeated samples
    
    logic [7:0] i2c_miso_data_from_master;
    logic i2c_busy_from_master;
    logic i2c_error_from_master;
    
    
    logic [19:0] wakeup_timer_reg; // 20-bit timer for waiting till the MPU-6050 wakes up
    
    
    // --- Internal Signals for I2C burst read ---
    // NOTE: Having separate signals for the burst instance is recommended to avoid contention.
    // For example, we don't want the same enable signal for both the instances, otherwise they both might try to seize the bus at the same time.
    logic i2c_enable_burst_to_master_reg;
    logic i2c_read_write_burst_to_master_reg;
    logic [47:0] i2c_mosi_data_burst_to_master_reg;     // Only needed for initialization; never actually used since we never do burst writes
    logic [7:0] i2c_register_address_burst_to_master_reg;
    // NOTE: The localparams remain same as the single-byte i2c_master instance
    
    logic [47:0] i2c_miso_data_burst_from_master;       // For 3-axis accelerometer readings; MSByte is ACCEL_XOUT_H, LSByte is ACCEL_ZOUT_L
    logic i2c_busy_burst_from_master;
    logic i2c_error_burst_from_master;
    
    
    // --- Internal Signals for UART ---
    logic uart_tx_en_pulse_sig;         // 1-cycle enable pulse for UART TX module
    logic [7:0] data_to_transmit_reg;   // Register to hold the data to be sent
    
    logic uart_busy_sig;                // Busy signal from UART TX module
    
    
    // --- Internal Signals for the Hamming Window ROM ---
    logic [15:0] hamming_lut_rom [1024]; // entire Hamming window
    logic [15:0] hamming_coeff_reg; // 16-bit register for storing Hamming window coefficient // unsigned
    
    logic signed [15:0] raw_accel_x_reg, raw_accel_y_reg, raw_accel_z_reg; // raw MPU-6050 accelerometer readings
    //logic signed [31:0] product_x_reg, product_y_reg, product_z_reg; // 32-bit to store 16-bit * 16-bit output
    logic signed [63:0] product_x_reg, product_y_reg, product_z_reg; // 64-bit to store 32-bit * 32-bit output
    // NOTE: these must be signed. otherwise multiplications with -ve values are not handled. The raw MPU-6050 data is signed.
    // The product should also be signed (to preserve the sign of the raw value). The hamm coeffs are unsigned since all are +ve
    
    // for handling -ve inputs:
    logic signed [31:0] mult_operand_a_x, mult_operand_a_y, mult_operand_a_z;
    logic signed [31:0] mult_operand_b;
    
    // for synchronous reset warnings:
    logic           sync_reset;                    // (wire) Unnecessary; just used for better readability
    logic           reset_sync_reg1, reset_sync_reg2;   // 1-bit registers are equivalent to FFs
    // The "srst" port of the FIFO Generator IP is an active-high synchronous reset. "Synchronous" means that the reset only happens on the
    // rising edge of the clk. We cannot directly connect to RESET_N switch since that is asynchronous, and could cause metastability in the FIFO core.
    // Thus, we update this in a separate always_ff block, synchronised with the clk. For this, we will use a two-flop synchronizer chain:
    always_ff @(posedge CLK100MHZ) begin
        // First FF samples the asynchronous switch input. This FF is allowed to go metastable.
        reset_sync_reg1 <= !rst_n;              // This FF samples asynchronous rst_n on the clk edge; !rst_n since active-high
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
        // posedge 3: FF2 samples the stable output of FF1, which is now the correctly value.
    end
    assign sync_reset = reset_sync_reg2;    // Wire holds synchronous value for !rst_n (because active-high)
    
    logic [10:0] coeff_counter_reg;  // Incremented every time a coeff is read from ROM. 11-bit, since we need to represent values till 1024
                                                // (10-bit is 0-1023)
                                                // This is esstially the same as fifo_in_write_counter_reg BUT synchronous
    // control signals ( enable and reset) for coeff_counter_reg; to decouple it from the asych current_state
    logic increment_coeff_counter;
    logic reset_coeff_counter;
    // Synchronized versions of the control signals (synch to clk)
    logic increment_coeff_counter_sync;
    logic reset_coeff_counter_sync;
    
    
    // NOTE: Both the FIFO IP cores are configured in First Word Fall Through (FWFT) mode instead of Standard mode. This reduces latency by 
    // eliminating the initial one-cycle delay on the first read. Although throughput remains the SAME in both modes after the initial read,
    // single-cycle AXIS streaming is much simpler in FWFT mode. Standard mode can only achieve single-cycle streaming using a 
    // complex pipelined look-ahead logic.
    
    // --- Internal Signals for Input FIFO Buffer ---
    logic           fifo_in_write_en_reg;       // 1-cycle enable pulse for writing into the buffer; register for sequential I2C logic
    logic [47:0]    fifo_in_write_data_reg;     // Register bus for sequential I2C logic. Only 16-bit real input values are stored to save resources; imag=0
    
    logic           fifo_in_read_en;            // 1-cycle enable pulse for reading from the buffer; wire for combinational AXIS logic
    logic [47:0]    fifo_in_read_data;
    
    logic           fifo_in_full;               // Output wire indicating FIFO is full (1025 samples); should never be HIGH as we only store 1024 samples
    logic           fifo_in_empty;              // Output wire indicating FIFO is empty
    
    // Counter for inputting exactly 1024 samples to the FIFO buffer (1025 is the actual depth):
    logic [10:0]    fifo_in_write_counter_reg;  // Incremented every time a sample is written. 11-bit, since we need to represent values till 1024
                                                // (10-bit is 0-1023)
    
    // NOTE: This buffer's input logic is sequential for slow I2C, but output logic is combinational for fast AXI4-Stream.
    // Combinational logic is needed in AXI4-Stream for reading in a single clock cycle.
    
    
    // --- Internal Signals for the FFT IP ---
    // Signals for AXI4-Stream Master logic (for inputting data to the AXI4-Stream Slave in FFT core):
    logic [95:0]    m_axis_data_tdata;
    logic           m_axis_data_tvalid;
    logic           m_axis_data_tready;
    logic           m_axis_data_tlast;
    logic [23:0] m_axis_data_tuser; // not used; only used for connecting to the port; i use the m_axis_status channel for BLK_EXP instead
    
    // Signals for AXI4-Stream Slave logic (for outputting data from the AXI4-Stream Master in FFT core):
    logic [95:0]    s_axis_data_tdata;
    logic           s_axis_data_tvalid;
    logic           s_axis_data_tready;
    logic           s_axis_data_tlast;
    
    // Signals for handling the Block FP scaling:
    logic [23:0]    s_axis_status_tdata;
    logic           s_axis_status_tvalid;
    logic           s_axis_status_tready;
    logic [4:0]     blk_exp_x_reg, blk_exp_y_reg, blk_exp_z_reg;
    
    // Counter for inputting exactly 1024 samples to the FFT core:
    logic [10:0]    flit_transfer_counter_reg;  // 11-bit, since we need to represent values till 1024
                                                // NOTE: Cannot use fifo_in_empty to assert m_axis_data_tlast, since it must be asserted BEFORE the
                                                // 1024th flit transfer, not AFTER it
    
    
    // --- Internal Signals for Output FIFO Buffer ---
    logic           fifo_out_write_en;          // 1-cycle enable pulse for writing into the buffer; wire for combinational AXIS logic
    // NOTE: Initializing a register fifo_out_write_data_reg would waste a clock cycle; we can directly connect FFT core's output for higher throughput
    
    logic           fifo_out_read_en_reg;       // 1-cycle enable pulse for reading from the buffer; register for sequential SPI logic
    logic [95:0]    fifo_out_read_data;
    
    logic           fifo_out_full;              // Output wire indicating FIFO is full (1025 samples); should never be HIGH as we only store 1024 samples
    logic           fifo_out_empty;             // Output wire indicating FIFO is empty
    
    // NOTE: Unlike the Input FIFO Buffer core, we won't need a counter for inputting exactly 1024 samples here,
    // since we can use the s_axis_data_tlast output signal from the FFT core for indicating the last sample.
    
    // NOTE: This buffer's input logic is combinational for fast AXI4-Stream, but output logic is sequential for slow SPI.
    // Combinational logic is needed in AXI4-Stream for writing in a single clock cycle.
    
    
    // --- Internal Signals for Output Handling: Block FP Scaling, Finding Magnitude Squared, Normalizing ---
    //logic signed [15:0] raw_x_imag; assign raw_x_imag = fifo_out_read_data[95:80]; //TEMP
    //logic signed [15:0] raw_x_real; assign raw_x_real = fifo_out_read_data[79:64]; //TEMP
    
    /*logic signed [15:0] true_x_real; assign true_x_real = (32'(raw_x_real) << blk_exp_x_reg) >>> 10; //TEMP*/
    // NOTE: concatenate to make this 32-bits so that the left shift doesn't erase data
    // NOTE: SIGN EXTENSION!!!! OTHERWISE GARBAGE OUTPUT AT FREQ BINS SEEN
    // >>> is arithmetic right shift operator, which preserves the sign bit
    
    /*
    logic [32:0] raw_x_magnitude_squared; // 33-bit to store 32-bit + 32-bit    // NOTE: not signed since squared???
    assign raw_x_magnitude_squared = (raw_x_real * raw_x_real) + (raw_x_imag * raw_x_imag);
    logic [47:0] true_x_magnitude_squared; // 48-bit to store raw_x_magnitude with up to ?? left shifts (i guess ?? will be more than enough)
    assign true_x_magnitude_squared = (raw_x_magnitude_squared << (blk_exp_x_reg << 1)) >> 20;
    //NOTE: normalised by N^2 (right shift of 20) since we are dealing with squares here
    // understand the 48 bit width??
    */
    
    // NOTE: WITHOUT pipelining, I get Critical Timing Warnings for true_x_magnitude_squared. The WNS is -ve.
    // Thus, I will have to pipeline it with registers.
    logic signed [15:0] raw_x_imag_reg; logic signed [15:0] raw_x_real_reg; //16-bit
    logic [31:0] raw_x_real_squared_reg, raw_x_imag_squared_reg; // 32-bit to store 16-bit * 16-bit
    logic [31:0] raw_x_real_squared_reg2, raw_x_imag_squared_reg2; // for pipelining
    logic [32:0] raw_x_magnitude_squared_reg; // 33-bit to store 32-bit + 32-bit    // NOTE: Unsigned since squares are always positive
    logic [47:0] normal_x_magnitude_squared_reg; // 48-bit to store raw_x_magnitude with up to ?? left shifts (i guess ?? will be more than enough)
    
    
    // --- Internal Signals for Normalized-Thresholds-Squared ROM ---
    logic [47:0] thresholds_lut_rom [0:1023]; // thresholds ROM; WIDTH = 48, DEPTH = 1024
    logic [9:0] thresholds_addr_reg; // 10-bit register for storing address; ADDR_WIDTH = 10
    logic [47:0] thresholds_data_out_reg; // 48-bit register for storing ROM output
    logic [47:0] thresholds_add_reg; // 48-bit register for storing ROM output + sensitivity
                                        //NOTE: This is STILL 48-bit since max(ROM) + sensitivity cannot overflow 48-bits
    
    
    // --- Internal Control Signals for SPI Transmission ---
    logic spi_go_to_master_reg; // pulse for 1 cycle to trigger SPI transaction (ONLY if the SPI-FSM is idle)
    //logic [47:0] fft_output_sample_reg; // register to hold the output to transmit
    
    
    // --- Instantiating the Debouncer module ---
    debouncer debouncer_inst (
    .clk(CLK100MHZ),            // System clock
    .switch_in(RESET_N),        // Raw, noisy switch input (active-low or active-high)
    .switch_out(rst_n)          // Clean, debounced and registered output
    );
    
    
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
        .error(i2c_error_from_master),
        
        .external_serial_data(MPU_SDA),
        .external_serial_clock(MPU_SCL)
    );
    
    
    // --- Instantiating I2C Master module for burst reads (for accelerometer readings) ---
    i2c_master #(
        .NUMBER_OF_DATA_BYTES(6)    // This parameter allows this instance to perform both burst reads and writes upto 2 bytes
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
        .error(i2c_error_burst_from_master),
        
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
    
    
    // --- Instantiating the Hamming Window ROM ---
    initial begin
        // Reading the 'hamming_lut_1024x16.mem' file generated by the Python script:
        $readmemh("hamming_lut_1024x16.mem", hamming_lut_rom);
    end
    
    
    // --- Instantiating the FIFO IP for Input Buffer ---
    fifo_buffer in_fifo_buffer_inst (
        .clk(CLK100MHZ),                    // (input wire) System clock
        .rst(!rst_n),                       // input wire) Asynchronous active-high reset; negate rst_n to handle active-high
        
        .din(fifo_in_write_data_reg),       // (input wire [47:0])
        .wr_en(fifo_in_write_en_reg),       // (input wire)
        
        .rd_en(fifo_in_read_en),            // (input wire)
        .dout(fifo_in_read_data),           // (output wire [47:0])
        
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
        .s_axis_config_tdata(16'h0400),                             // (input wire [15:0])
        .s_axis_config_tvalid(1'b0),                                // (input wire) Tied to '0' / no transfer occurs
        .s_axis_config_tready(s_axis_config_tready),                // (output wire)
        
        // AXI4-Stream Slave ports (for input):
        .s_axis_data_tdata(m_axis_data_tdata),                      // (input wire [95:0])
        .s_axis_data_tvalid(m_axis_data_tvalid),                    // (input wire)
        .s_axis_data_tready(m_axis_data_tready),                    // (output wire)
        .s_axis_data_tlast(m_axis_data_tlast),                      // (input wire)
        
        // AXI4-Stream Master ports (for output):
        .m_axis_data_tdata(s_axis_data_tdata),                      // (output wire [95:0])
        .m_axis_data_tuser(m_axis_data_tuser),                      // (output wire [23:0]) //temp: unconnected for now
        .m_axis_data_tvalid(s_axis_data_tvalid),                    // (output wire)
        .m_axis_data_tready(s_axis_data_tready),                    // (input wire)
        .m_axis_data_tlast(s_axis_data_tlast),                      // (output wire)
        
        .m_axis_status_tdata(s_axis_status_tdata),                  // (output wire [23:0])
        .m_axis_status_tvalid(s_axis_status_tvalid),                // (output wire)
        .m_axis_status_tready(s_axis_status_tready),                                // (input wire)
        
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
        .rst(!rst_n),                       // (input wire) Asynchronous active-high reset; negate rst_n to handle active-high
        
        .din(s_axis_data_tdata),            // (input wire [95:0])
        .wr_en(fifo_out_write_en),          // (input wire)
        
        .rd_en(fifo_out_read_en_reg),       // (input wire)
        .dout(fifo_out_read_data),          // (output wire [95:0])
        
        .full(fifo_out_full),               // (output wire)
        .empty(fifo_out_empty)              // (output wire)
    );
    
    
    // --- Instantiating the Thresholds ROM ---
    initial begin
        // Reading the 'avg_thresholds.mem' file generated by the Python script:
        $readmemh("avg_thresholds.mem", thresholds_lut_rom);
    end
    
    
    
    
    
    // --- Internal Signals for SPI FIFO Buffer ---
    logic           fifo_spi_write_en_reg;      // 1-cycle enable pulse for writing into the buffer; register for sequential I2C logic
    logic [47:0]    fifo_spi_write_data_reg;    // Register bus
    
    logic           fifo_spi_read_en_reg;       // 1-cycle enable pulse for reading from the buffer; wire for combinational AXIS logic
    logic [47:0]    fifo_spi_read_data;
    
    logic           fifo_spi_full;              // Output wire indicating FIFO is full (1025 samples); should never be HIGH as we only store 1024 samples
    logic           fifo_spi_empty;             // Output wire indicating FIFO is empty
    
    // Counter for inputting exactly 1024 samples to the FIFO buffer (1025 is the actual depth):
    logic [10:0]    fifo_spi_write_counter_reg;     // Incremented every time a sample is written. 11-bit, since we need to represent values till 1024
                                                    // (10-bit is 0-1023)
    
    logic           fifo_spi_write_valid_reg;   // check if the first word into the SPI FIFO buffer is actually first FFT sample
    
    
    // --- Internal Signals for SPI Master ---
    // Internal signals for MOSI:
    logic [7:0] spi_transmit_byte_to_master;                // Wire holding the current byte of the SPI transaction
    logic       spi_transmit_data_valid_to_master_reg;
    logic       spi_transmit_data_ready_from_master;
    
    // Internal signals for MISO (just for initializing; won't be using them here):
    logic [4:0] spi_receive_byte_counter_from_master;
    logic [7:0] spi_receive_byte_from_master;
    logic       spi_receive_data_valid_from_master;
    
    // Timer for a 10 ms delay between SPI transactions: (5 ms delay does not work properly bcs of logging delays)
    localparam DELAY_COUNT_MAX = 1_000_000;
    logic [$clog2(DELAY_COUNT_MAX)-1:0] delay_counter_reg;
    
    
    logic [3:0] byte_counter_reg;           // Byte counter for slicing the output bytes to transmit
    // Multiplexer logic:
    always_comb begin
        case (byte_counter_reg)
            4'd1:       spi_transmit_byte_to_master = fifo_spi_read_data[47:40]; // Byte 0 (MSB)
            4'd2:       spi_transmit_byte_to_master = fifo_spi_read_data[39:32]; // Byte 1
            4'd3:       spi_transmit_byte_to_master = fifo_spi_read_data[31:24]; // Byte 2
            4'd4:       spi_transmit_byte_to_master = fifo_spi_read_data[23:16]; // Byte 3
            4'd5:       spi_transmit_byte_to_master = fifo_spi_read_data[15:8];  // Byte 4
            4'd6:       spi_transmit_byte_to_master = fifo_spi_read_data[7:0];   // Byte 5 (LSB)
            default:    spi_transmit_byte_to_master = 8'hAB;                    // Value for padding the 6 bytes to 8 bytes for DMA on ESP32
        endcase
    end
    
    
    // --- Instantiating the FIFO IP for SPI Buffer ---
    fifo_spi fifo_spi_inst (
        .clk(CLK100MHZ),      // (input wire)
        .rst(!rst_n),      // (input wire)
        
        .din(fifo_spi_write_data_reg),    // (input wire [47:0])
        .wr_en(fifo_spi_write_en_reg),  // (input wire)
        
        .rd_en(fifo_spi_read_en_reg),  // (input wire)
        .dout(fifo_spi_read_data),    // (output wire [47:0])
        
        .full(fifo_spi_full),    // (output wire)
        .empty(fifo_spi_empty)  // (output wire)
    );
    
    
    // --- Instantiating SPI Master module ---
    // NOTE: Using these default parameters of the SPI_Master_With_Single_CS module:
    // - SPI mode = 0,
    // - SPI_SCLK frequency = 25MHz
    // - SPI_CS_N stays HIGH for a minimum of 1 system clock cycle when idle
    SPI_Master_With_Single_CS #(
        .MAX_BYTES_PER_CS(24)           // Can send up to 24 bytes in one SPI transaction (between CS being pulled LOW and going idle HIGH again)
    ) spi_master_inst (
        .i_Rst_L(rst_n),
        .i_Clk(CLK100MHZ),
        
        // The SPI bus:
        .o_SPI_CS_n(SPI_CS_N),
        .o_SPI_Clk(SPI_SCLK),
        .o_SPI_MOSI(SPI_MOSI),
        .i_SPI_MISO(SPI_MISO),
        
        // Signals for the MOSI line:
        .i_TX_Count(5'd8),  // Tying to fixed 8 bytes per transaction (1 output sample for 1 axis). Port is 5-bit because clog2(MAX_BYTES_PER_CS)
        .i_TX_Byte(spi_transmit_byte_to_master),
        .i_TX_DV(spi_transmit_data_valid_to_master_reg),
        .o_TX_Ready(spi_transmit_data_ready_from_master),
        
        // Signals for the MISO line:
        .o_RX_Count(spi_receive_byte_counter_from_master),
        .o_RX_Byte(spi_receive_byte_from_master),
        .o_RX_DV(spi_receive_data_valid_from_master)
    );
    
    
    // --- Defining the SPI-FSM states ---
    typedef enum logic [2:0] {
        S_SPI_IDLE,
        
        S_SPI_SEND_FRAME,           // Send the SPI frame one byte at-a-time
        S_SPI_DELAY,                // Delay between transactions to ensure ESP32 is ready to receive
        S_SPI_FIFO_READ_ENABLE,     // Read the NEXT frame to transmit from the SPI FIFO buffer
        S_SPI_CHECK_IF_FIFO_EMPTY,
        
        S_SPI_DONE                  // Reset all the counters used in the FSM
    } spi_state_t;
    
    spi_state_t spi_current_state, spi_next_state;
    
    // NOTE: We need a separate always_ff block to update spi_current_state. Otherwise, if we used the same always_ff block as the
    // rest of the sequential logic, the case(spi_current_state) won't know which spi_current_state to use: the one that just updated
    // vs one that was used in the previous cycle. This causes a Synthesis Error.
    always_ff @(posedge CLK100MHZ or negedge rst_n) begin
        if (!rst_n) begin
            spi_current_state <= S_SPI_IDLE;
        end else begin
            spi_current_state <= spi_next_state;
        end
    end
    
    
    // --- SPI-FSM Sequential Logic ---
    // NOTE: The SPI logic is both sequential and combinational
    always_ff @(posedge CLK100MHZ or negedge rst_n) begin
        if (!rst_n) begin
            // Reset values for all registers controlled by this FSM
            spi_transmit_data_valid_to_master_reg <= 1'b0;
            delay_counter_reg <= '0;
            byte_counter_reg <= '0;     // Initialized to 0. In the 1st iteration of S_SPI_SEND_FRAME, it becomes 1, corresponding to 1st byte
            
            fifo_spi_read_en_reg <= 1'b0;
            
        end else begin
            // Defaults:
            spi_transmit_data_valid_to_master_reg <= 1'b0;
            fifo_spi_read_en_reg <= 1'b0;
            
            case (spi_current_state)
                
                S_SPI_SEND_FRAME: begin
                    // Handshaking: Pulse enable for 1 cycle if we receive the ready to transmit signal from SPI master
                    if (spi_transmit_data_ready_from_master) begin          // Means SPI Master is ready to send a new byte
                        spi_transmit_data_valid_to_master_reg <= 1'b1;
                        byte_counter_reg <= byte_counter_reg + 1;           // Increment to the next byte for the next loop
                    end
                end
                
                S_SPI_DELAY: begin
                    delay_counter_reg <= delay_counter_reg + 1;
                end
                
                S_SPI_FIFO_READ_ENABLE: begin
                    if (!fifo_spi_empty)
                        fifo_spi_read_en_reg <= 1'b1;
                    
                    // Prepare for the next transaction:
                    byte_counter_reg <= '0;
                    delay_counter_reg <= '0;
                end
                
                
                /*S_SPI_DONE: begin
                    //
                end*/
                
            endcase
        end
    end
    
    
    // --- SPI-FSM Combinational Logic ---
    always_comb begin
        // Defaults:
        spi_next_state = spi_current_state;                 // By default, stay in spi_current_state, unless explicitly mentioned in a case
        
        case (spi_current_state)
            S_SPI_IDLE: begin
                // NOTE: Add logic to leave this state ONLY after the processed FFT data is ready and we have not yet completed 1024 transactions.
                if (spi_go_to_master_reg)
                    spi_next_state = S_SPI_SEND_FRAME;
                else
                    spi_next_state = S_SPI_IDLE;
            end
            
            S_SPI_SEND_FRAME: begin
                // Send the SPI frame one byte at-a-time
                if (byte_counter_reg >= 8)                  // Means last byte (8) was sent to SPI Master for transmitting; we exit this state
                    spi_next_state = S_SPI_DELAY;
                else
                    spi_next_state = S_SPI_SEND_FRAME;
            end
            
            S_SPI_DELAY: begin
                if (delay_counter_reg >= DELAY_COUNT_MAX)
                    spi_next_state = S_SPI_FIFO_READ_ENABLE;
                else
                    spi_next_state = S_SPI_DELAY;
            end
            
            S_SPI_FIFO_READ_ENABLE: begin
                spi_next_state = S_SPI_CHECK_IF_FIFO_EMPTY;
            end
            
            S_SPI_CHECK_IF_FIFO_EMPTY: begin // check for empty can only be done after read enable
                if (fifo_spi_empty)
                    spi_next_state = S_SPI_DONE;        // If complete FIFO buffer transmitted, SPI communication is done
                else
                    spi_next_state = S_SPI_SEND_FRAME;  // Loop back to send the next transaction
            end
            
            S_SPI_DONE: begin
                spi_next_state = S_SPI_IDLE;            // Loop back to idle
            end
            
        endcase
    end
    
    
    
    
    
    
    // --- Defining the FSM states ---
    typedef enum logic [6:0] {
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
        
        // X.
        S_WAIT_FOR_WAKEUP,  // wait till MPU-6050 wakes up. Otherwise we read 0x00 from accel registers.
        
        // 3. Reading ACCEL_XOUT_H, ACCEL_XOUT_L, ACCEL_YOUT_H, ACCEL_YOUT_L, ACCEL_ZOUT_H, ACCEL_ZOUT_L registers (burst read)
        S_I2C_ACCEL_XOUT_CONFIG,                    // Configure the inputs to the burst instance of the i2c_master module
        S_I2C_ACCEL_XOUT_WAIT_BEFORE_ENABLE,        // Ensure that the inputs are stable for 1 clock cycle
        S_I2C_ACCEL_XOUT_ENABLE,                    // Enable the burst instance of the i2c_master module
        S_I2C_ACCEL_XOUT_BUSY_HIGH_ENABLE_LOW,      // If busy is HIGH, pull enable LOW
        S_I2C_ACCEL_XOUT_WAIT_BUSY_LOW,             // Wait till i2c_master busy wire is LOW
        S_I2C_ACCEL_XOUT_READ_OUTPUT,               // Read the ACCEL_XOUT_H and ACCEL_XOUT_L registers
        
        // X. Applying the Hamming Window
        S_HAMMING_WINDOW_FETCH_COEFFICIENT,     // Fetch the Hamming window coefficient from ROM
        S_HAMMING_WINDOW_FETCH_COEFFICIENT_WAIT, // Wait one cycle till the ROM read data is valid (BRAM read takes 2 cycles????)
        S_HAMMING_WINDOW_PREPARE_MULTIPLY,              // for handling -ve input samples
        S_HAMMING_WINDOW_EXECUTE_MULTIPLY, // Multiply the readings by the coefficient
        
        // 4. Writing windowed accelerometer readings to input FIFO buffer
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
        
        // X.
        S_WAIT_BEFORE_NEXT_BURST_READ, // wait exactly 1ms
        
        // 9. Streaming the 1024-sample input frame from the input FIFO buffer to the FFT core
        S_FFT_INPUT_FRAME,                      // High-speed combinational logic for AXIS flit transfer on every clock cycle
        
        // 10. Streaming the 1024-sample output frame from the FFT core to the output FIFO buffer
        S_FFT_OUTPUT_FRAME,                     // High-speed combinational logic for AXIS flit transfer on every clock cycle
        
        // X. AXIS handshake for collecting the BLK_EXP for each channel
        S_GET_BLK_EXP,
        
        // X. Output Handling pipeline
        S_OUTPUT_READ,
        S_OUTPUT_MULTIPLY,
        S_OUTPUT_MULTIPLY_WAIT,//TEMP
        S_OUTPUT_ADD,
        S_OUTPUT_NORMALIZE,
        
        // X. threshold detection
        S_THRESHOLD_FETCH_COEFFICIENT, // fetch corresponding coefficient from from ROM .mem
        S_THRESHOLD_ADD,    // add the sensitivity to the read value from ROM (state needed for pipelining and better performance)
        S_THRESHOLD_COMPARE, // do the actual comparison
        
        // X. try SPI
        S_FIFO_SPI_CHECK_VALID_WRITE,
        S_FIFO_SPI_WRITE_ENABLE,
        S_CHECK_SPI_READY,
        
        // 11. Sending higher real byte in output FIFO buffer over UART
        S_UART_FIFO_OUT_1_LATCH,                // Latch the higher real byte to the uart_tx input data_to_transmit_reg
        S_UART_FIFO_OUT_1_PULSE_ENABLE,         // Pulse the uart_tx enable wire for 1 system clock cycle
        S_UART_FIFO_OUT_1_WAIT_BUSY_LOW,        // Wait till the uart_tx busy wire is LOW
        
        // 12. Sending lower real byte in output FIFO buffer over UART
        S_UART_FIFO_OUT_2_LATCH,                // Latch the lower real byte to the uart_tx input data_to_transmit_reg
        S_UART_FIFO_OUT_2_PULSE_ENABLE,         // Pulse the uart_tx enable wire for 1 system clock cycle
        S_UART_FIFO_OUT_2_WAIT_BUSY_LOW,        // Wait till the uart_tx busy wire is LOW
        
        S_UART_FIFO_OUT_3_LATCH,                // Latch the lower real byte to the uart_tx input data_to_transmit_reg
        S_UART_FIFO_OUT_3_PULSE_ENABLE,         // Pulse the uart_tx enable wire for 1 system clock cycle
        S_UART_FIFO_OUT_3_WAIT_BUSY_LOW,        // Wait till the uart_tx busy wire is LOW
        
        S_UART_FIFO_OUT_4_LATCH,                // Latch the lower real byte to the uart_tx input data_to_transmit_reg
        S_UART_FIFO_OUT_4_PULSE_ENABLE,         // Pulse the uart_tx enable wire for 1 system clock cycle
        S_UART_FIFO_OUT_4_WAIT_BUSY_LOW,        // Wait till the uart_tx busy wire is LOW
        
        S_UART_FIFO_OUT_5_LATCH,                // Latch the lower real byte to the uart_tx input data_to_transmit_reg
        S_UART_FIFO_OUT_5_PULSE_ENABLE,         // Pulse the uart_tx enable wire for 1 system clock cycle
        S_UART_FIFO_OUT_5_WAIT_BUSY_LOW,        // Wait till the uart_tx busy wire is LOW
        
        S_UART_FIFO_OUT_6_LATCH,                // Latch the lower real byte to the uart_tx input data_to_transmit_reg
        S_UART_FIFO_OUT_6_PULSE_ENABLE,         // Pulse the uart_tx enable wire for 1 system clock cycle
        S_UART_FIFO_OUT_6_WAIT_BUSY_LOW,        // Wait till the uart_tx busy wire is LOW
        
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
    
    
    // --- synch reset FSM ---
    
    always_ff @(posedge CLK100MHZ) begin
        if (sync_reset) begin
            increment_coeff_counter_sync <= 1'b0;
            reset_coeff_counter_sync <= 1'b0;
        end else begin
            increment_coeff_counter_sync <= increment_coeff_counter;
            reset_coeff_counter_sync <= reset_coeff_counter;
        end
    end
    
    always_ff @(posedge CLK100MHZ) begin
        // Highest priority is the synchronous reset you created
        if (sync_reset) begin
            hamming_coeff_reg <= 16'b0;
            raw_accel_x_reg <= 16'b0; raw_accel_y_reg <= 16'b0; raw_accel_z_reg <= 16'b0;
            mult_operand_a_x <= 32'b0; mult_operand_a_y <= 32'b0; mult_operand_a_z <= 32'b0;
            mult_operand_b <= 32'b0;
            product_x_reg <= 64'b0; product_y_reg <= 64'b0; product_z_reg <= 64'b0;
            coeff_counter_reg <= 11'b0;
            
            fifo_in_write_data_reg <= 48'b0; //this gets its data from a synch registers; thus, we need to make this synch as well
            
            raw_x_imag_reg <= 16'b0; raw_x_real_reg <= 16'b0;
            raw_x_imag_squared_reg <= 32'b0; raw_x_real_squared_reg <= 32'b0;
            raw_x_imag_squared_reg2 <= 32'b0; raw_x_real_squared_reg2 <= 32'b0;
            raw_x_magnitude_squared_reg <= 33'b0;
            
        end else begin
            
            // default counter logic:
            if (increment_coeff_counter_sync == 1'b1)
                coeff_counter_reg <= coeff_counter_reg + 1;
            if (reset_coeff_counter_sync == 1'b1)
                coeff_counter_reg <= 11'b0;
            
            // During normal operation, update datapath based on FSM state
            case (current_state)
                
                S_I2C_ACCEL_XOUT_READ_OUTPUT: begin
                    raw_accel_x_reg <= i2c_miso_data_burst_from_master[47:32];
                    raw_accel_y_reg <= i2c_miso_data_burst_from_master[31:16];
                    raw_accel_z_reg <= i2c_miso_data_burst_from_master[15:0];
                end
                
                
                S_HAMMING_WINDOW_FETCH_COEFFICIENT: begin
                    hamming_coeff_reg <= hamming_lut_rom[coeff_counter_reg];
                end
                
                S_HAMMING_WINDOW_PREPARE_MULTIPLY: begin
                    // Operand A: Assign the 16-bit signed sample. The tool will correctly sign-extend it to 32 bits.???
                    mult_operand_a_x <= raw_accel_x_reg;
                    mult_operand_a_y <= raw_accel_y_reg;
                    mult_operand_a_z <= raw_accel_z_reg;
                    
                    // Operand B: Zero-extend the positive 16-bit coefficient to 32 bits.
                    // This correctly converts the unsigned magnitude to a signed positive value.
                    mult_operand_b <= {16'b0, hamming_coeff_reg};
                end
                
                S_HAMMING_WINDOW_EXECUTE_MULTIPLY: begin
                    // Now we multiply the prepared 32-bit operands. This is an unambiguous signed multiplication.
                    product_x_reg <= mult_operand_a_x * mult_operand_b;//each is 64-bits. but only 32-bit is the valid output
                    product_y_reg <= mult_operand_a_y * mult_operand_b;
                    product_z_reg <= mult_operand_a_z * mult_operand_b;
                end
                
                
                S_FIFO_IN_WRITE_ENABLE: begin
                    // Scaling the 32-bit products back to 16-bit:
                    fifo_in_write_data_reg <= {product_x_reg[31:16], product_y_reg[31:16], product_z_reg[31:16]};
                    //useful info is only in [31:16] bits. the bits [63:32] are just for sign extension
                end
                
                
                
                S_OUTPUT_READ: begin
                    if (!fifo_out_empty) begin
                        // Reading Channel-2 (X-axis) output:
                        raw_x_imag_reg <= fifo_out_read_data[95:80];
                        raw_x_real_reg <= fifo_out_read_data[79:64];
                    end
                end
                
                S_OUTPUT_MULTIPLY: begin
                    // Since both operands of multiplication are signed, the result is also signed.
                    // The result is then assigned to an unsigned register, which is perfectly valid.
                    raw_x_real_squared_reg <= raw_x_real_reg * raw_x_real_reg;
                    raw_x_imag_squared_reg <= raw_x_imag_reg * raw_x_imag_reg;
                end
                
                S_OUTPUT_MULTIPLY_WAIT: begin
                    raw_x_real_squared_reg2 <= raw_x_real_squared_reg;
                    raw_x_imag_squared_reg2 <= raw_x_imag_squared_reg;
                end
                
                S_OUTPUT_ADD: begin
                    raw_x_magnitude_squared_reg <= raw_x_real_squared_reg2 + raw_x_imag_squared_reg2;
                    // even though we aren't multipying here, this still needs to be sync otherqise we get the impl warning for not being pipelined
                    // since raw_x_real_squared_reg and raw_x_imag_squared_reg are mult results
                end
                
            endcase
        end
    end
    
    
    // burst read delay logic
    logic [16:0] burst_read_timer_reg; //17-bit timer for counting exactly 1ms between consecutive burst reads for max accelerometer ODR of 1kHz
    always_ff @(posedge CLK100MHZ or negedge rst_n) begin
        if (!rst_n)
            burst_read_timer_reg <= 17'b0;
        else if (current_state == S_I2C_ACCEL_XOUT_READ_OUTPUT) // when a burst read is complete, we want to reset the timer
            burst_read_timer_reg <= 17'b0;
        else
            burst_read_timer_reg <= burst_read_timer_reg + 1;
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
            
            wakeup_timer_reg <= 20'b0;
            
            i2c_enable_burst_to_master_reg <= 1'b0;
            i2c_read_write_burst_to_master_reg <= 1'b0;
            i2c_mosi_data_burst_to_master_reg <= 48'b0;     // Always remains 0, since we never need to burst write
            i2c_register_address_burst_to_master_reg <= 8'h00;
            
            data_to_transmit_reg <= 8'h00;
            
            /*hamming_coeff_reg <= 16'b0;
            raw_accel_x_reg <= 16'b0; raw_accel_y_reg <= 16'b0; raw_accel_z_reg <= 16'b0;
            product_x_reg <= 32'b0; product_y_reg <= 32'b0; product_z_reg <= 32'b0;*/
            
            fifo_in_write_en_reg <= 1'b0;
            //fifo_in_write_data_reg <= 48'b0;
            fifo_in_write_counter_reg <= 11'b0;
            
            flit_transfer_counter_reg <= 11'b0;
            
            fifo_out_read_en_reg <= 1'b0;
            
            blk_exp_x_reg <= 5'b0; blk_exp_y_reg <= 5'b0; blk_exp_z_reg <= 5'b0;
            
            normal_x_magnitude_squared_reg <= 48'b0;
            
            thresholds_addr_reg <= 10'b0;
            thresholds_data_out_reg <= 48'b0;
            thresholds_add_reg <= 48'b0;
            MOTOR_CTRL <= 1'b0;     // when in reset state, we reset this signal
            
            fifo_spi_write_en_reg <= 1'b0;
            fifo_spi_write_data_reg <= 48'b0;
            fifo_spi_write_counter_reg <= 11'b0;
            fifo_spi_write_valid_reg <= 1'b0;
            spi_go_to_master_reg <= 1'b0;
            
        end else begin
            // Defaults:
            i2c_enable_to_master_reg <= 1'b0;
            i2c_enable_burst_to_master_reg <= 1'b0;
            // NOTE: The rest of the I2C registers are held undefined/latched right now
            
            fifo_in_write_en_reg <= 1'b0;
            // NOTE: The rest of the input FIFO core registers are held undefined/latched right now
            
            fifo_out_read_en_reg <= 1'b0;
            
            fifo_spi_write_en_reg <= 1'b0;
            spi_go_to_master_reg <= 1'b0;
            if (spi_current_state == S_SPI_DONE) begin
                fifo_spi_write_counter_reg <= 11'b0; // reset once SPI communication is done for the entire FFT frame
                // NOTE: Doing this in the SPI-FSM causes Critical Warnings since a register must be driven from ONLY ONE always_ff block.
            end
            
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
                
                
                // X.
                
                S_WAIT_FOR_WAKEUP: begin
                    wakeup_timer_reg <= wakeup_timer_reg + 1; // increment the counter
                end
                
                
                // 3. Reading ACCEL_XOUT_H and ACCEL_XOUT_L (burst read)
                
                S_I2C_ACCEL_XOUT_CONFIG: begin
                    i2c_read_write_burst_to_master_reg <= 1'b1;         // For reading (burst read 2 bytes)
                    i2c_mosi_data_burst_to_master_reg <= 48'b0;         // We want to read, not write. But we should still define this with a fixed value
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
                    //raw_accel_x_reg <= i2c_miso_data_burst_from_master[47:32];
                    //raw_accel_y_reg <= i2c_miso_data_burst_from_master[31:16];
                    //raw_accel_z_reg <= i2c_miso_data_burst_from_master[15:0];
                    //fifo_in_write_data_reg <= i2c_miso_data_burst_from_master;      // Writing to input FIFO buffer
                    
                    // For UART:
                    data_to_transmit_reg <= i2c_miso_data_burst_from_master[47:40];
                    // We read the ACCEL_XOUT_H byte right now.
                    // The ACCEL_XOUT_L still remains on the i2c_miso_data_burst_from_master line, which we read later.
                end
                
                
                // X.
                
                /*S_HAMMING_WINDOW_FETCH_COEFFICIENT: begin
                    // Fetching the coefficient from the Hamming window ROM:
                    hamming_coeff_reg <= hamming_lut_rom[fifo_in_write_counter_reg];
                    // fifo_in_write_counter_reg is SAME for counting address of coeffs in ROM
                end
                
                S_HAMMING_WINDOW_MULTIPLY: begin
                    // Performing the actual multiplication for windowing:
                    product_x_reg <= $signed(raw_accel_x_reg) * $signed(hamming_coeff_reg);
                    product_y_reg <= $signed(raw_accel_y_reg) * $signed(hamming_coeff_reg);
                    product_z_reg <= $signed(raw_accel_z_reg) * $signed(hamming_coeff_reg);
                end*/
                
                
                // 4. Writing accelerometer readings to input FIFO buffer
                
                S_FIFO_IN_WRITE_ENABLE: begin
                    // Scaling the 32-bit products back to 16-bit:
                    //fifo_in_write_data_reg <= {product_x_reg[31:16], product_y_reg[31:16], product_z_reg[31:16]};
                    
                    if (!fifo_in_full) begin
                        fifo_in_write_en_reg <= 1'b1;   // Pulse write enable (check for 1024 samples is handled combinationally)
                        
                        // Increment counter indicating a write to the FIFO buffer:
                        fifo_in_write_counter_reg <= fifo_in_write_counter_reg + 1;
                    end
                end
                
                
                // 6. Sending ACCEL_XOUT_L over UART
                
                S_UART_ACCEL_XOUT_L_LATCH: begin
                    // Latch the ACCEL_XOUT_L to uart_tx's data_to_transmit_reg (previously ACCEL_XOUT_H) for transmission
                    data_to_transmit_reg <= i2c_miso_data_burst_from_master[39:32];
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
                
                
                // X.
                
                S_GET_BLK_EXP: begin
                    if (s_axis_status_tvalid && s_axis_status_tready) begin // AXIS handshake
                        blk_exp_x_reg <= s_axis_status_tdata[20:16];
                        blk_exp_y_reg <= s_axis_status_tdata[12:8];
                        blk_exp_z_reg <= s_axis_status_tdata[4:0];
                    end
                end
                
                
                // X.
                
                // other states in sync FSM
                
                S_OUTPUT_NORMALIZE: begin
                    normal_x_magnitude_squared_reg <= (raw_x_magnitude_squared_reg << (blk_exp_x_reg << 1)) >> 20;
                end
                
                
                // X.
                
                S_THRESHOLD_FETCH_COEFFICIENT: begin
                    thresholds_data_out_reg <= thresholds_lut_rom[thresholds_addr_reg]; //read the ROM
                end
                
                S_THRESHOLD_ADD: begin
                    thresholds_add_reg <= thresholds_data_out_reg + THRESHOLD_SENSITIVITY;
                end
                
                S_THRESHOLD_COMPARE: begin
                    if (normal_x_magnitude_squared_reg > thresholds_add_reg)
                        MOTOR_CTRL <= 1'b1;  // indicates abnormality detected
                    // NOTE: ONLY change if abnormality detected. This stays for the entire frame.
                    
                    //else
                    //    MOTOR_CTRL <= 1'b0;  // indicates normal function
                    
                    thresholds_addr_reg <= thresholds_addr_reg + 1; // increment addr
                end
                
                
                // X.
                
                S_FIFO_SPI_CHECK_VALID_WRITE: begin
                    if (thresholds_addr_reg == 1 && fifo_spi_empty && fifo_spi_write_counter_reg == 0 && spi_current_state == S_SPI_IDLE)
                        // ensures loop is at first FFT sample
                        fifo_spi_write_valid_reg <= 1'b1;
                end
                
                S_FIFO_SPI_WRITE_ENABLE: begin
                    if (!fifo_spi_full && fifo_spi_write_valid_reg && fifo_spi_write_counter_reg < 1024 && spi_current_state == S_SPI_IDLE) begin
                        fifo_spi_write_data_reg <= normal_x_magnitude_squared_reg;
                        fifo_spi_write_en_reg <= 1'b1;
                        fifo_spi_write_counter_reg <= fifo_spi_write_counter_reg + 1; // increment counter indicating write
                    end
                end
                
                S_CHECK_SPI_READY: begin
                    if (spi_current_state == S_SPI_IDLE && fifo_spi_write_counter_reg >= 1024) begin
                        spi_go_to_master_reg <= 1'b1;
                    end
                end
                
                
                // 11. Sending higher real byte in output FIFO buffer over UART
                
                S_UART_FIFO_OUT_1_LATCH: begin
                    /*if (!fifo_out_empty) begin
                        // Latch the higher real byte to uart_tx's data_to_transmit_reg for transmission
                        //data_to_transmit_reg <= fifo_out_read_data[79:72];
                        //data_to_transmit_reg <= true_x_real[15:8]; //TEMP
                    end*/
                    data_to_transmit_reg <= normal_x_magnitude_squared_reg[47:40]; //TEMP
                end
                
                
                // 12. Sending lower real byte in output FIFO buffer over UART
                
                S_UART_FIFO_OUT_2_LATCH: begin
                    /*if (!fifo_out_empty) begin
                        // Latch the higher real byte to uart_tx's data_to_transmit_reg for transmission
                        data_to_transmit_reg <= normal_x_magnitude_squared_reg[39:32]; //TEMP
                    end*/
                    data_to_transmit_reg <= normal_x_magnitude_squared_reg[39:32]; //TEMP
                end
                
                S_UART_FIFO_OUT_3_LATCH: begin
                    /*if (!fifo_out_empty) begin
                        // Latch the higher real byte to uart_tx's data_to_transmit_reg for transmission
                        data_to_transmit_reg <= normal_x_magnitude_squared_reg[31:24]; //TEMP
                    end*/
                    data_to_transmit_reg <= normal_x_magnitude_squared_reg[31:24]; //TEMP
                end
                
                S_UART_FIFO_OUT_4_LATCH: begin
                    /*if (!fifo_out_empty) begin
                        // Latch the higher real byte to uart_tx's data_to_transmit_reg for transmission
                        data_to_transmit_reg <= normal_x_magnitude_squared_reg[23:16]; //TEMP
                    end*/
                    data_to_transmit_reg <= normal_x_magnitude_squared_reg[23:16]; //TEMP
                end
                
                S_UART_FIFO_OUT_5_LATCH: begin
                    /*if (!fifo_out_empty) begin
                        // Latch the higher real byte to uart_tx's data_to_transmit_reg for transmission
                        data_to_transmit_reg <= normal_x_magnitude_squared_reg[15:8]; //TEMP
                    end*/
                    data_to_transmit_reg <= normal_x_magnitude_squared_reg[15:8]; //TEMP
                end
                
                S_UART_FIFO_OUT_6_LATCH: begin
                    data_to_transmit_reg <= normal_x_magnitude_squared_reg[7:0]; //TEMP
                    
                    if (!fifo_out_empty) begin
                        // Latch the lower real byte to uart_tx's data_to_transmit_reg for transmission
                        //data_to_transmit_reg <= fifo_out_read_data[71:64];
                        //data_to_transmit_reg <= true_x_real[7:0]; //TEMP
                        
                        
                        // Assert read enable to request the next word from the output FIFO buffer:
                        fifo_out_read_en_reg <= 1'b1;   // Pulse enable for NEXT word due to FWFT mode
                    end
                end
                
                
                S_DONE: begin
                    // Resetting all the counters used in the FSM:
                    wakeup_timer_reg <= 20'b0;
                    fifo_in_write_counter_reg <= 11'b0;
                    flit_transfer_counter_reg <= 11'b0;
                    thresholds_addr_reg <= 10'b0;
                    
                    MOTOR_CTRL <= 1'b0;     // reset motor control for next frame
                    // don't reset this?? just latch previous value??
                    
                    fifo_spi_write_valid_reg <= 1'b0; // reset the flag for "first FFT sample valid"
                end
                
            endcase
        end
    end
    
    
    // --- FSM Combinational Logic ---
    // NOTE: The AXI4-Stream and UART logic is purely combinational
    always_comb begin
        // Defaults:
        next_state = current_state;     // By default, stay in current_state, unless explicitly mentioned in a case
        
        uart_tx_en_pulse_sig = 1'b0;
        
        increment_coeff_counter = 1'b0;
        reset_coeff_counter = 1'b0;
        
        // AXIS Master signals:
        m_axis_data_tdata = 96'b0;
        m_axis_data_tvalid = 1'b0;
        m_axis_data_tlast = 1'b0;
        fifo_in_read_en = 1'b0;
        
        // AXIS Slave signals:
        s_axis_data_tready = 1'b0;
        fifo_out_write_en = 1'b0;
        s_axis_status_tready = 1'b0;
        
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
                if (!i2c_busy_from_master && !i2c_error_from_master)
                    next_state = S_I2C_ACCEL_CONFIG_CONFIG;
                else if (i2c_error_from_master)
                    next_state = S_I2C_PWR_MGMT_1_CONFIG;               // Restart transaction in case of I2C protocol error
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
                if (!i2c_busy_from_master && !i2c_error_from_master)
                    next_state = S_WAIT_FOR_WAKEUP;
                else if (i2c_error_from_master)
                    next_state = S_I2C_ACCEL_CONFIG_CONFIG;             // Restart transaction in case of I2C protocol error
                else
                    next_state = S_I2C_ACCEL_CONFIG_WAIT_BUSY_LOW;
            end
            
            
            // X.
            
            S_WAIT_FOR_WAKEUP: begin
                // NOTE: 100,000 clock cycles represent 1 ms
                if (wakeup_timer_reg >= 1000000)    // 10 ms delay
                    next_state = S_I2C_ACCEL_XOUT_CONFIG;
                else
                    next_state = S_WAIT_FOR_WAKEUP;
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
                if (!i2c_error_burst_from_master) // error port is registered (holds memory). so even after busy is deasserted, I can check the error port
                    next_state = S_HAMMING_WINDOW_FETCH_COEFFICIENT;
                else
                    next_state = S_I2C_ACCEL_XOUT_CONFIG;       //repeat if error
            end
            
            
            // X.
            
            S_HAMMING_WINDOW_FETCH_COEFFICIENT: begin
                next_state = S_HAMMING_WINDOW_FETCH_COEFFICIENT_WAIT;
            end
            
            S_HAMMING_WINDOW_FETCH_COEFFICIENT_WAIT: begin
                next_state = S_HAMMING_WINDOW_PREPARE_MULTIPLY;
            end
            
            S_HAMMING_WINDOW_PREPARE_MULTIPLY: begin
                next_state = S_HAMMING_WINDOW_EXECUTE_MULTIPLY;
            end
            
            S_HAMMING_WINDOW_EXECUTE_MULTIPLY: begin
                next_state = S_FIFO_IN_WRITE_ENABLE;
            end
            
            
            // 4. Writing accelerometer readings to input FIFO buffer
            
            S_FIFO_IN_WRITE_ENABLE: begin
                if (!fifo_in_full)
                    increment_coeff_counter = 1'b1;
                
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
                        next_state = S_WAIT_BEFORE_NEXT_BURST_READ;
                end else
                    next_state = S_UART_WHO_AM_I_WAIT_BUSY_LOW;
            end
            
            
            // X.
            
            S_WAIT_BEFORE_NEXT_BURST_READ: begin
                if (burst_read_timer_reg >= 100000) // 100000 means 1 ms
                    next_state = S_I2C_ACCEL_XOUT_CONFIG;               // Loop back to burst reading accelerometer values
                else
                    next_state = S_WAIT_BEFORE_NEXT_BURST_READ;
            end
            
            
            // 9. Streaming the 1024-sample input frame from the input FIFO buffer to the FFT core
            
            S_FFT_INPUT_FRAME: begin
                if (!fifo_in_empty) begin
                    // Concatenating 16-bit real with 16-bit imag 0 (real-valued signal)
                    m_axis_data_tdata = {16'b0, fifo_in_read_data[47:32], 16'b0, fifo_in_read_data[31:16], 16'b0, fifo_in_read_data[15:0]};
                    // Channel 2 is x-axis; channel 1 is y-axis; channel 0 is z-axis
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
                    next_state = S_GET_BLK_EXP;
                else
                    next_state = S_FFT_OUTPUT_FRAME;
            end
            
            
            // X.
            
            S_GET_BLK_EXP: begin
                s_axis_status_tready = 1'b1;
                
                if (s_axis_status_tvalid && s_axis_status_tready) begin // AXIS handshake
                    next_state = S_OUTPUT_READ;
                end else
                    next_state = S_GET_BLK_EXP;
            end
            
            
            // X.
            
            S_OUTPUT_READ: begin
                next_state = S_OUTPUT_MULTIPLY;
            end
            
            S_OUTPUT_MULTIPLY: begin
                next_state = S_OUTPUT_MULTIPLY_WAIT;
            end
            
            S_OUTPUT_MULTIPLY_WAIT: begin
                next_state = S_OUTPUT_ADD;
            end
            
            S_OUTPUT_ADD: begin
                next_state = S_OUTPUT_NORMALIZE;
            end
            
            S_OUTPUT_NORMALIZE: begin
                next_state = S_THRESHOLD_FETCH_COEFFICIENT;
            end
            
            
            // X.
            
            S_THRESHOLD_FETCH_COEFFICIENT: begin
                next_state = S_THRESHOLD_ADD;
            end
            
            S_THRESHOLD_ADD: begin
                next_state = S_THRESHOLD_COMPARE;
            end
            
            S_THRESHOLD_COMPARE: begin
                next_state = S_FIFO_SPI_CHECK_VALID_WRITE;
            end
            
            
            // X.
            
            S_FIFO_SPI_CHECK_VALID_WRITE: begin
                next_state = S_FIFO_SPI_WRITE_ENABLE;
            end
            
            S_FIFO_SPI_WRITE_ENABLE: begin
                next_state = S_CHECK_SPI_READY;
            end
            
            S_CHECK_SPI_READY: begin
                next_state = S_UART_FIFO_OUT_1_LATCH;
            end
            
            
            // 11. Sending higher real byte in output FIFO buffer over UART
            
            S_UART_FIFO_OUT_1_LATCH: begin
                // Just wait 1 clock cycle to latch the higher byte to uart_tx's data_to_transmit_reg
                next_state = S_UART_FIFO_OUT_1_PULSE_ENABLE;
            end
            
            S_UART_FIFO_OUT_1_PULSE_ENABLE: begin
                uart_tx_en_pulse_sig = !uart_busy_sig && rst_n;         // HIGH only when the UART TX module is NOT busy and NOT in reset
                // uart_tx_en_pulse_sig is HIGH for 1 cycle, since the FSM states last 1 clock cycle each
                
                if (uart_tx_en_pulse_sig)
                    next_state = S_UART_FIFO_OUT_1_WAIT_BUSY_LOW;       // Only transition to the next state once enable signal is HIGH
                else
                    next_state = S_UART_FIFO_OUT_1_PULSE_ENABLE;
            end
            
            S_UART_FIFO_OUT_1_WAIT_BUSY_LOW: begin
                // Transition when UART TX module is not busy, else wait here till communication is finished
                if (!uart_busy_sig)
                    next_state = S_UART_FIFO_OUT_2_LATCH;
                else
                    next_state = S_UART_FIFO_OUT_1_WAIT_BUSY_LOW;
            end
            
            
            // 12. Sending lower real byte in output FIFO buffer over UART
            
            S_UART_FIFO_OUT_2_LATCH: begin
                // Just wait 1 clock cycle to latch the higher byte to uart_tx's data_to_transmit_reg
                next_state = S_UART_FIFO_OUT_2_PULSE_ENABLE;
            end
            
            S_UART_FIFO_OUT_2_PULSE_ENABLE: begin
                uart_tx_en_pulse_sig = !uart_busy_sig && rst_n;         // HIGH only when the UART TX module is NOT busy and NOT in reset
                // uart_tx_en_pulse_sig is HIGH for 1 cycle, since the FSM states last 1 clock cycle each
                
                if (uart_tx_en_pulse_sig)
                    next_state = S_UART_FIFO_OUT_2_WAIT_BUSY_LOW;       // Only transition to the next state once enable signal is HIGH
                else
                    next_state = S_UART_FIFO_OUT_2_PULSE_ENABLE;
            end
            
            S_UART_FIFO_OUT_2_WAIT_BUSY_LOW: begin
                // Transition when UART TX module is not busy, else wait here till communication is finished
                if (!uart_busy_sig)
                    next_state = S_UART_FIFO_OUT_3_LATCH;
                else
                    next_state = S_UART_FIFO_OUT_2_WAIT_BUSY_LOW;
            end
            
            
            S_UART_FIFO_OUT_3_LATCH: begin
                // Just wait 1 clock cycle to latch the higher byte to uart_tx's data_to_transmit_reg
                next_state = S_UART_FIFO_OUT_3_PULSE_ENABLE;
            end
            
            S_UART_FIFO_OUT_3_PULSE_ENABLE: begin
                uart_tx_en_pulse_sig = !uart_busy_sig && rst_n;         // HIGH only when the UART TX module is NOT busy and NOT in reset
                // uart_tx_en_pulse_sig is HIGH for 1 cycle, since the FSM states last 1 clock cycle each
                
                if (uart_tx_en_pulse_sig)
                    next_state = S_UART_FIFO_OUT_3_WAIT_BUSY_LOW;       // Only transition to the next state once enable signal is HIGH
                else
                    next_state = S_UART_FIFO_OUT_3_PULSE_ENABLE;
            end
            
            S_UART_FIFO_OUT_3_WAIT_BUSY_LOW: begin
                // Transition when UART TX module is not busy, else wait here till communication is finished
                if (!uart_busy_sig)
                    next_state = S_UART_FIFO_OUT_4_LATCH;
                else
                    next_state = S_UART_FIFO_OUT_3_WAIT_BUSY_LOW;
            end
            
            
            S_UART_FIFO_OUT_4_LATCH: begin
                // Just wait 1 clock cycle to latch the higher byte to uart_tx's data_to_transmit_reg
                next_state = S_UART_FIFO_OUT_4_PULSE_ENABLE;
            end
            
            S_UART_FIFO_OUT_4_PULSE_ENABLE: begin
                uart_tx_en_pulse_sig = !uart_busy_sig && rst_n;         // HIGH only when the UART TX module is NOT busy and NOT in reset
                // uart_tx_en_pulse_sig is HIGH for 1 cycle, since the FSM states last 1 clock cycle each
                
                if (uart_tx_en_pulse_sig)
                    next_state = S_UART_FIFO_OUT_4_WAIT_BUSY_LOW;       // Only transition to the next state once enable signal is HIGH
                else
                    next_state = S_UART_FIFO_OUT_4_PULSE_ENABLE;
            end
            
            S_UART_FIFO_OUT_4_WAIT_BUSY_LOW: begin
                // Transition when UART TX module is not busy, else wait here till communication is finished
                if (!uart_busy_sig)
                    next_state = S_UART_FIFO_OUT_5_LATCH;
                else
                    next_state = S_UART_FIFO_OUT_4_WAIT_BUSY_LOW;
            end
            
            
            S_UART_FIFO_OUT_5_LATCH: begin
                // Just wait 1 clock cycle to latch the higher byte to uart_tx's data_to_transmit_reg
                next_state = S_UART_FIFO_OUT_5_PULSE_ENABLE;
            end
            
            S_UART_FIFO_OUT_5_PULSE_ENABLE: begin
                uart_tx_en_pulse_sig = !uart_busy_sig && rst_n;         // HIGH only when the UART TX module is NOT busy and NOT in reset
                // uart_tx_en_pulse_sig is HIGH for 1 cycle, since the FSM states last 1 clock cycle each
                
                if (uart_tx_en_pulse_sig)
                    next_state = S_UART_FIFO_OUT_5_WAIT_BUSY_LOW;       // Only transition to the next state once enable signal is HIGH
                else
                    next_state = S_UART_FIFO_OUT_5_PULSE_ENABLE;
            end
            
            S_UART_FIFO_OUT_5_WAIT_BUSY_LOW: begin
                // Transition when UART TX module is not busy, else wait here till communication is finished
                if (!uart_busy_sig)
                    next_state = S_UART_FIFO_OUT_6_LATCH;
                else
                    next_state = S_UART_FIFO_OUT_5_WAIT_BUSY_LOW;
            end
            
            
            S_UART_FIFO_OUT_6_LATCH: begin
                // Just wait 1 clock cycle to latch the lower byte to uart_tx's data_to_transmit_reg
                next_state = S_UART_FIFO_OUT_6_PULSE_ENABLE;
            end
            
            S_UART_FIFO_OUT_6_PULSE_ENABLE: begin
                uart_tx_en_pulse_sig = !uart_busy_sig && rst_n;         // HIGH only when the UART TX module is NOT busy and NOT in reset
                // uart_tx_en_pulse_sig is HIGH for 1 cycle, since the FSM states last 1 clock cycle each
                
                if (uart_tx_en_pulse_sig)
                    next_state = S_UART_FIFO_OUT_6_WAIT_BUSY_LOW;       // Only transition to the next state once enable signal is HIGH
                else
                    next_state = S_UART_FIFO_OUT_6_PULSE_ENABLE;
            end
            
            S_UART_FIFO_OUT_6_WAIT_BUSY_LOW: begin
                // Transition when UART TX module is not busy, else wait here till communication is finished
                if (!uart_busy_sig) begin
                    if (fifo_out_empty)
                        next_state = S_DONE;
                    else
                        next_state = S_OUTPUT_READ;
                end else
                    next_state = S_UART_FIFO_OUT_6_WAIT_BUSY_LOW;
            end
            
            
            S_DONE: begin
                reset_coeff_counter = 1'b1; // for decoupling sync coeff_counter_reg from async current_state register
                
                next_state = S_I2C_ACCEL_XOUT_CONFIG;       // Loop back to burst reading accelerometer values
            end
            
        endcase
    end
    
    
endmodule
