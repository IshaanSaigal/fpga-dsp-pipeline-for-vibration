`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 04.06.2025 18:47:20
// Design Name: 
// Module Name: spi_top
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


module spi_top (
    input wire CLK100MHZ,       // 100MHz clock pin, Pin W5
    input wire RESET_N,         // Active-low reset switch
    
    output wire SPI_CS_N,       // PMOD JB, Pin A14
    output wire SPI_SCLK,       // PMOD JB, Pin A16
    output wire SPI_MOSI,       // PMOD JB, Pin B15
    input  wire SPI_MISO        // PMOD JB, Pin B16
);
    
    // --- Internal Signals ---
    logic rst_n;                        // Internal active-low reset signal
    assign rst_n = RESET_N;
    
    
    // --- Internal Signals for SPI Master ---
    // Internal signals for MOSI:
    logic [7:0] spi_transmit_byte_to_master_reg;                // Wire holding the current byte of the SPI transaction
    logic       spi_transmit_data_valid_to_master_reg;
    logic       spi_transmit_data_ready_from_master;
    
    // Internal signals for MISO (just for initializing; won't be using them here):
    logic [4:0] spi_receive_byte_counter_from_master;
    logic [7:0] spi_receive_byte_from_master;
    logic       spi_receive_data_valid_from_master;
    
    // Timer for a 20 ms delay between SPI transactions:
    localparam DELAY_COUNT_MAX = 2_000_000;
    logic [$clog2(DELAY_COUNT_MAX)-1:0] delay_counter_reg;
    
    
    // Assume output to transmit:
    logic [47:0]    fft_output_sample;
    assign fft_output_sample = 48'hF1020304056F;    // First and last bits are 1s to detect data loss at the edges of transaction
    
    logic [3:0] byte_counter_reg;//temp 4bits       // Byte counter for slicing the output bytes to transmit
    // Multiplexer logic:
    always_comb begin
        case (byte_counter_reg)
            4'd1:       spi_transmit_byte_to_master_reg = fft_output_sample[47:40]; // Byte 0 (MSB)
            4'd2:       spi_transmit_byte_to_master_reg = fft_output_sample[39:32]; // Byte 1
            4'd3:       spi_transmit_byte_to_master_reg = fft_output_sample[31:24]; // Byte 2
            4'd4:       spi_transmit_byte_to_master_reg = fft_output_sample[23:16]; // Byte 3
            4'd5:       spi_transmit_byte_to_master_reg = fft_output_sample[15:8];  // Byte 4
            4'd6:       spi_transmit_byte_to_master_reg = fft_output_sample[7:0];   // Byte 5 (LSB)
            default:    spi_transmit_byte_to_master_reg = 8'hAB;                    // Value for padding the 6 bytes to 8 bytes for DMA on ESP32
        endcase
    end
    
    
    // --- Instantiating SPI Master module ---
    // NOTE: Using these default parameters of the SPI_Master_With_Single_CS module:
    // - SPI mode = 0,
    // - SPI_SCLK frequency = 25MHz
    // - SPI_CS_N stays HIGH for a minimum of 1 system clock cycle when idle
    SPI_Master_With_Single_CS #(
        .MAX_BYTES_PER_CS(24)           // Can send up to 24 bytes in one SPI transaction (between CS being pulled LOW and going idle HIGH again)
    ) spi_master_inst (
        .i_Rst_L(RESET_N),
        .i_Clk(CLK100MHZ),
        
        // The SPI bus:
        .o_SPI_CS_n(SPI_CS_N),
        .o_SPI_Clk(SPI_SCLK),
        .o_SPI_MOSI(SPI_MOSI),
        .i_SPI_MISO(SPI_MISO),
        
        // Signals for the MOSI line:
        .i_TX_Count(5'd8),  // Tying to fixed 6 bytes per transaction (1 output sample for 1 axis). Port is 5-bit because clog2(MAX_BYTES_PER_CS)
        .i_TX_Byte(spi_transmit_byte_to_master_reg),
        .i_TX_DV(spi_transmit_data_valid_to_master_reg),
        .o_TX_Ready(spi_transmit_data_ready_from_master),
        
        // Signals for the MISO line:
        .o_RX_Count(spi_receive_byte_counter_from_master),
        .o_RX_Byte(spi_receive_byte_from_master),
        .o_RX_DV(spi_receive_data_valid_from_master)
    );
    
    
    // --- Defining the FSM states ---
    typedef enum logic [2:0] {
        S_IDLE,
        
        S_SPI_SEND_FRAME,           // Send the SPI frame one byte at-a-time
        S_SPI_DELAY,                // Delay between transactions to ensure ESP32 is ready to receive
        
        S_SPI_DONE                  // Reset all the counters used in the FSM
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
    // NOTE: The SPI logic is both sequential and combinational
    always_ff @(posedge CLK100MHZ or negedge rst_n) begin
        if (!rst_n) begin
            // Reset values for all registers controlled by this FSM
            spi_transmit_data_valid_to_master_reg <= 1'b0;
            delay_counter_reg <= '0;
            byte_counter_reg <= '0;     // Initialized to 0. In the 1st iteration of S_SPI_SEND_FRAME, it becomes 1, corresponding to 1st byte
            
        end else begin
            // Defaults:
            spi_transmit_data_valid_to_master_reg <= 1'b0;
            
            case (current_state)
                
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
                
                
                S_SPI_DONE: begin
                    byte_counter_reg <= '0;
                    delay_counter_reg <= '0;
                end
                
            endcase
        end
    end
    
    
    // --- FSM Combinational Logic ---
    always_comb begin
        // Defaults:
        next_state = current_state;     // By default, stay in current_state, unless explicitly mentioned in a case
        
        case (current_state)
            S_IDLE: begin
                // NOTE: Add logic to leave this state ONLY after the processed FFT data is ready and we have not yet completed 1024 transactions.
                next_state = S_SPI_SEND_FRAME;
            end
            
            S_SPI_SEND_FRAME: begin
                // Send the SPI frame one byte at-a-time
                if (byte_counter_reg >= 8)                      // Means last byte (8) was sent to SPI Master for transmitting; we exit this state
                    next_state = S_SPI_DELAY;
                else
                    next_state = S_SPI_SEND_FRAME;
            end
            
            S_SPI_DELAY: begin
                if (delay_counter_reg >= DELAY_COUNT_MAX)
                    next_state = S_SPI_DONE;
                else
                    next_state = S_SPI_DELAY;
            end
            
            S_SPI_DONE: begin
                next_state = S_IDLE;    // Loop back
            end
            
        endcase
    end
    
endmodule
