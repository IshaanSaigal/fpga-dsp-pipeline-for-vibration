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
    input wire BYTE_SWITCH,     // Switch for controlling the byte to be sent over SPI
    
    output wire SPI_CS_N,       // PMOD JB, Pin A14
    output wire SPI_SCLK,       // PMOD JB, Pin A16
    output wire SPI_MOSI,       // PMOD JB, Pin B15
    input  wire SPI_MISO        // PMOD JB, Pin B16
);
    
    // --- Internal Signals ---
    logic rst_n;                        // Internal active-low reset signal
    assign rst_n = RESET_N;
    // The above code seems redundant for the spi_master module, which has active-low reset.
    // However, in case any other module in the future requires a reset signal, which may be active-high, we need to have the
    // wire rst_n which we can alter as per our requirements (active-high vs active-low), while still using the same external switch.
    
    
    // --- Internal Signals for SPI Master ---
    // Internal signals for MOSI:
    logic       spi_transmit_byte_counter_to_master_reg;            // 1-bit only since we only want to transmit 1-byte right now
    logic [7:0] spi_transmit_byte_to_master_reg;
    logic       spi_transmit_data_valid_to_master_reg;
    logic       spi_transmit_data_ready_from_master;
    
    // Internal signals for MISO (just for initializing; won't be using them here):
    logic       spi_receive_byte_counter_from_master;
    logic [7:0] spi_receive_byte_from_master;
    logic       spi_receive_data_valid_from_master;
    
    
    // --- Instantiating SPI Master module ---
    // NOTE: Using the default parameters of the SPI_Master_With_Single_CS module:
    // - SPI mode = 0,
    // - SPI_SCLK frequency = 25MHz
    // - SPI_CS_N stays HIGH for a minimum of 1 system clock cycle when idle
    SPI_Master_With_Single_CS #(
        .MAX_BYTES_PER_CS(1)        // Can send only 1 byte in an SPI transaction (between CS being pulled LOW and going idle HIGH again)
    ) spi_master_inst (
        .i_Rst_L(RESET_N),
        .i_Clk(CLK100MHZ),
        
        // The SPI bus:
        .o_SPI_CS_n(SPI_CS_N),
        .o_SPI_Clk(SPI_SCLK),
        .o_SPI_MOSI(SPI_MOSI),
        .i_SPI_MISO(SPI_MISO),
        
        // Signals for the MOSI line:
        .i_TX_Count(spi_transmit_byte_counter_to_master_reg),
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
        
        S_SPI_CONFIG,
        S_SPI_ENABLE,
        S_SPI_WAIT_READY_HIGH,
        
        S_DONE
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
            spi_transmit_byte_counter_to_master_reg <= 1'b0;
            spi_transmit_byte_to_master_reg <= 8'h00;
            spi_transmit_data_valid_to_master_reg <= 1'b0;
            
        end else begin
            // Defaults:
            spi_transmit_byte_counter_to_master_reg <= 1'b1;
            spi_transmit_data_valid_to_master_reg <= 1'b0;
            
            case (current_state)
                
                S_SPI_CONFIG: begin
                    spi_transmit_byte_to_master_reg <= BYTE_SWITCH ? 8'h59 : 8'h4E;
                    // If the BYTE_SWITCH is HIGH, send ASCII "Y". If the BYTE_SWITCH is LOW, send ASCII "N".
                end
                
                S_SPI_ENABLE: begin
                    spi_transmit_data_valid_to_master_reg <= 1'b1;
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
                next_state = S_SPI_CONFIG;
            end
            
            S_SPI_CONFIG: begin
                if (spi_transmit_data_ready_from_master)
                    next_state = S_SPI_ENABLE;      // Only pulse enable if we receive the ready to transmit signal from master
                else
                    next_state = S_SPI_CONFIG;
            end
            
            S_SPI_ENABLE: begin
                next_state = S_SPI_WAIT_READY_HIGH;
            end
            
            S_SPI_WAIT_READY_HIGH: begin
                if (spi_transmit_data_ready_from_master)
                    next_state = S_DONE;
                else
                    next_state = S_SPI_WAIT_READY_HIGH;
            end
            
            S_DONE: begin
                next_state = S_IDLE;    // Loop back
            end
            
        endcase
    end
    
endmodule
