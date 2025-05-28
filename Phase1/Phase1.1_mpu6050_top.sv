`timescale 1ns / 1ps

module mpu6050_top (
    input  wire CLK100MHZ,      // Basys 3 100MHz clock input
    input  wire CPU_RESETN,     // Active-low reset switch
    output wire UART_TXD        // Basys 3 UART TX pin (A18)
);
    
    // --- Internal Signals ---
    logic rst_n;                        // Internal active-low reset signal
    assign rst_n = CPU_RESETN;
    logic uart_tx_en_pulse_sig;         // 1-cycle enable pulse for UART TX module
    logic uart_busy_sig;                // Busy signal from UART TX module
    logic [7:0] data_to_transmit;   // Register to hold the data to be sent
    assign data_to_transmit = 8'h4B;    // ASCII 'K'
    
    
    // --- Instantiate UART TX Module ---
    uart_tx #(
        .BIT_RATE(115200),          // Baud rate
        .CLK_HZ(100_000_000),       // System clock frequency
        .PAYLOAD_BITS(8),           // Number of data bits
        .STOP_BITS(1)               // Number of stop bits
    ) uart_transmitter_inst (
        .clk(CLK100MHZ),                    // System clock
        .resetn(rst_n),                     // Active-low reset
        .uart_txd(UART_TXD),                // UART transmit data output pin
        .uart_tx_busy(uart_busy_sig),       // UART busy status output
        .uart_tx_en(uart_tx_en_pulse_sig),  // UART transmit enable (1-cycle pulse)
        .uart_tx_data(data_to_transmit)     // Data to be transmitted
    );
    
    
    // --- Defining the FSM states ---
    typedef enum logic [1:0] {
        S_IDLE,
        S_UART_PULSE_ENABLE,
        S_UART_WAIT_BUSY_LOW,
        S_DONE
    } main_state_t;
    
    main_state_t current_state, next_state;
    
    always_ff @(posedge CLK100MHZ or negedge rst_n) begin
        if (!rst_n) begin
            current_state <= S_IDLE;
        end else begin
            current_state <= next_state;
        end
    end
    
    
    // --- FSM Combinational Logic ---
    always_comb begin
        // Defaults:
        uart_tx_en_pulse_sig = 1'b0;    // When in any other state except S_UART_PULSE_ENABLE, uart_tx_en_pulse_sig is LOW by default
        
        next_state = current_state;
        
        case (current_state)
            S_IDLE: begin
                next_state = S_UART_PULSE_ENABLE;
            end
            
            S_UART_PULSE_ENABLE: begin
                uart_tx_en_pulse_sig = !uart_busy_sig && rst_n;     // HIGH only when the UART TX module is NOT busy and NOT in reset
                // uart_tx_en_pulse_sig is HIGH for 1 cycle, since the FSM states last 1 clock cycle each
                
                if (uart_tx_en_pulse_sig)
                    next_state = S_UART_WAIT_BUSY_LOW;   // Only transition to the next state once enable signal is HIGH
                else
                    next_state = S_UART_PULSE_ENABLE;
            end
            
            S_UART_WAIT_BUSY_LOW: begin
                // Transition when UART TX module is not busy, else wait here till communication is finished
                if (!uart_busy_sig)
                    next_state = S_DONE;
                else
                    next_state = S_UART_WAIT_BUSY_LOW;
            end
            
            S_DONE: begin
                next_state = S_IDLE;    // Loop back
            end
            
        endcase
    end

endmodule
