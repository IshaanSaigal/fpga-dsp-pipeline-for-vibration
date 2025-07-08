`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 08.07.2025 18:48:35
// Design Name: 
// Module Name: debouncer
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


module debouncer #(
    parameter CLK_FREQ_HZ      = 100_000_000,    // System clock frequency (100MHz)
    parameter DEBOUNCE_TIME_MS = 20              // Desired debounce time in milliseconds
) (
    input  wire clk,            // System clock
    input  wire switch_in,      // Raw, noisy switch input (active-low or active-high)
    output logic switch_out     // Clean, debounced and registered output
);
    
    // Calculating number of clock cycles needed for the debounce time:
    localparam COUNTER_MAX = (DEBOUNCE_TIME_MS * CLK_FREQ_HZ) / 1000;
    
    // Calculating the required bit-width for the counter:
    localparam COUNTER_WIDTH = $clog2(COUNTER_MAX);
    
    
    // --- Internal Signals ---
    // NOTE: We need to synchronize the raw asynchronous switch input to prevent metastability.
    // If the switch changes in the setup/hold window of the register, it can lead to metastability and unreliable outcomes.
    // For this, we will use a 2-flop synchronizer:
    logic [1:0] sync_reg;                       // 2 FFs for the 2-flop synchronizer
    
    logic [COUNTER_WIDTH-1:0] counter_reg;
    
    // The output register should be initialized to the switch's initial state (idle HIGH for active-low)
    initial begin
        switch_out = 1'b0;      // Ensures the system resets every time the FPGA is programmed, regardless of RESET_N switch position
        counter_reg = '0;
        // NOTE: Here, we are using blocking assignment for a register.
        // The synthesizer ensures that the FF is initialized with this internal value at power-up.
        // Since this is a one-time async logic, not a synch logic, we can use blocking assignment. 
    end
    
    always_ff @(posedge clk) begin
        // Stage 1: Synchronize the asynchronous switch input to the clock domain
        // We use a 2-flop synchronizer to prevent metastability.
        sync_reg[0] <= switch_in;
        sync_reg[1] <= sync_reg[0];     // synch_reg[1] contains the synchronized switch input without metastability
        
        // Stage 2: Debounce logic
        // If the synchronized input (sync_reg[1]) is different from the current stable output (debounced_output_reg),
        // it means a potential change in switch state has occurred. Reset the counter.
        
        if (sync_reg[1] != switch_out) begin
            // Runs when sync input is different from current output, due to a switch flip OR subsequent debounce. We start/increment timer:
            
            if (counter_reg < COUNTER_MAX) begin
                counter_reg <= counter_reg + 1'b1;      // Increment the counter for every clock cycle the input is stable and != output
            end else begin
                // Runs when counter_reg == COUNTER_MAX, which implies counter has maxed out.
                // This means the input has been stable at this NEW value for the entire debounce period.
                // We can now safely update the output.
                switch_out  <= sync_reg[1];
                counter_reg <= 0;       // Reset counter for the next event
            end
            
        end else begin
            // Runs when sync input is same as current output, so there is no change.
            counter_reg <= 0;           // Keep the counter reset
        end
    end
    
endmodule
