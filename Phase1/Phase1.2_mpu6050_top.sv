`timescale 1ns / 1ps

module mpu6050_top (
    input wire CLK100MHZ,       // 100MHz clock pin, Pin W5
    input wire CPU_RESETN,      // Active-low reset switch, Pin R2
    
    inout wire MPU_SCL,         // PMOD JB, Pin A14
    inout wire MPU_SDA,         // PMOD JB, Pin A16
    // NOTE: Since both SDA and SCL (clock stretching) can be controlled by both i2c_master and slave (MPU6050),
    // these must be modelled as inout ports.
    
    output wire UART_TXD        // UART Transmit Data line, Pin A18
);
    
    // --- Internal Signals ---
    logic rst_n;                        // Internal active-low reset signal
    assign rst_n = CPU_RESETN;
    // The above code seems redundant for the i2c_master and uart_tx modules, which both have active-low resets.
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
    
    
    // --- Internal Signals for UART ---
    logic uart_tx_en_pulse_sig;         // 1-cycle enable pulse for UART TX module
    logic [7:0] data_to_transmit_reg;   // Register to hold the data to be sent
    
    logic uart_busy_sig;                // Busy signal from UART TX module
    
    
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
    
    
    // --- Instantiating UART TX Module ---
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
        .uart_tx_data(data_to_transmit_reg) // Data to be transmitted
    );
    
    
    // --- Defining the FSM states ---
    typedef enum logic [3:0] {
        S_IDLE,
        
        S_I2C_CONFIG,               // Configure the inputs to the i2c_master module
        S_I2C_WAIT_BEFORE_ENABLE,   // Ensure that the inputs are stable for 1 clock cycle
        S_I2C_ENABLE,               // Enable the i2c_master module
        S_I2C_BUSY_HIGH_ENABLE_LOW, // If busy is HIGH, pull enable LOW
        S_I2C_WAIT_BUSY_LOW,        // Wait till i2c_master busy wire is LOW
        S_I2C_READ_OUTPUT,          // Read the WHOAMI register's value
        
        S_UART_PULSE_ENABLE,        // Pulse the uart_tx enable wire for 1 system clock cycle
        S_UART_WAIT_BUSY_LOW,       // Wait till the uart_tx busy wire is LOW
        
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
    // NOTE: The I2C logic is both sequential and combinational
    always_ff @(posedge CLK100MHZ or negedge rst_n) begin
        if (!rst_n) begin
            // Reset values for all outputs controlled by this FSM
            i2c_enable_to_master_reg <= 1'b0;
            i2c_read_write_to_master_reg <= 1'b0;
            i2c_mosi_data_to_master_reg <= 8'h00;
            i2c_register_address_to_master_reg <= 8'h00;
            
            data_to_transmit_reg <= 8'h00;
            
        end else begin
            // Defaults:
            i2c_enable_to_master_reg <= 1'b0;
            // The rest of the I2C registers are held undefined/latched right now
            
            case (current_state)
                S_I2C_CONFIG: begin
                    i2c_read_write_to_master_reg <= 1'b1;       // For reading
                    i2c_mosi_data_to_master_reg <= 8'h00;       // We want to read, not write. But we should still define this with a fixed value
                    i2c_register_address_to_master_reg <= 8'h75;     // Address of the WHOAMI register
                end
                
                S_I2C_ENABLE: begin
                    i2c_enable_to_master_reg <= 1'b1;
                end
                
                S_I2C_BUSY_HIGH_ENABLE_LOW: begin
                    if (!i2c_busy_from_master)
                        i2c_enable_to_master_reg <= 1'b1;   // Keep enable high if busy is low
                    else
                        i2c_enable_to_master_reg <= 1'b0;   // Keep enable low if busy is high
                end
                
                S_I2C_READ_OUTPUT: begin
                    // Actually read the i2c_master master
                    data_to_transmit_reg <= i2c_miso_data_from_master;      // 8'h68 (ASCII 'h') is the ideal return from WHOAMI
                end
                
            endcase
        end
    end
    
    
    // --- FSM Combinational Logic ---
    // NOTE: The UART logic is fully combinational
    always_comb begin
        // Defaults:
        uart_tx_en_pulse_sig = 1'b0;    // When in any other state except S_UART_PULSE_ENABLE, uart_tx_en_pulse_sig is LOW by default
        
        next_state = current_state;     // By default, stay in current_state, unless explicitly mentioned in a case
        
        case (current_state)
            S_IDLE: begin
                next_state = S_I2C_CONFIG;
            end
            
            S_I2C_CONFIG: begin
                next_state = S_I2C_WAIT_BEFORE_ENABLE;
            end
            
            S_I2C_WAIT_BEFORE_ENABLE: begin
            // Wait 1 clock cycle for the configuration values to stabilize
                if (!i2c_busy_from_master) begin
                    next_state = S_I2C_ENABLE; // If I2C core is idle, proceed to enable it
                end else begin
                    next_state = S_I2C_WAIT_BEFORE_ENABLE; // Else, keep waiting
                end
            end
            
            S_I2C_ENABLE: begin
                next_state = S_I2C_BUSY_HIGH_ENABLE_LOW;
            end
            
            S_I2C_BUSY_HIGH_ENABLE_LOW: begin
                // Wait till busy signal is HIGH to set enable signal LOW
                if (!i2c_enable_to_master_reg && i2c_busy_from_master)
                    next_state = S_I2C_WAIT_BUSY_LOW;
                else
                    next_state = S_I2C_BUSY_HIGH_ENABLE_LOW;
            end
            
            S_I2C_WAIT_BUSY_LOW: begin
                // Wait till busy is LOW, which means data is ready to read
                if (!i2c_busy_from_master)
                    next_state = S_I2C_READ_OUTPUT;
                else
                    next_state = S_I2C_WAIT_BUSY_LOW;
            end
            
            S_I2C_READ_OUTPUT: begin
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
