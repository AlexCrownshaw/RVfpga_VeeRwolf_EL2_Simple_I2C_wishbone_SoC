//  i2c_core.sv
//
// clk_div = f_in / (f_out * 2) - 1


module i2c_core #(
    parameter SLAVE_ADDRESS = 7'h50 
    ) 
    (
    input wire        clk,         
    input wire        rst,

    // Data ports
    input  reg  [7:0]  tx,            // Data to transmit
    output reg  [7:0]  rx,            // Recieved data
    inout  wire [7:0]  slave_addr,    // Master = Target slave device address / Slave = Recieved slave address
    inout  wire [7:0]  reg_addr,      // Master = Target register address / Slave = Recieved register address
    input  wire [15:0] clk_div,       // SCL clock divider

    // Control signals
    input wire        en,            // Enable bit
    input wire        mode,          // Master (1) or Slave (0)
    input wire        start,         // Start transaction
    input wire        stop,          // Stop transaction
    input wire        rw,            // Read/write bit (write = 0 / read = 1)

    // Status signals
    output reg        busy,          // Signals transaction in progress
    output reg        done,          // Signals transaction complete
    output reg        nack,          // Signals ack not recieved from slave

    // I2C signals
    inout             scl,           // I2C clock line
    inout             sda,           // I2C data line

    // Debug signals
    output wire [3:0] state_debug,
    output wire [2:0] bit_counter_debug,
    output wire       scl_out_debug,
    output wire       sda_out_debug
    );

    // Clock Divider for SCL generation (only for Master)
    reg [15:0] clk_div_counter;    // Counter for the clock divider
    reg scl_clk;  // Divided clock signal for SCL (used by master only)

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            clk_div_counter <= 16'b0;
            scl_clk <= 1'b0;
        end else begin
            if (clk_div_counter >= clk_div) begin
                clk_div_counter <= 16'b0;
                scl_clk <= ~scl_clk;
            end else begin
                clk_div_counter <= clk_div_counter + 1;
            end
        end
    end

    // I2C signals
    reg sda_out;
    reg scl_out; // Master only drives SCL

    wire sda_in;  // SDA input from the shared bus
    assign sda_in = sda;  // Read SDA state

    assign sda = (sda_out == 1'b0) ? 1'b0 : 1'bz;  // Open drain for SDA
    assign scl = (mode == 1'b1 && scl_clk == 1'b0) ? 1'b0 : 1'bz;  // Open drain for SCL, driven by master only

    // Bi-directional address signals
    reg [7:0] rx_slave_addr;
    reg [7:0] rx_reg_addr;

    assign slave_addr = (mode == 0) ? rx_slave_addr : 8'bz;
    assign reg_addr = (mode == 0) ? rx_reg_addr : 8'bz;

    // I/O reset condition
    always @ (posedge clk or posedge rst) begin
        if (rst) begin
            busy <= 0;
            done <= 0;
            nack <= 0;
            rx <= 8'b0;
            scl_out <= 1'b1;  // Release SCL
            sda_out <= 1'b1;  // Release SDA
            rx_slave_addr <= 8'b0;
            rx_reg_addr <= 8'b0;
        end
    end

    // Master FSM
    typedef enum reg [3:0] {
        IDLE,
        START_CONDITION,
        SEND_ADDRESS,
        SETUP_ACK_ADDRESS,
        ACK_ADDRESS,
        SEND_REGISTER,
        SETUP_ACK_REGISTER,   
        ACK_REGISTER,
        READ,
        WRITE,
        SETUP_STOP_CONDITION,
        STOP_CONDITION
    } i2c_state_t;

    i2c_state_t state, next_state;
    reg [2:0] bit_counter;   // Bit counter for shifting data/address

    always @(posedge scl or posedge rst) begin
        if (rst) begin
            state <= IDLE;
        end else if (en && mode) begin  // Only run when in master mode and `en == 1`
            state <= next_state;  // Transition to the next state on clock edge
        end else begin
            state <= IDLE;  // If disabled, go to IDLE state
        end
    end

    always @(*) begin
        next_state = state;  // Default is to stay in the current state
        case (state)
            IDLE: begin
                if (start) next_state = START_CONDITION;  
            end
            START_CONDITION: begin
                next_state = SEND_ADDRESS;
            end
            SEND_ADDRESS: begin
                if (bit_counter == 3'd8) next_state = SETUP_ACK_ADDRESS;
            end
            SETUP_ACK_ADDRESS: begin
                next_state = ACK_ADDRESS;
            end 
            ACK_ADDRESS: begin
                if (sda_in == 1'b0) begin  // Slave sends ACK (pulls SDA low)
                    next_state = SEND_REGISTER;
                end else begin
                    next_state = STOP_CONDITION;  // If no ACK, stop transaction
                end
            end
            SEND_REGISTER: begin
                if (bit_counter == 3'd8) next_state = SETUP_ACK_REGISTER;
            end
            SETUP_ACK_REGISTER: begin
                next_state = ACK_REGISTER;
            end
            ACK_REGISTER: begin
                if (sda_in == 1'b0) begin  // Slave sends ACK (pulls SDA low)
                    if (rw) 
                        next_state = READ;
                    else
                        next_state = WRITE;
                end else begin
                    next_state = STOP_CONDITION;  // If no ACK, stop transaction
                end
            end
            READ: begin
                if (bit_counter == 3'd7) next_state = SETUP_STOP_CONDITION;
            end
            WRITE: begin
                if (bit_counter == 3'd8) next_state = SETUP_STOP_CONDITION;
            end
            SETUP_STOP_CONDITION: begin
                next_state = STOP_CONDITION;
            end 
            STOP_CONDITION: begin
                if (scl_clk == 1'b1) next_state = IDLE;
            end
        endcase
    end

    always @(negedge scl or posedge rst) begin
        if (rst) begin
            bit_counter <= 0;
        end else begin
            case (state)
                SEND_ADDRESS: begin
                    sda_out <= slave_addr[7 - bit_counter];  // Send slave address MSB first
                    bit_counter <= bit_counter + 1;
                end
                SEND_REGISTER: begin
                    sda_out <= reg_addr[7 - bit_counter];  // Send the register address
                    bit_counter <= bit_counter + 1;
                end
                WRITE: begin
                    sda_out <= tx[7 - bit_counter];  // Transmit data
                    bit_counter <= bit_counter + 1;
                end
            endcase
        end
    end

    always @ (posedge scl) begin
        case (state)
            IDLE: begin
                busy <= 0;
                if (start) begin
                    busy <= 1;
                    done <= 0;
                    nack <= 0;
                    sda_out <= 1'b0;  // Drive SDA low to signal start
                end
            end
            SETUP_ACK_ADDRESS: begin
                sda_out <= 1;
            end
            ACK_ADDRESS: begin
                if (sda_in != 1'b0) nack <= 1;                
            end
            SETUP_ACK_REGISTER: begin
                sda_out <= 1;
            end
            ACK_REGISTER: begin
                if (sda_in != 1'b0) nack <= 1;                
            end
            READ: begin
                sda_out <= 1;
                rx[7 - bit_counter] <= sda_in;  // Receive data from SDA
                bit_counter <= bit_counter + 1;
            end
            SETUP_STOP_CONDITION: begin
                sda_out <= 1'b1;  // Release SDA to signal stop
            end
            STOP_CONDITION: begin
                sda_out <= 1'b1;  // Release SDA to signal stop
                done <= 1;
                busy <= 0;
            end
        endcase
    end

    // Slave FSM
    typedef enum reg [3:0] {
        SLAVE_IDLE,
        SLAVE_LISTEN_ADDRESS,
        SLAVE_SETUP_ACK,
        SLAVE_ACK,
        SLAVE_LISTEN_REGISTER,
        SLAVE_SETUP_ACK_REGISTER,
        SLAVE_ACK_REGISTER,
        SLAVE_READ,
        SLAVE_WRITE,
        SLAVE_SETUP_WAIT_STOP,
        SLAVE_WAIT_STOP
    } i2c_slave_state_t;

    i2c_slave_state_t slave_state, next_slave_state;
    reg [2:0] slave_bit_counter;
    wire slave_rw;
    assign slave_rw = slave_addr[0];

    always @(posedge scl or posedge rst) begin
        if (rst) begin
            slave_state <= SLAVE_IDLE;
            slave_bit_counter <= 0;
        end else if (en && !mode) begin  // Only run in slave mode
            slave_state <= next_slave_state;  // Transition to the next state on clock edge
        end else
            slave_state <= SLAVE_IDLE;
    end

    always @(*) begin
        next_slave_state = slave_state;  // Default is to stay in the current state
        case (slave_state)
            SLAVE_IDLE: begin
                if (scl == 1 && sda == 0) begin
                    next_slave_state = SLAVE_LISTEN_ADDRESS;
                end
            end
            SLAVE_LISTEN_ADDRESS: begin
                if (slave_bit_counter == 3'd7) begin
                    next_slave_state = SLAVE_SETUP_ACK;
                end
            end
            SLAVE_SETUP_ACK: begin
                next_slave_state = SLAVE_ACK;
            end
            SLAVE_ACK: begin
                if (slave_addr[7:1] == SLAVE_ADDRESS) begin  // Compare the address
                    next_slave_state = SLAVE_LISTEN_REGISTER;  // Now listen for the register address
                end else begin
                    next_slave_state = SLAVE_WAIT_STOP;
                end
            end
            SLAVE_LISTEN_REGISTER: begin
                if (slave_bit_counter == 3'd7) begin
                    next_slave_state = SLAVE_SETUP_ACK_REGISTER;  // Acknowledge the register address
                end
            end
            SLAVE_SETUP_ACK_REGISTER: begin
                next_slave_state = SLAVE_ACK_REGISTER;
            end
            SLAVE_ACK_REGISTER: begin
                if (slave_rw == 1'b1) begin
                    next_slave_state = SLAVE_READ;
                end else begin
                    next_slave_state = SLAVE_WRITE;
                end
            end
            SLAVE_READ: begin
                if (slave_bit_counter == 3'd8) begin
                    next_slave_state = SLAVE_SETUP_WAIT_STOP;
                end
            end
            SLAVE_WRITE: begin
                if (slave_bit_counter == 3'd7) begin
                    next_slave_state = SLAVE_SETUP_WAIT_STOP;
                end
            end
            SLAVE_SETUP_WAIT_STOP: begin
                next_slave_state = SLAVE_WAIT_STOP;
            end
            SLAVE_WAIT_STOP: begin
                if (scl == 1 && sda == 1) begin
                    next_slave_state = SLAVE_IDLE;
                end
            end
        endcase
    end

    always @(posedge scl or posedge rst) begin
        if (rst) begin
            slave_bit_counter <= 0;
        end else if (en && !mode) begin  // Only run in slave mode
            case (slave_state)
                SLAVE_IDLE: begin
                    // Wait for the start condition, no actions needed here
                end
                SLAVE_LISTEN_ADDRESS: begin
                    rx_slave_addr[7 - slave_bit_counter] <= sda_in;
                    slave_bit_counter <= slave_bit_counter + 1;
                end
                SLAVE_SETUP_ACK: begin
                    if (slave_addr[7:1] == SLAVE_ADDRESS) begin  // Compare the address
                        sda_out <= 1'b0;  // Send ACK (drive SDA low)
                    end
                end
                SLAVE_ACK: begin
                    sda_out <= 1; // Release sda
                end
                SLAVE_LISTEN_REGISTER: begin
                    rx_reg_addr[7 - slave_bit_counter] <= sda_in;  // Capture register address
                    slave_bit_counter <= slave_bit_counter + 1;
                end
                SLAVE_SETUP_ACK_REGISTER: begin
                    sda_out <= 1'b0;  // Send ACK for register address
                end
                SLAVE_ACK_REGISTER: begin
                    sda_out <= 1; // Release sda
                end
                SLAVE_WRITE: begin
                    rx[7 - slave_bit_counter] <= sda_in;  // Read data from master
                    slave_bit_counter <= slave_bit_counter + 1;
                end
                SLAVE_SETUP_WAIT_STOP: begin
                    sda_out <= 1; // Release sda
                end
                SLAVE_WAIT_STOP: begin
                    // Dont need to do anything here
                end
            endcase
        end
    end

    always @ (negedge scl) begin
        case (slave_state)
            SLAVE_READ: begin
                sda_out <= tx[7 - slave_bit_counter];  // Write data to master
                slave_bit_counter <= slave_bit_counter + 1;
            end
        endcase
    end

    // Assign debug signals
    assign state_debug = (mode == 1) ? state : slave_state;
    assign bit_counter_debug = (mode == 1) ? bit_counter : slave_bit_counter;
    assign scl_out_debug = scl_out;
    assign sda_out_debug = sda_out;

endmodule
