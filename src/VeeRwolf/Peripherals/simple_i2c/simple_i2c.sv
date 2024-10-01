module simple_i2c #(
    parameter SLAVE_ADDRESS = 7'h50 
    ) 
    (
    input wire clk,         
    input wire rst,
    input wire [15:0] clk_div,
    input reg [7:0] tx,     // Tx Buffer or Address buffer 
    output reg [7:0] rx,    // Rx Buffer
    input wire [7:0] ctrl,
    output reg [7:0] status,
    inout scl,
    inout sda

    // Debug ports
    ,output wire en_debug,
    output wire mode_debug,
    output wire start_debug,
    output wire stop_debug,
    output wire rw_debug,
    output wire slave_rw_debug,
    output wire ld_slave_addr_debug,
    output wire ld_reg_addr_debug,
    output wire [3:0] master_state_debug,
    output wire [3:0] slave_state_debug,
    output wire scl_clk_debug,
    output wire [2:0] bit_counter_debug,
    output wire [2:0] slave_bit_counter_debug,
    output wire scl_out_debug,
    output wire sda_out_debug,
    output wire [7:0] slave_addr_debug,
    output wire [7:0] reg_addr_debug
    );

    // Control register signals
    wire en;        // Enable bit
    wire mode;      // Master (1) or Slave (0)
    wire start;     // Start transaction
    wire stop;      // Stop transaction
    wire rw;        // Read/write bit (write = 0 / read = 1)
    wire ld_slave_addr;   // Load tx -> addr when set
    wire ld_reg_addr;

    assign en = ctrl[0];
    assign mode = ctrl[1];
    assign start = ctrl[2];
    assign stop = ctrl[3];
    assign rw = ctrl[4];
    assign ld_slave_addr = ctrl[5];
    assign ld_reg_addr = ctrl[6];

    // Status register
    reg busy;       // Set during transaction FSM
    reg done;       // Transaction complete
    reg no_ack;     // No acknowledge recieved

    always @ (posedge clk) begin
        if (rst) begin
            busy <= 1'b0;
            done <= 1'b0;
            no_ack <= 1'b0;
        end
    end

    assign status[0] = busy;
    assign status[1] = done;
    assign status[2] = no_ack;
    assign status[7:3] = 5'b0;

    // Slave Address register
    reg [7:0] slave_addr;
    always @ (posedge clk) begin
        if (rst) begin
            slave_addr <= 8'b0;
        end else begin
            if (ld_slave_addr) begin
                slave_addr <= {tx[6:0], rw}; // Set read/write bit at the end of the address
            end
        end
    end
    
    // Register address register
    reg [7:0] reg_addr;
    always @ (posedge clk) begin
        if (rst) begin
            reg_addr <= 8'b0;
        end else begin
            if (ld_reg_addr) begin
                reg_addr <= tx;
            end
        end
    end

    // Slave read/write
    wire slave_rw;
    assign slave_rw = slave_addr[0];

    // I2C signals
    reg sda_out;
    reg scl_out; // Master only drives SCL

    wire sda_in;  // SDA input from the shared bus
    assign sda_in = sda;  // Read SDA state

    // Clock Divider for SCL generation (only for Master)
    reg [15:0] clk_div_counter;    // Counter for the clock divider
    reg scl_clk;  // Divided clock signal for SCL (used by master only)

    always @ (posedge clk or posedge rst) begin
        if (rst) begin
            clk_div_counter <= 16'b0;
            scl_clk <= 1'b0;
        end else begin
            if (clk_div_counter == clk_div) begin
                scl_clk <= ~scl_clk;  // Toggle SCL clock
                clk_div_counter <= 16'b0;
            end else begin
                clk_div_counter <= clk_div_counter + 1;
            end
        end
    end

    assign sda = (sda_out == 1'b0) ? 1'b0 : 1'bz;  // Open drain for SDA
    assign scl = (mode == 1'b1 && scl_clk == 1'b0) ? 1'b0 : 1'bz;  // Open drain for SCL, driven by master only

     // Master FSM (Master only drives SCL)
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
            busy <= 0;
            bit_counter <= 0;
            scl_out <= 1'b1;  // Release SCL
            sda_out <= 1'b1;  // Release SDA
            rx <= 8'b0;
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
        if (rst) begin
        end else begin
            case (state)
                IDLE: begin
                    busy <= 0;
                    if (start) begin
                        busy <= 1;
                        done <= 0;
                        no_ack <= 0;
                        sda_out <= 1'b0;  // Drive SDA low to signal start
                    end
                end
                SETUP_ACK_ADDRESS: begin
                    sda_out <= 1;
                end
                ACK_ADDRESS: begin
                    if (sda_in != 1'b0) no_ack <= 1;                
                end
                SETUP_ACK_REGISTER: begin
                    sda_out <= 1;
                end
                ACK_REGISTER: begin
                    if (sda_in != 1'b0) no_ack <= 1;                
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
                end
            endcase
        end
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

    always @(posedge scl or posedge rst) begin
        if (rst) begin
            slave_state <= SLAVE_IDLE;
            slave_bit_counter <= 0;
            sda_out <= 1'b1; // Release SDA in slave mode
            rx <= 8'b0;
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
            sda_out <= 1'b1;  // Release SDA in slave mode
        end else if (en && !mode) begin  // Only run in slave mode
            case (slave_state)
                SLAVE_IDLE: begin
                    // Wait for the start condition, no actions needed here
                end
                SLAVE_LISTEN_ADDRESS: begin
                    slave_addr[7 - slave_bit_counter] <= sda_in;
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
                    reg_addr[7 - slave_bit_counter] <= sda_in;  // Capture register address
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
                    // if (scl == 1 && sda == 1) begin
                    //     sda_out <= 1'b1;  // Release SDA after stop condition
                    //     slave_bit_counter <= 0;
                    // end
                end
            endcase
        end
    end

    always @ (negedge scl or posedge rst) begin
        if (rst) begin
            sda_out <= 1'b1;  // Ensure SDA is released after reset
            slave_bit_counter <= 0;  // Reset the bit counter
        end else begin
            case (slave_state)
                SLAVE_READ: begin
                    sda_out <= tx[7 - slave_bit_counter];  // Write data to master
                    slave_bit_counter <= slave_bit_counter + 1;
                end
            endcase
        end
    end

    // Debug signal assignments
    assign en_debug = en;
    assign mode_debug = mode;
    assign start_debug = start;
    assign stop_debug = stop;
    assign rw_debug = rw;
    assign slave_rw_debug = slave_rw;
    assign ld_slave_addr_debug = ld_slave_addr;
    assign ld_reg_addr_debug = ld_reg_addr;
    assign master_state_debug = state;
    assign slave_state_debug = slave_state;
    assign scl_clk_debug = scl_clk;
    assign bit_counter_debug = bit_counter;
    assign slave_bit_counter_debug = slave_bit_counter;
    assign scl_out_debug = scl_out;
    assign sda_out_debug = sda_out;
    assign slave_addr_debug = slave_addr;
    assign reg_addr_debug = reg_addr;

endmodule
