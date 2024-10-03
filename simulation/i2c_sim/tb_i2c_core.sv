module tb_i2c_core();

    // Shared constant parameters
    localparam SLAVE_ADDRESS = 7'h2A;
    localparam CLK_DIV_400KHZ = 16'h3d;  // clk_div = f_in / (f_out * 2) - 1

    // Clock and reset
    reg clk;
    reg rst;

    // I2C bus signals
    wire scl;
    wire sda;

    pullup(scl);    // Simulate weak pullup resistors
    pullup(sda);

    // Master debug signals
    wire [3:0] state_m;
    wire [2:0] bit_counter_m;
    wire       scl_out_m;
    wire       sda_out_m;

    // Slave debug signals
    wire [3:0] state_s;
    wire [2:0] bit_counter_s;
    wire       scl_out_s;
    wire       sda_out_s;

    // Master i2c_core instance
    reg  [7:0]  tx_m;
    wire [7:0]  rx_m;
    reg  [7:0]  slave_addr_m;                   // Use registers as the slave and reg inout ports 
    wire [7:0]  slave_addr_wire_m;              // for the master are driven by the testbench.
    reg  [7:0]  reg_addr_m;                     // Need to use wires to connect to inouts.
    wire [7:0]  reg_addr_wire_m;
    reg  [15:0] clk_div_m;
    reg         en_m;
    reg         mode_m;
    reg         start_m;
    reg         stop_m;
    reg         rw_m;
    wire        busy_m;
    wire        done_m;
    wire        nack_m;

    assign slave_addr_wire_m = {slave_addr_m[6:0], rw_m};    // Assign inout wires using registers
    assign reg_addr_wire_m = reg_addr_m;

    // I2C core master instance (DUT Master)
    i2c_core dut_master (
        .clk(clk),
        .rst(rst),
        .tx(tx_m),
        .rx(rx_m),
        .slave_addr(slave_addr_wire_m),
        .reg_addr(reg_addr_wire_m),
        .clk_div(clk_div_m),
        .en(en_m),
        .mode(mode_m),
        .start(start_m),
        .stop(stop_m),
        .rw(rw_m),
        .busy(busy_m),
        .done(done_m),
        .nack(nack_m),
        .scl(scl),
        .sda(sda),
        .state_debug(state_m),
        .bit_counter_debug(bit_counter_m),
        .scl_out_debug(scl_out_m),
        .sda_out_debug(sda_out_m)
    );

    // Slave i2c_core instance
    reg  [7:0]  tx_s;
    wire [7:0]  rx_s;
    wire [7:0]  slave_addr_s;    // Use wires as the slave and reg inout ports are driven by the slave
    wire [7:0]  reg_addr_s;
    reg  [15:0] clk_div_s;
    reg         en_s;
    reg         mode_s;
    reg         start_s;
    reg         stop_s;
    reg         rw_s;
    wire        busy_s;
    wire        done_s;
    wire        nack_s;

    // I2C core slave instance (DUT Slave)
    i2c_core #(
        .SLAVE_ADDRESS(SLAVE_ADDRESS)
        ) dut_slave (
        .clk(clk),
        .rst(rst),
        .tx(tx_s),
        .rx(rx_s),
        .slave_addr(slave_addr_s),
        .reg_addr(reg_addr_s),
        .clk_div(clk_div_s),
        .en(en_s),
        .mode(mode_s),
        .start(start_s),
        .stop(stop_s),
        .rw(rw_s),
        .busy(busy_s),
        .done(done_s),
        .nack(nack_s),
        .scl(scl),
        .sda(sda),
        .state_debug(state_s),
        .bit_counter_debug(bit_counter_s),
        .scl_out_debug(scl_out_s),
        .sda_out_debug(sda_out_s)
        );

    // Clock generation
    always begin
        #10 clk = ~clk;  // 50 MHz clock
    end

    // Coverage
    reg [7:0] cov_slave_addr;
    reg [7:0] cov_reg_addr;
    reg [7:0] cov_data;
    reg       cov_nack;

    covergroup i2c_coverage;
        // Coverpoints will sample values passed during the sample() call
        cp_slave_addr: coverpoint cov_slave_addr {
            bins valid_addr = {SLAVE_ADDRESS};   // Track valid slave address
            bins invalid_addr = default;         // Track any invalid address
        }

        cp_reg_addr: coverpoint cov_reg_addr {
            bins all_regs = {['h00:'hFF]};       // Track the full range of register addresses
        }

        cp_data: coverpoint cov_data {
            bins all_data = {['h00:'hFF]};       // Track all data values
        }

        cp_nack: coverpoint cov_nack {
            bins ack_received = {0};             // Track successful ACK
            bins nack_received = {1};            // Track NACKs
        }

        // Cross coverage to capture combinations of slave address, register address, and data
        cross cp_slave_addr, cp_reg_addr, cp_data;
    endgroup

    i2c_coverage i2c_cov;

    initial begin
        // Initialize the coverage group instance
        i2c_cov = new();

        clk = 0;
        rst = 1;

        // Initialise master signals
        tx_m           = 8'b0;
        slave_addr_m   = 8'h0;        
        reg_addr_m     = 8'b0;      
        clk_div_m      = 16'h0;     
        en_m           = 1'b0;      
        mode_m         = 1'b0;     
        start_m        = 1'b0;      
        stop_m         = 1'b0;     
        rw_m           = 1'b0;      

        // Initialise slave signals
        tx_s           = 8'b0;     
        clk_div_s      = 16'h0;     
        en_s           = 1'b0;      
        mode_s         = 1'b0;    
        start_s        = 1'b0;       
        stop_s         = 1'b0; 
        rw_s           = 1'b0;

        // Remove reset
        #20 rst = 0;

        // Init I2C Master
        en_m = 1;
        mode_m = 1;
        clk_div_m = CLK_DIV_400KHZ;

        // Init I2C slave
        en_s = 1;
        mode_s = 0;
        clk_div_s = CLK_DIV_400KHZ;

        // Run regression tests
        test_valid_i2c_write(SLAVE_ADDRESS, 8'h68, 8'hAA);
        test_valid_i2c_read(SLAVE_ADDRESS, 8'h69, 8'hAB);
        test_invalid_i2c_slave_address(8'h00, 8'h68, 8'hAA);
        // test_scl_freq(400);

        // End simulation after some time
        #200;
        $stop;  // Stop the simulation
    end

    task i2c_write;
        input [7:0] slave_addr;
        input [7:0] reg_addr;
        input [7:0] data;
        begin
            rw_m = 0;                    // Set write mode
            slave_addr_m = slave_addr;   // Set slave address
            reg_addr_m = reg_addr;       // Set reg address
            tx_m = data;                 // Set write data
            @ (posedge clk);
            start_m = 1;                 // Set start bit
            @ (posedge clk);
            while (!busy_m) begin        // Wait for transaction in progress
                @ (posedge clk);
            end
            start_m = 0;                   // Reset start bit
            while (!done_m) begin        // Wait for transaction complete
                @ (posedge clk);
            end                          // Data avaliable on rx_s
        end
    endtask

    task i2c_read;
        input [7:0] slave_addr;
        input [7:0] reg_addr;
        begin
            rw_m = 1;                    // Set read mode
            slave_addr_m = slave_addr;   // Set slave address
            reg_addr_m = reg_addr;       // Set reg address
            @ (posedge clk);
            start_m = 1;                 // Set start bit
            @ (posedge clk);
            while (!busy_m) begin        // Wait for transaction in progress
                @ (posedge clk);
            end
            start_m = 0;                   // Reset start bit
            while (!done_m) begin        // Wait for transaction complete
                @ (posedge clk);
            end                          // Data avaliable on rx_m                 
        end
    endtask

    // Regression Tests
    task test_scl_freq;
        input real target_freq_khz;

        time t_start;
        time t_end;
        integer edge_count;
        real meas_freq_khz;

        begin
            $display("STARTING TEST: SCL frequency check: Target_Freq: %0f kHz", target_freq_khz);
            edge_count = 0;
            @ (posedge scl);
            t_start = $time;

            while (edge_count < 10) begin
                @ (posedge scl);
                edge_count = edge_count + 1;
            end
            t_end = $time;

            meas_freq_khz = (edge_count * 1e6) / (t_end - t_start); // kHz
            if (meas_freq_khz == target_freq_khz) begin
                $display("TEST PASSED: Measured frequency: %0f kHz", meas_freq_khz);
            end else begin
                $display("TEST FAILED: Measured frequency: %0f kHz", meas_freq_khz);
                $fatal;
            end
        end
    endtask

    task test_valid_i2c_write;
        input [7:0] slave_addr;
        input [7:0] reg_addr;
        input [7:0] data;

        begin
            $display("STARTING TEST: Valid I2C Write to Slave Address: %h, Register: %h, Data: %h", slave_addr, reg_addr, data);
            
            // Perform the I2C write operation
            i2c_write(slave_addr, reg_addr, data);
            wait(done_m == 1);  // Wait for operation to complete

            // Assertions to verify the write operation
            if (slave_addr_s[7:1] !== slave_addr) begin
                $display("Test failed: Slave address mismatch! Expected: %h, Actual: %h", slave_addr, slave_addr_s[7:1]);
                $fatal;
            end
            if (reg_addr_s !== reg_addr) begin
                $display("Test failed: Register address mismatch! Expected: %h, Actual: %h", reg_addr, reg_addr_s);
                $fatal;
            end
            if (rx_s !== data) begin
                $display("Test failed: Data mismatch in slave RX register! Expected: %h, Actual: %h", data, rx_s);
                $fatal;
            end
            if (nack_m !== 0) begin
                $display("Test failed: Unexpected NACK received! Expected: 0, Actual: %h", nack_m);
                $fatal;
            end

            // Sample the coverage group with the actual values
            cov_slave_addr = slave_addr;
            cov_reg_addr = reg_addr;
            cov_data = data;
            cov_nack = nack_m;
            i2c_cov.sample();

            $display("TEST PASSED: Valid I2C Write successful.\n");
        end
    endtask

    task test_valid_i2c_read;
        input [7:0] slave_addr;
        input [7:0] reg_addr;
        input [7:0] data;

        begin
            $display("STARTING TEST: Valid I2C Read from Slave Address: %h, Register: %h", slave_addr, reg_addr);

            tx_s = data;  // Set the data to read from the slave

            // Perform the I2C read operation
            i2c_read(slave_addr, reg_addr);
            wait(done_m == 1);  // Wait for operation to complete

            if (slave_addr_s[7:1] !== slave_addr) begin
                $display("Test failed: Slave address mismatch! Expected: %h, Actual: %h", slave_addr, slave_addr_s[7:1]);
                $fatal;
            end
            if (reg_addr_s !== reg_addr) begin
                $display("Test failed: Register address mismatch! Expected: %h, Actual: %h", reg_addr, reg_addr_s);
                $fatal;
            end
            if (rx_m !== data) begin
                $display("Test failed: Data mismatch in master RX register! Expected: %h, Actual: %h", data, rx_m);
                $fatal;
            end
            if (nack_m !== 0) begin
                $display("Test failed: Unexpected NACK received! Expected: 0, Actual: %h", nack_m);
                $fatal;
            end

            // Sample coverage
            cov_slave_addr = slave_addr;
            cov_reg_addr = reg_addr;
            cov_data = data;
            cov_nack = nack_m;
            i2c_cov.sample();
            
            $display("TEST PASSED: Valid I2C Read successful.\n");
        end
    endtask

    task test_invalid_i2c_slave_address;
        input [7:0] invalid_slave_addr;
        input [7:0] reg_addr;
        input [7:0] data;

        begin
            $display("STARTING TEST: Invalid I2C Slave Address Test. Address: %h, Register: %h, Data: %h", invalid_slave_addr, reg_addr, data);

            // Perform the I2C write operation with an invalid slave address
            i2c_write(invalid_slave_addr, reg_addr, data);
            wait(done_m == 1);  // Wait for operation to complete

            // Expect a NACK (nack should be set)
            if (nack_m !== 1) begin
                $display("Test failed: Expected NACK not received for invalid slave address! Expected: 1, Actual: %h", nack_m);
                $fatal;
            end

            // Sample coverage
            cov_slave_addr = invalid_slave_addr;
            cov_reg_addr = reg_addr;
            cov_data = data;
            cov_nack = nack_m;
            i2c_cov.sample();

            $display("TEST PASSED: NACK correctly received for invalid slave address.\n");
        end
    endtask

endmodule
