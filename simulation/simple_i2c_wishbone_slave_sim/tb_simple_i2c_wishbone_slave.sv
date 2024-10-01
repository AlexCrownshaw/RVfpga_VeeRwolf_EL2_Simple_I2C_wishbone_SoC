`include "../../src/VeeRwolf/Peripherals/simple_i2c/simple_i2c_wishbone_mem_map.vh"


module tb_simple_i2c_wishbone_slave();

    // Shared constant parameters
    localparam SLAVE_ADDRESS = 7'h2A;
    localparam CLK_DIV = 16'h00;

    // Clock and reset
    reg clk;
    reg rst;

    // Wishbone signals
    reg [5:0] adr_i;
    reg [7:0] dat_i;
    wire [7:0] dat_o;
    reg we_i;
    reg stb_i;
    reg cyc_i;
    wire ack_o;

    // I2C bus signals
    wire scl;
    wire sda;

    pullup(scl);    // Simulate weak pullup resistors
    pullup(sda);

    // I2C wishbone inteconnect device-under-test
    wire [3:0] state_debug;
    wire [2:0] bit_counter_debug;
    wire scl_out_debug;
    wire sda_out_debug; 
    wire [7:0] tx_debug;
    wire [7:0] rx_debug;
    wire [7:0] ctrl_debug;
    wire [7:0] status_debug;
    wire en_i2c_debug;
    wire mode_i2c_debug;
    wire start_i2c_debug;
    wire stop_i2c_debug;
    wire rw_i2c_debug; 
    wire [7:0] master_slave_addr_debug; 
    wire [7:0] master_reg_addr_debug;
    wire [15:0] clk_div_debug;
    wire [7:0] clk_div_lo_debug;
    wire [7:0] clk_div_hi_debug;

    simple_i2c_wishbone_slave dut (
        .clk(clk),
        .rst(rst),
        .adr_i(adr_i),
        .dat_i(dat_i),
        .dat_o(dat_o),
        .we_i(we_i),
        .stb_i(stb_i),
        .cyc_i(cyc_i),
        .ack_o(ack_o),
        .scl(scl),
        .sda(sda),
        .master_state_debug(state_debug),
        .bit_counter_debug(bit_counter_debug),
        .clk_div_debug(clk_div_debug),
        .clk_div_lo_debug(clk_div_lo_debug),
        .clk_div_hi_debug(clk_div_hi_debug),
        .tx_debug(tx_debug),
        .rx_debug(rx_debug),
        .ctrl_debug(ctrl_debug),
        .status_debug(status_debug),
        .en_i2c_debug(en_i2c_debug),
        .mode_i2c_debug(mode_i2c_debug),
        .start_i2c_debug(start_i2c_debug),
        .stop_i2c_debug(stop_i2c_debug),
        .rw_i2c_debug(rw_i2c_debug),
        .scl_out_i2c_debug(scl_out_debug),
        .sda_out_i2c_debug(sda_out_debug),
        .slave_addr_debug(master_slave_addr_debug),
        .reg_addr_debug(master_reg_addr_debug)
    );

    // Slave I2C device
    wire [3:0] slave_state_debug;
    wire [2:0] slave_bit_counter_debug; 
    wire scl_out_slave_debug;
    wire sda_out_slave_debug;
    reg [7:0] tx_slave;
    wire [7:0] rx_slave;
    reg [7:0] ctrl_slave;
    wire [7:0] status_slave;

    wire en_slave_debug;
    wire mode_slave_debug;
    wire start_slave_debug;
    wire stop_slave_debug;
    wire rw_slave_debug;
    wire [7:0] slave_slave_addr_debug;
    wire [7:0] slave_reg_addr_debug;


    simple_i2c #(
        .SLAVE_ADDRESS(SLAVE_ADDRESS)
    ) i2c_slave (
        .clk(clk),
        .rst(rst),
        .clk_div(CLK_DIV),
        .tx(tx_slave),
        .rx(rx_slave),
        .ctrl(ctrl_slave),
        .status(status_slave),
        .scl(scl),
        .sda(sda),
        .en_debug(en_slave_debug),
        .mode_debug(mode_slave_debug),
        .start_debug(start_slave_debug),
        .stop_debug(stop_slave_debug),
        .rw_debug(rw_slave_debug),
        .slave_state_debug(slave_state_debug),
        .slave_bit_counter_debug(slave_bit_counter_debug),
        .scl_out_debug(scl_out_slave_debug),
        .sda_out_debug(sda_out_slave_debug),
        .slave_addr_debug(slave_slave_addr_debug),
        .reg_addr_debug(slave_reg_addr_debug)
    );

    // I2C control register components
    reg [7:0] ctrl;
    reg en;
    reg mode;
    reg start;
    reg stop;
    reg rw;
    reg ld_slave_addr;
    reg ld_reg_addr;

    always @ (*) begin
        // Assign the control register bits
        ctrl[0] = en;              // Enable
        ctrl[1] = mode;            // Mode
        ctrl[2] = start;           // Start signal
        ctrl[3] = stop;            // Stop signal
        ctrl[4] = rw;              // Read/Write
        ctrl[5] = ld_slave_addr;   // Load slave address
        ctrl[6] = ld_reg_addr;     // Load register address
        ctrl[7] = 1'b0;            // Reserved or unused bit
    end

    // I2C status register components
    reg [7:0] status;
    wire busy;
    wire done;
    wire no_ack;

    assign busy = status[0];
    assign done = status[1];
    assign no_ack = status[2];

    // Clock generation
    always begin
        #10 clk = ~clk;  // 50 MHz clock
    end

    // Coverage
    reg [7:0] cov_slave_addr;
    reg [7:0] cov_reg_addr;
    reg [7:0] cov_data;
    reg       cov_no_ack;

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

        cp_no_ack: coverpoint cov_no_ack {
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

        // Initialise wb signals
        clk = 0;
        rst = 1;
        we_i = 0;
        stb_i = 0;
        cyc_i = 0;
        adr_i = 0;
        dat_i = 0;

        // Init ctrl register components
        en = 0;
        mode = 0;
        start = 0;
        stop = 0;
        rw = 0;
        ld_slave_addr = 0;
        ld_reg_addr = 0;
        ctrl[7] = 0;

        // Init status components
        status = 8'b0;

        // Init slave inputs
        tx_slave = 8'b0;
        ctrl_slave = 8'b0;

        // Remove reset
        #20 rst = 0;

        // Init I2C Master
        en = 1;
        mode = 1;
        wb_write(`CTRL_REG_ADDR, ctrl);

        // Init I2C slave
        ctrl_slave = 8'b1;

        // Run regression tests
        test_valid_i2c_write(SLAVE_ADDRESS, 8'h68, 8'hAA);
        test_valid_i2c_read(SLAVE_ADDRESS, 8'h68, 8'hAA);
        test_invalid_i2c_slave_address(8'h00, 8'h68, 8'hAA);

        // End simulation after some time
        #2000;
        $stop;  // Stop the simulation
    end

    task wb_write;
        input [5:0] adr;
        input [7:0] data;
        begin
            @(posedge clk);
            adr_i = adr;
            dat_i = data;
            cyc_i = 1;
            stb_i = 1;
            we_i = 1;
            @(posedge clk);
            cyc_i = 0;
            stb_i = 0;
            we_i = 0;
            $display("WB Write (%0t ns) --- Addr: %h, Data: %h, Ack: %h", $time, adr, data, ack_o);
        end
    endtask

    task wb_read;
        input [5:0] adr;
        begin
            @(posedge clk);
            adr_i = adr;
            cyc_i = 1;
            stb_i = 1;
            we_i = 0;
            @(posedge clk);
            cyc_i = 0;
            stb_i = 0;
            @(posedge clk);
            $display("WB Read (%0t ns) --- Addr: %h, Data: %h, Ack", $time, adr, dat_o, ack_o);
        end
    endtask

    task i2c_write;
        input [7:0] slave_addr;
        input [7:0] reg_addr;
        input [7:0] data;
        begin
            // Set write mode and write slave addr
            @ (posedge clk);
            rw = 0;                         // Set write mode
            ld_slave_addr = 1;              // Set load slave addr bit
            @ (posedge clk);
            wb_write(`CTRL_REG_ADDR, ctrl);
            @ (posedge clk);
            wb_write(`TX_REG_ADDR, slave_addr);

            // Write reg addr
            @ (posedge clk);
            ld_slave_addr = 0;              // Reset load slave addr bit
            ld_reg_addr = 1;                // Set load register addr bit
            @ (posedge clk);
            wb_write(`CTRL_REG_ADDR, ctrl);
            @ (posedge clk);
            wb_write(`TX_REG_ADDR, reg_addr);

            // Write data
            @ (posedge clk);
            ld_reg_addr = 0;                // Reset load register bit
            @ (posedge clk);
            wb_write(`CTRL_REG_ADDR, ctrl);
            @ (posedge clk);
            wb_write(`TX_REG_ADDR, data);

            // Set start bit
            @ (posedge clk);
            start = 1;                      // Set start bit
            @ (posedge clk);
            wb_write(`CTRL_REG_ADDR, ctrl);

            // Reset start bit once busy bit is set
            @ (posedge clk);
            wb_read(`STATUS_REG_ADDR);       // Initial status register read
            status = dat_o;
            while (!busy) begin
                @ (posedge clk);
                wb_read(`STATUS_REG_ADDR);   // continue reading status reg until busy bit is set
                status = dat_o;
            end
            start = 0;                      // Reset start bit
            @ (posedge clk);
            wb_write(`CTRL_REG_ADDR, ctrl);

            // Read status register until done flag is set
            @ (posedge clk);
            wb_read(`STATUS_REG_ADDR);       // Initial status register read
            status = dat_o;
            while (!done) begin
                @ (posedge clk);
                wb_read(`STATUS_REG_ADDR);   // continue reading status reg until done bit is set
                status = dat_o;
            end
        end
    endtask

    task i2c_read;
        input [7:0] slave_addr;
        input [7:0] reg_addr;
        begin
            // Set write mode and write slave addr
            @ (posedge clk);
            rw = 1;                         // Set read mode
            ld_slave_addr = 1;              // Set load slave addr bit
            @ (posedge clk);
            wb_write(`CTRL_REG_ADDR, ctrl);
            @ (posedge clk);
            wb_write(`TX_REG_ADDR, slave_addr);

            // Write reg addr
            @ (posedge clk);
            ld_slave_addr = 0;              // Reset load slave addr bit
            ld_reg_addr = 1;                // Set load register addr bit
            @ (posedge clk);
            wb_write(`CTRL_REG_ADDR, ctrl);
            @ (posedge clk);
            wb_write(`TX_REG_ADDR, reg_addr);

            // Set start bit
            @ (posedge clk);
            start = 1;                      // Set start bit
            @ (posedge clk);
            wb_write(`CTRL_REG_ADDR, ctrl);

            // Reset start bit once busy bit is set
            @ (posedge clk);
            wb_read(`STATUS_REG_ADDR);       // Initial status register read
            status = dat_o;
            while (!busy) begin
                @ (posedge clk);
                wb_read(`STATUS_REG_ADDR);   // continue reading status reg until busy bit is set
                status = dat_o;
            end
            start = 0;                      // Reset start bit
            @ (posedge clk);
            wb_write(`CTRL_REG_ADDR, ctrl);

            // Read status register until done flag is set
            @ (posedge clk);
            wb_read(`STATUS_REG_ADDR);       // Initial status register read
            status = dat_o;
            while (!done) begin
                @ (posedge clk);
                wb_read(`STATUS_REG_ADDR);   // continue reading status reg until done bit is set
                status = dat_o;
            end

            // Read rx register
            @ (posedge clk);
            wb_read(`RX_REG_ADDR);

            @ (posedge clk);
        end
    endtask

    // Regression Tests
    task test_valid_i2c_write;
        input [7:0] slave_addr;
        input [7:0] reg_addr;
        input [7:0] data;

        begin
            $display("STARTING TEST: Valid I2C Write to Slave Address: %h, Register: %h, Data: %h", slave_addr, reg_addr, data);
            
            // Perform the I2C write operation
            i2c_write(slave_addr, reg_addr, data);
            wait(done == 1);  // Wait for operation to complete

            // Assertions to verify the write operation
            if (slave_slave_addr_debug[7:1] !== slave_addr) begin
                $display("Test failed: Slave address mismatch! Expected: %h, Actual: %h", slave_addr, slave_slave_addr_debug[7:1]);
                $fatal;
            end
            if (slave_reg_addr_debug !== reg_addr) begin
                $display("Test failed: Register address mismatch! Expected: %h, Actual: %h", reg_addr, slave_reg_addr_debug);
                $fatal;
            end
            if (rx_slave !== data) begin
                $display("Test failed: Data mismatch in RX register! Expected: %h, Actual: %h", data, rx_slave);
                $fatal;
            end
            if (no_ack !== 0) begin
                $display("Test failed: Unexpected NACK received! Expected: 0, Actual: %h", no_ack);
                $fatal;
            end

            // Sample the coverage group with the actual values
            cov_slave_addr = slave_addr;
            cov_reg_addr = reg_addr;
            cov_data = data;
            cov_no_ack = no_ack;
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

            tx_slave = data;  // Set the data to read from the slave

            // Perform the I2C read operation
            i2c_read(slave_addr, reg_addr);
            wait(done == 1);  // Wait for operation to complete

            if (slave_slave_addr_debug[7:1] !== slave_addr) begin
                $display("Test failed: Slave address mismatch! Expected: %h, Actual: %h", slave_addr, slave_slave_addr_debug[7:1]);
                $fatal;
            end
            if (slave_reg_addr_debug !== reg_addr) begin
                $display("Test failed: Register address mismatch! Expected: %h, Actual: %h", reg_addr, slave_reg_addr_debug);
                $fatal;
            end
            if (dat_o !== data) begin
                $display("Test failed: Data mismatch in RX register! Expected: %h, Actual: %h", data, dat_o);
                $fatal;
            end
            if (no_ack !== 0) begin
                $display("Test failed: Unexpected NACK received! Expected: 0, Actual: %h", no_ack);
                $fatal;
            end

            // Sample coverage
            cov_slave_addr = slave_addr;
            cov_reg_addr = reg_addr;
            cov_data = data;
            cov_no_ack = no_ack;
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
            wait(done == 1);  // Wait for operation to complete

            // Expect a NACK (no_ack should be set)
            if (no_ack !== 1) begin
                $display("Test failed: Expected NACK not received for invalid slave address! Expected: 1, Actual: %h", no_ack);
                $fatal;
            end

            // Sample coverage
            cov_slave_addr = invalid_slave_addr;
            cov_reg_addr = reg_addr;
            cov_data = data;
            cov_no_ack = no_ack;
            i2c_cov.sample();

            $display("TEST PASSED: NACK correctly received for invalid slave address.\n");
        end
    endtask

endmodule
