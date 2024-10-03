`include "../../src/VeeRwolf/Peripherals/i2c/i2c_wishbone.sv"


module tb_i2c_wishbone();

    // Shared constant parameters
    localparam SLAVE_ADDRESS = 7'h2A;
    localparam CLK_DIV_400KHZ = 16'h3d;  // (400kHz)

    // Clock and reset
    reg clk;
    reg rst;

    // Wishbone signals
    reg  [5:0] adr_i;
    reg  [7:0] dat_i;
    wire [7:0] dat_o;
    reg        we_i;
    reg        stb_i;
    reg        cyc_i;
    wire       ack_o;

    // I2C bus signals
    wire scl;
    wire sda;

    pullup(scl);    // Simulate weak pullup resistors
    pullup(sda);

    // Master debug signals
    wire [3:0] master_state;
    wire [2:0] master_bit_counter;

    // I2C wishbone instance (DUT)
    i2c_wishbone dut (
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
        .state_debug(master_state),
        .bit_counter_debug(master_bit_counter)
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
    wire nack;

    assign busy = status[0];
    assign done = status[1];
    assign nack = status[2];

    // Slave I2C device
    reg  [7:0]  tx_slave;
    reg  [7:0]  rx_slave;
    wire [7:0]  rx_slave_addr;  // Slave address recived by the slave from the master
    wire [7:0]  rx_reg_addr;    // Register address recived by the slave from the master
    reg  [15:0] clk_div_slave;        
    reg         en_slave;
    reg         mode_slave;

    // Slave debug signals
    wire [3:0] slave_state;
    wire [2:0] slave_bit_counter;
    wire slave_scl_out;
    wire slave_sda_out;

    i2c_core #(
        .SLAVE_ADDRESS(SLAVE_ADDRESS)
    ) slave_inst (
        .clk(clk),
        .rst(rst),
        .tx(tx_slave),
        .rx(rx_slave),
        .slave_addr(rx_slave_addr),
        .reg_addr(rx_reg_addr),
        .clk_div(clk_div_slave),
        .en(en_slave),
        .mode(mode_slave),
        .scl(scl),
        .sda(sda),
        .state_debug(slave_state),
        .bit_counter_debug(slave_bit_counter),
        .scl_out_debug(slave_scl_out),
        .sda_out_debug(slave_sda_out)
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
        en_slave = 0;
        mode_slave = 0;

        // Remove reset
        #20 rst = 0;

        // Init I2C Master
        en = 1;
        mode = 1;
        wb_write(`CTRL_REG_ADDR, ctrl);
        wb_write(`CLK_DIV_LO_REG_ADDR, CLK_DIV_400KHZ[7:0]);
        wb_write(`CLK_DIV_HI_REG_ADDR, CLK_DIV_400KHZ[15:8]);

        // Init I2C slave
        en_slave = 1;
        mode_slave = 0;
        clk_div_slave = CLK_DIV_400KHZ;

        // Run regression tests
        test_valid_i2c_write(SLAVE_ADDRESS, 8'h68, 8'hAA);
        test_valid_i2c_read(SLAVE_ADDRESS, 8'h68, 8'hAA);
        test_invalid_i2c_slave_address(8'h00, 8'h68, 8'hAA);

        // End simulation after some time
        #200;
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
            wait (ack_o)
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
            if (rx_slave_addr[7:1] !== slave_addr) begin
                $display("Test failed: Slave address mismatch! Expected: %h, Actual: %h", slave_addr, rx_slave[7:1]);
                $fatal;
            end
            if (rx_reg_addr !== reg_addr) begin
                $display("Test failed: Register address mismatch! Expected: %h, Actual: %h", reg_addr, rx_reg_addr);
                $fatal;
            end
            if (rx_slave !== data) begin
                $display("Test failed: Data mismatch in RX register! Expected: %h, Actual: %h", data, rx_slave);
                $fatal;
            end
            if (nack !== 0) begin
                $display("Test failed: Unexpected NACK received! Expected: 0, Actual: %h", nack);
                $fatal;
            end

            // Sample the coverage group with the actual values
            cov_slave_addr = slave_addr;
            cov_reg_addr = reg_addr;
            cov_data = data;
            cov_nack = nack;
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

            if (rx_slave_addr[7:1] !== slave_addr) begin
                $display("Test failed: Slave address mismatch! Expected: %h, Actual: %h", slave_addr, rx_slave_addr[7:1]);
                $fatal;
            end
            if (rx_reg_addr !== reg_addr) begin
                $display("Test failed: Register address mismatch! Expected: %h, Actual: %h", reg_addr, rx_reg_addr);
                $fatal;
            end
            if (dat_o !== data) begin
                $display("Test failed: Data mismatch in RX register! Expected: %h, Actual: %h", data, dat_o);
                $fatal;
            end
            if (nack !== 0) begin
                $display("Test failed: Unexpected NACK received! Expected: 0, Actual: %h", nack);
                $fatal;
            end

            // Sample coverage
            cov_slave_addr = slave_addr;
            cov_reg_addr = reg_addr;
            cov_data = data;
            cov_nack = nack;
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

            // Expect a NACK (nack should be set)
            if (nack !== 1) begin
                $display("Test failed: Expected NACK not received for invalid slave address! Expected: 1, Actual: %h", nack);
                $fatal;
            end

            // Sample coverage
            cov_slave_addr = invalid_slave_addr;
            cov_reg_addr = reg_addr;
            cov_data = data;
            cov_nack = nack;
            i2c_cov.sample();

            $display("TEST PASSED: NACK correctly received for invalid slave address.\n");
        end
    endtask

endmodule
