module tb_simple_i2c;

    localparam SLAVE_ADDRESS = 7'h2A;

    reg clk;
    reg rst;
    reg [15:0] clk_div;
    reg [7:0] tx_master;
    reg [7:0] tx_slave;
    reg [7:0] ctrl_master;
    reg [7:0] ctrl_slave;
    wire [7:0] rx_master;
    wire [7:0] rx_slave;
    wire scl;
    wire sda;

    // Debug signals for master
    wire en_master_debug;
    wire mode_master_debug;
    wire start_master_debug;
    wire stop_master_debug;
    wire rw_master_debug;
    wire ld_slave_addr_master_debug;
    wire ld_reg_addr_master_debug;
    wire [3:0] master_state_debug;
    wire scl_clk_master_debug;
    wire [2:0] bit_counter_master_debug;
    wire scl_out_master_debug;
    wire sda_out_master_debug;
    wire [7:0] slave_addr_master_debug;
    wire [7:0] reg_addr_master_debug;

    // Debug signals for slave
    wire en_slave_debug;
    wire mode_slave_debug;
    wire start_slave_debug;
    wire stop_slave_debug;
    wire slave_rw_debug;
    wire ld_slave_addr_slave_debug;
    wire ld_reg_addr_slave_debug;
    wire [3:0] slave_state_debug;
    wire scl_clk_slave_debug;
    wire [2:0] bit_counter_slave_debug;
    wire scl_out_slave_debug;
    wire sda_out_slave_debug;
    wire [7:0] slave_addr_slave_debug;
    wire [7:0] reg_addr_slave_debug;
    
    wire [6:0] slave_check_addr;
    assign slave_check_addr = slave_addr_slave_debug[7:1];

    pullup(scl);
    pullup(sda);

    // Instantiate the I2C Master
    simple_i2c #(.SLAVE_ADDRESS(SLAVE_ADDRESS)) master_i2c (
        .clk(clk),
        .rst(rst),
        .clk_div(clk_div),
        .tx(tx_master),
        .rx(rx_master),
        .ctrl(ctrl_master),
        .status(),  // You can monitor status if needed
        .scl(scl),
        .sda(sda),
        
        // Debug signals for master
        .en_debug(en_master_debug),
        .mode_debug(mode_master_debug),
        .start_debug(start_master_debug),
        .stop_debug(stop_master_debug),
        .rw_debug(rw_master_debug),
        .ld_slave_addr_debug(ld_slave_addr_master_debug),
        .ld_reg_addr_debug(ld_reg_addr_master_debug),
        .master_state_debug(master_state_debug),
        .scl_clk_debug(scl_clk_master_debug),
        .bit_counter_debug(bit_counter_master_debug),
        .scl_out_debug(scl_out_master_debug),
        .sda_out_debug(sda_out_master_debug),
        .slave_addr_debug(slave_addr_master_debug),
        .reg_addr_debug(reg_addr_master_debug)
    );

    // Instantiate the I2C Slave
    simple_i2c #(.SLAVE_ADDRESS(SLAVE_ADDRESS)) slave_i2c (
        .clk(clk),
        .rst(rst),
        .clk_div(clk_div),  // Optional, but should be the same for synchronous operations
        .tx(tx_slave),
        .rx(rx_slave),
        .ctrl(ctrl_slave),
        .status(),  // You can monitor status if needed
        .scl(scl),
        .sda(sda),

        // Debug signals for slave
        .en_debug(en_slave_debug),
        .mode_debug(mode_slave_debug),
        .start_debug(start_slave_debug),
        .stop_debug(stop_slave_debug),
        .slave_rw_debug(slave_rw_debug),
        .ld_slave_addr_debug(ld_slave_addr_slave_debug),
        .ld_reg_addr_debug(ld_reg_addr_slave_debug),
        .slave_state_debug(slave_state_debug),
        .scl_clk_debug(scl_clk_slave_debug),
        .slave_bit_counter_debug(bit_counter_slave_debug),
        .scl_out_debug(scl_out_slave_debug),
        .sda_out_debug(sda_out_slave_debug),
        .slave_addr_debug(slave_addr_slave_debug),
        .reg_addr_debug(reg_addr_slave_debug)
    );

    // Clock generation
    always #10 clk = ~clk;  // 50 MHz clock

    initial begin
        // Initialize signals
        clk = 0;
        rst = 1;
        clk_div = 16'b0; 
        tx_master = 8'h00;
        tx_slave = 8'h00;  // Slave has some data to send back
        ctrl_master = 8'b0;
        ctrl_slave = 8'b0;

        #20;
        rst = 0;

        // init master
        ctrl_master[0] = 1;
        ctrl_master[1] = 1;

        @ (posedge clk);

        // init slave
        ctrl_slave[0] = 1;
        ctrl_slave[1] = 0;

        @ (posedge clk);
        i2c_write(SLAVE_ADDRESS, 'hAA, 'h69);

        #1500;
        $stop;  // Stop the simulation
    end

    task i2c_write;
        reg [7:0] slave_addr;
        reg [7:0] reg_addr;
        reg [7:0] data
        begin
            // Master write to slave
            ctrl_master[4] = 1'b0;  // Set read/write bit
            ctrl_master[5] = 1; // load slave addr
            tx_master = slave_addr;
            @ (posedge clk);
            ctrl_master[5] = 0; // load reg addr
            ctrl_master[6] = 1;
            tx_master = reg_addr;
            @ (posedge clk);
            ctrl_master[6] = 0;
            tx_master = data;
            ctrl_master[2] = 1; // set start bit
            @ (posedge clk);    
            @ (posedge clk);    
            @ (posedge clk);    
            ctrl_master[2] = 0; // Reset start bit
            wait (!status[1])
        end
    endtask

endmodule
