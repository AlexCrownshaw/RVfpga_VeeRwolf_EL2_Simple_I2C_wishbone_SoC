module i2c_memory_map #(
    parameter SLAVE_ADDRESS = 7'h50 
    ) 
    (
    input  wire          clk,         
    input  wire          rst,
    input  wire [7:0]    clk_div_lo,
    input  wire [7:0]    clk_div_hi,
    input  wire [7:0]    tx,
    output wire [7:0]    rx,
    input  wire [7:0]    ctrl,
    output wire [7:0]    status,

    inout                scl,
    inout                sda,

    // Debug signals
    output wire [3:0] state_debug,
    output wire [2:0] bit_counter_debug
    );

    // Clock divider
    wire [15:0] clk_div;
    assign clk_div = {clk_div_hi, clk_div_lo};

    // Control register
    wire en;
    wire mode;
    wire start;
    wire stop;
    wire rw;
    wire ld_slave_addr;
    wire ld_reg_addr;

    assign en = ctrl[0];
    assign mode = ctrl[1];
    assign start = ctrl[2];
    assign stop = ctrl[3];
    assign rw = ctrl[4];
    assign ld_slave_addr = ctrl[5];
    assign ld_reg_addr = ctrl[6];

    // Status register
    wire busy;
    wire done;
    wire nack;

    assign status[0] = busy;
    assign status[1] = done;
    assign status[2] = nack;
    assign status[7:3] = 5'b0;

    // Slave Address register
    reg  [7:0] slave_addr;
    wire [7:0] slave_addr_wire;

    assign slave_addr_wire = slave_addr;

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
    reg  [7:0] reg_addr;
    wire [7:0] reg_addr_wire;

    assign reg_addr_wire = reg_addr;

    always @ (posedge clk) begin
        if (rst) begin
            reg_addr <= 8'b0;
        end else begin
            if (ld_reg_addr) begin
                reg_addr <= tx;
            end
        end
    end

    // I2C core instance
    i2c_core #(
        .SLAVE_ADDRESS(SLAVE_ADDRESS)
    ) i2c_core_inst (
        .clk(clk),
        .rst(rst),
        .tx(tx),
        .rx(rx),
        .slave_addr(slave_addr_wire),
        .reg_addr(reg_addr_wire),
        .clk_div(clk_div),
        .en(en),
        .mode(mode),
        .start(start),
        .stop(stop),
        .rw(rw),
        .busy(busy),
        .done(done),
        .nack(nack),
        .scl(scl),
        .sda(sda),
        .state_debug(state_debug),
        .bit_counter_debug(bit_counter_debug)
    );

endmodule
