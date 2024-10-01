`include "simple_i2c_wishbone_mem_map.vh"


module simple_i2c_wishbone_slave #(
    parameter SLAVE_ADDRESS = 7'h50
    )
    (
    input wire           clk,           // Clock
    input wire           rst,           // Reset
    input wire  [5:0]    adr_i,         // Address bus
    input wire  [7:0]    dat_i,         // Data input (from master)
    output reg  [7:0]    dat_o,         // Data output (to master)
    input wire           we_i,          // Write Enable
    input wire           stb_i,         // Strobe signal
    input wire           cyc_i,         // Cycle signal
    output reg           ack_o,         // Acknowledge signal

    inout                scl,           // I2C clk            
    inout                sda,           // I2C data

    // Debug/Verification Signals
    output wire [3:0]  master_state_debug,
    output wire [2:0]  bit_counter_debug,
    output wire [15:0] clk_div_debug,
    output wire [7:0]  clk_div_lo_debug,
    output wire [7:0]  clk_div_hi_debug,
    output wire [7:0]  tx_debug,
    output wire [7:0]  rx_debug,
    output wire [7:0]  ctrl_debug,
    output wire [7:0]  status_debug,
    output wire        en_i2c_debug,
    output wire        mode_i2c_debug,
    output wire        start_i2c_debug,
    output wire        stop_i2c_debug,
    output wire        rw_i2c_debug,
    output wire        scl_out_i2c_debug,
    output wire        sda_out_i2c_debug,
    output wire [7:0]  slave_addr_debug,
    output wire [7:0]  reg_addr_debug
    );

    // Internal signals for registers
    wire [15:0] clk_div;
    reg [7:0] clk_div_lo;
    reg [7:0] clk_div_hi;
    reg [7:0] tx;
    reg [7:0] rx;
    reg [7:0] ctrl;
    reg [7:0] status;

    assign clk_div = {clk_div_hi, clk_div_lo};

    simple_i2c #(
        .SLAVE_ADDRESS(SLAVE_ADDRESS)
    ) 
    i2c_master (
        .clk(clk),
        .rst(rst),
        .clk_div(clk_div),
        .tx(tx),
        .rx(rx),
        .ctrl(ctrl),
        .status(status),
        .scl(scl),
        .sda(sda),

        // Debug/verification connections
        .en_debug(en_i2c_debug),
        .mode_debug(mode_i2c_debug),
        .start_debug(start_i2c_debug),
        .stop_debug(stop_i2c_debug),
        .rw_debug(rw_i2c_debug),
        .master_state_debug(master_state_debug),
        .bit_counter_debug(bit_counter_debug),
        .scl_out_debug(scl_out_i2c_debug),
        .sda_out_debug(sda_out_i2c_debug),
        .slave_addr_debug(slave_addr_debug),
        .reg_addr_debug(reg_addr_debug)
    );

    // Wishbone protocol handshake
    wire valid_wb;
    assign valid_wb = cyc_i && stb_i;

    always @(posedge clk) begin
        if (rst) begin
            ack_o <= 0;
            dat_o <= 5'b0;
            clk_div_lo <= 7'b0;
            clk_div_hi <= 7'b0;
            tx <= 7'b0;
            ctrl <= 7'b0;
        end else begin
            ack_o <= 0;  // Default acknowledge to 0

            if (valid_wb && !ack_o) begin
                ack_o <= 1;  // Acknowledge the transaction in the next clock

                if (we_i) begin
                    // Write operation
                    case (adr_i)
                        `CTRL_REG_ADDR: ctrl <= dat_i;
                        `TX_REG_ADDR: tx <= dat_i;
                        `CLK_DIV_LO_REG_ADDR: clk_div_lo <= dat_i;
                        `CLK_DIV_HI_REG_ADDR: clk_div_hi <= dat_i;
                        default: ;
                    endcase
                end else begin
                    // Read operation
                    case (adr_i)
                        `CTRL_REG_ADDR: dat_o <= ctrl;
                        `STATUS_REG_ADDR: dat_o <= status;
                        `RX_REG_ADDR: dat_o <= rx;
                    endcase
                end
            end
        end
    end

    // Debug assignments
    assign clk_div_debug = clk_div;
    assign clk_div_lo_debug = clk_div_lo;
    assign clk_div_hi_debug = clk_div_hi;
    assign tx_debug = tx;
    assign rx_debug = rx;
    assign ctrl_debug = ctrl;
    assign status_debug = status;

endmodule
