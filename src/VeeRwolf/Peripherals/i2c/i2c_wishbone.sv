`define CTRL_REG_ADDR 5'h00
`define STATUS_REG_ADDR 5'h04
`define TX_REG_ADDR 5'h08
`define RX_REG_ADDR 5'h0C
`define CLK_DIV_LO_REG_ADDR 5'h10
`define CLK_DIV_HI_REG_ADDR 5'h14


module i2c_wishbone #(
    parameter SLAVE_ADDRESS = 7'h50
    )
    (
    input wire           clk,           // Clock
    input wire           rst,           // Reset
    input wire  [4:0]    adr_i,         // Address bus
    input wire  [7:0]    dat_i,         // Data input (from master)
    output reg  [7:0]    dat_o,         // Data output (to master)
    input wire           we_i,          // Write Enable
    input wire           stb_i,         // Strobe signal
    input wire           cyc_i,         // Cycle signal
    output reg           ack_o,         // Acknowledge signal

    inout                scl,           // I2C clk            
    inout                sda,           // I2C data

    // Debug signals
    output wire [3:0] state_debug,
    output wire [2:0] bit_counter_debug
    );

    // Internal signals for registers
    reg  [7:0]  clk_div_lo;
    reg  [7:0]  clk_div_hi;
    reg  [7:0]  tx;
    reg  [7:0]  rx;
    reg  [7:0]  ctrl;
    reg  [7:0]  status;

    i2c_memory_map #(
        .SLAVE_ADDRESS(SLAVE_ADDRESS)
    ) 
    i2c_memory_map_inst (
        .clk(clk),
        .rst(rst),
        .clk_div_lo(clk_div_lo),
        .clk_div_hi(clk_div_hi),
        .tx(tx),
        .rx(rx),
        .ctrl(ctrl),
        .status(status),
        .scl(scl),
        .sda(sda),
        .state_debug(state_debug),
        .bit_counter_debug(bit_counter_debug)
    );

    // Wishbone interface
    always @(posedge clk) begin
        if (rst) begin
            ack_o <= 0;
            dat_o <= 5'b0;
            clk_div_lo <= 8'b0;
            clk_div_hi <= 8'b0;
            tx <= 8'b0;
            ctrl <= 8'b0;
        end else begin
            ack_o <= 0;
            if (cyc_i && stb_i) begin
                ack_o <= 1;
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

endmodule
