module RegisterFile #(ADDR_WIDTH = 5, DATA_WIDTH = 32) (
    input clk,
    input rst,
    input [DATA_WIDTH-1:0] wdata,
    input [ADDR_WIDTH-1:0] waddr,
    input wen,
    input [ADDR_WIDTH-1:0] raddr1,
    input [ADDR_WIDTH-1:0] raddr2,
    output [DATA_WIDTH-1:0] rdata1,
    output [DATA_WIDTH-1:0] rdata2,
    input ren
);
    
    reg [DATA_WIDTH-1:0] rf [2**ADDR_WIDTH-1:0];
    integer i;
    always @(posedge clk) begin
        if (rst) begin
            for (i=0; i<2**ADDR_WIDTH; i=i+1) begin
                rf[i] <= 0;
            end
        end
        else if (wen) rf[waddr] <= wdata;
    end
    
    assign rdata1 = (ren&(raddr1!=0)) ? rf[raddr1] : 0;
    assign rdata2 = (ren&(raddr2!=0)) ? rf[raddr2] : 0;
`ifdef SIMULATION
    export "DPI-C" function get_reg;
    function void get_reg(int addr);
        output int reg_data;
        reg_data = addr == 0 ? 32'b0 : rf[addr];
    endfunction
`endif
endmodule
