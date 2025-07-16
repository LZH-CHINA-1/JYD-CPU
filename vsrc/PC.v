`include "define.v"
module PC(
    input clk,
    input rst,
    input [31:0] PC_src,
    input valid_in,
    output reg [31:0] PC_reg
);


always@(posedge clk) begin
    if(rst) begin
        PC_reg <= `PC_rst;
    end
    else begin
        if(valid_in)
        PC_reg <= PC_src;
    end
end


endmodule
