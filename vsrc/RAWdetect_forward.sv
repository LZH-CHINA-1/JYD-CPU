module RAWdetect_forward(
    input logic clk,
    input logic rst,
    input logic [4:0] type_D,
    input logic [4:0] type_E,
    input logic [4:0] type_M,
    input logic [4:0] type_W,
    

    input logic [4:0] rs1_D,
    input logic [4:0] rs2_D,
    input logic [4:0] rd_E,
    input logic [4:0] rd_M,
    input logic [4:0] rd_W,

    input logic valid_E,
    input logic ready_E,
    input logic valid_M,
    input logic ready_M,
    input logic valid_W,
    input logic ready_W,

    input logic load_E,
    input logic load_M,

    output logic stall_D,

    input logic [31:0] ALUResult_E,
    input logic [31:0] ALUResult_M,
    input logic [31:0] rdata_M,
    input logic [31:0] wdata,

    output logic [31:0] forward_rs1,
    output logic [31:0] forward_rs2,
    output logic valid_forward_rs1,
    output logic valid_forward_rs2
);

    // Wire signal declarations
    wire RC_rs1, RC_rs2; // RAW Condition for rs1 and rs2
    wire FC_rs1, FC_rs2; // Forwarding Condition for rs1 and rs2
    wire no_rs2; // I-type, U-type, J-type does not have rs2
    wire no_rd_E, no_rd_M, no_rd_W;
    wire conflict_rs1_E, conflict_rs1_M, conflict_rs1_W;
    wire conflict_rs2_E, conflict_rs2_M, conflict_rs2_W;
    wire idle_E, idle_M, idle_W;
    wire RC_rs1_E, RC_rs1_M, RC_rs1_W;
    wire FC_rs1_E, FC_rs1_M, FC_rs1_W;
    wire FC_rs2_E, FC_rs2_M, FC_rs2_W;
    wire [31:0] wdata_E, wdata_M, wdata_W;

    // Define missing signals to resolve IMPLICIT warnings
    wire RC_rs2_E, RC_rs2_M, RC_rs2_W;

    // Wire signal assignments
    // 第一级：产生中间信号（组合逻辑）
    assign no_rs2 = |(type_D[2:0]);
    assign no_rd_E = |(type_E[4:3]); 
    assign no_rd_M = |(type_M[4:3]);
    assign no_rd_W = |(type_W[4:3]); 

    assign conflict_rs1_E = (rs1_D == rd_E) && (rd_E != 0) && (~no_rd_E);
    assign conflict_rs1_M = (rs1_D == rd_M) && (rd_M != 0) && (~no_rd_M);
    assign conflict_rs1_W = (rs1_D == rd_W) && (rd_W != 0) && (~no_rd_W);
    assign conflict_rs2_E = (rs2_D == rd_E) && (rd_E != 0) && (~no_rd_E);
    assign conflict_rs2_M = (rs2_D == rd_M) && (rd_M != 0) && (~no_rd_M);
    assign conflict_rs2_W = (rs2_D == rd_W) && (rd_W != 0) && (~no_rd_W);

    assign idle_E = (~valid_E) & ready_E;
    assign idle_M = (~valid_M) & ready_M;
    assign idle_W = (~valid_W) & ready_W;

    // 第一级输出
    wire RC_rs1_E_comb = (~idle_E) & conflict_rs1_E;
    wire RC_rs1_M_comb = (~idle_M) & conflict_rs1_M;
    wire RC_rs1_W_comb = (~idle_W) & conflict_rs1_W;
    wire RC_rs2_E_comb = (~no_rs2) & (~idle_E) & conflict_rs2_E;
    wire RC_rs2_M_comb = (~no_rs2) & (~idle_M) & conflict_rs2_M;
    wire RC_rs2_W_comb = (~no_rs2) & (~idle_W) & conflict_rs2_W;

    wire FC_rs1_E_comb = RC_rs1_E_comb & (~load_E);
    wire FC_rs1_M_comb = RC_rs1_M_comb & valid_M;
    wire FC_rs1_W_comb = RC_rs1_W_comb;
    wire FC_rs2_E_comb = RC_rs2_E_comb & (~load_E);
    wire FC_rs2_M_comb = RC_rs2_M_comb & valid_M;
    wire FC_rs2_W_comb = RC_rs2_W_comb;

    wire [31:0] wdata_E_comb = ALUResult_E;
    wire [31:0] wdata_M_comb = load_M ? rdata_M : ALUResult_M;
    wire [31:0] wdata_W_comb = wdata;

    // ========== 插入流水寄存器 ==========
    logic RC_rs1_E_r, RC_rs1_M_r, RC_rs1_W_r;
    logic RC_rs2_E_r, RC_rs2_M_r, RC_rs2_W_r;
    logic FC_rs1_E_r, FC_rs1_M_r, FC_rs1_W_r;
    logic FC_rs2_E_r, FC_rs2_M_r, FC_rs2_W_r;
    logic [31:0] wdata_E_r, wdata_M_r, wdata_W_r;

    always_ff @(posedge clk) begin
        if (rst) begin
            RC_rs1_E_r <= 1'b0;
            RC_rs1_M_r <= 1'b0;
            RC_rs1_W_r <= 1'b0;
            RC_rs2_E_r <= 1'b0;
            RC_rs2_M_r <= 1'b0;
            RC_rs2_W_r <= 1'b0;
            FC_rs1_E_r <= 1'b0;
            FC_rs1_M_r <= 1'b0;
            FC_rs1_W_r <= 1'b0;
            FC_rs2_E_r <= 1'b0;
            FC_rs2_M_r <= 1'b0;
            FC_rs2_W_r <= 1'b0;
            wdata_E_r  <= 32'b0;
            wdata_M_r  <= 32'b0;
            wdata_W_r  <= 32'b0;
        end else if(ready_E)begin
            RC_rs1_E_r <= RC_rs1_E_comb;
            RC_rs1_M_r <= RC_rs1_M_comb;
            RC_rs1_W_r <= RC_rs1_W_comb;
            RC_rs2_E_r <= RC_rs2_E_comb;
            RC_rs2_M_r <= RC_rs2_M_comb;
            RC_rs2_W_r <= RC_rs2_W_comb;
            FC_rs1_E_r <= FC_rs1_E_comb;
            FC_rs1_M_r <= FC_rs1_M_comb;
            FC_rs1_W_r <= FC_rs1_W_comb;
            FC_rs2_E_r <= FC_rs2_E_comb;
            FC_rs2_M_r <= FC_rs2_M_comb;
            FC_rs2_W_r <= FC_rs2_W_comb;
            wdata_E_r  <= wdata_E_comb;
            wdata_M_r  <= wdata_M_comb;
            wdata_W_r  <= wdata_W_comb;
        end
    end


    // Stall condition: if any of the rs1 or rs2 is in conflict and not forwarded
    assign stall_D = 
    ((~idle_E) & load_E & (conflict_rs1_E | ((~no_rs2) & conflict_rs2_E))) |
     ((~idle_M) & (~valid_M) & (conflict_rs1_M | ((~no_rs2) & conflict_rs2_M)));

    assign forward_rs1 = (FC_rs1_E_r) ? wdata_E_r :
                        (FC_rs1_M_r) ? wdata_M_r :
                        (FC_rs1_W_r) ? wdata_W_r : 32'b0;
    
    assign forward_rs2 = (FC_rs2_E_r) ? wdata_E_r :
                        (FC_rs2_M_r) ? wdata_M_r :
                        (FC_rs2_W_r) ? wdata_W_r : 32'b0;
    
    assign valid_forward_rs1 = FC_rs1_E_r | FC_rs1_M_r | FC_rs1_W_r;
    assign valid_forward_rs2 = FC_rs2_E_r | FC_rs2_M_r | FC_rs2_W_r;

endmodule
