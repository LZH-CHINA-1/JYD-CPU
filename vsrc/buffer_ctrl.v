`timescale 1ns / 1ps
`include "define.v"
`include "pipeline_config.v"
/* verilator lint_off DECLFILENAME */
/* verilator lint_off MULTITOP */

// 取指到解码阶段缓冲区（F->D）
module buffer_F_D(
        input clk,
        input rst,
        input [31:0] instr_F,
        input [31:0] PC_reg_F,
        input predict_F,

        output reg [31:0] instr_D,
        output reg [31:0] PC_reg_D,
        output reg predict_D,

        input valid_F,
        input ready_D
    );

    // 数据通路合并到always块中
    always @(posedge clk) begin
        if (rst) begin
            instr_D <= 32'b0;
            PC_reg_D <= 32'h8000_0000;
            predict_D <= 1'b0; // 复位预测信号
        end
        else if (valid_F & ready_D) begin
            instr_D <= instr_F;
            PC_reg_D <= PC_reg_F;
            predict_D <= predict_F; // 直接传递预测信号
        end
    end

endmodule

// 解码到执行阶段缓冲区（D->E）
module buffer_D_E(
        // 时钟和复位
        input clk,
        input rst,
        input valid_D,
        input ready_E,

        // 控制信号输入
        input RegWrite_D,
        input [1:0] ResultSrc_D,
        input MemWrite_D,
        input MemRead_D,
        input jal_D,
        input Branch_D,
        input [3:0] ALUControl_D,
        input ALUSrc_D,
        input auipc_D,
        input [2:0] funct3_D,
        input reg_ren_D,
        input [6:0] opcode_D,
        input jalr_D,
        input predict_D,
        input ebreak_D,
        input [4:0] type_D,
`ifdef RV32M
        input mulsign_D,
`endif

        // 数据通路输入
        input [31:0] PC_reg_D,
        input [31:0] imme_D,
        input [31:0] rdata1_D,
        input [31:0] rdata2_D,
        input [4:0] Rd_D,
        input [4:0] Rs1_D,
        input [4:0] Rs2_D,

        // 控制信号输出
        output reg RegWrite_E,
        output reg [1:0] ResultSrc_E,
        output reg MemWrite_E,
        output reg MemRead_E,
        output reg jal_E,
        output reg Branch_E,
        output reg [3:0] ALUControl_E,
        output reg ALUSrc_E,
        output reg auipc_E,
        output reg [2:0] funct3_E,
        output reg reg_ren_E,
        output reg [6:0] opcode_E,
        output reg jalr_E,
        output reg predict_E,
        output reg ebreak_E,
        output reg [4:0] type_E,
`ifdef RV32M
        output reg mulsign_E,
`endif

        // 数据通路输出
        output reg [31:0] PC_reg_E,
        output reg [31:0] imme_E,
        output reg [31:0] rdata1_E,
        output reg [31:0] rdata2_E,
        output reg [4:0] Rd_E,
        output reg [4:0] Rs1_E,
        output reg [4:0] Rs2_E
    );

    // 控制和数据通路信号统一处理
    always @(posedge clk) begin
        if (rst) begin
            // 控制信号复位
            RegWrite_E <= 1'b0;
            ResultSrc_E <= 2'b0;
            MemWrite_E <= 1'b0;
            MemRead_E <= 1'b0;
            jal_E <= 1'b0;
            Branch_E <= 1'b0;
            ALUControl_E <= 4'b0;
            ALUSrc_E <= 1'b0;
            auipc_E <= 1'b0;
            funct3_E <= 3'b0;
            reg_ren_E <= 1'b0;
            opcode_E <= 7'b0;
            jalr_E <= 1'b0;
            predict_E <= 1'b0;
            ebreak_E <= 1'b0;
            type_E <= 5'b0;
`ifdef RV32M
            mulsign_E <= 1'b0;
`endif

            // 数据通路复位
            PC_reg_E <= 32'h8000_0000;
            imme_E <= 32'b0;
            rdata1_E <= 32'b0;
            rdata2_E <= 32'b0;
            Rd_E <= 5'b0;
            Rs1_E <= 5'b0;
            Rs2_E <= 5'b0;
        end
        else if (valid_D & ready_E) begin
            // 控制信号赋值
            RegWrite_E <= RegWrite_D;
            ResultSrc_E <= ResultSrc_D;
            MemWrite_E <= MemWrite_D;
            MemRead_E <= MemRead_D;
            jal_E <= jal_D;
            Branch_E <= Branch_D;
            ALUControl_E <= ALUControl_D;
            ALUSrc_E <= ALUSrc_D;
            auipc_E <= auipc_D;
            funct3_E <= funct3_D;
            reg_ren_E <= reg_ren_D;
            opcode_E <= opcode_D;
            jalr_E <= jalr_D;
            predict_E <= predict_D;
            ebreak_E <= ebreak_D;
            type_E <= type_D;
`ifdef RV32M
            mulsign_E <= mulsign_D;
`endif

            // 数据通路赋值
            PC_reg_E <= PC_reg_D;
            imme_E <= imme_D;
            rdata1_E <= rdata1_D;
            rdata2_E <= rdata2_D;
            Rd_E <= Rd_D;
            Rs1_E <= Rs1_D;
            Rs2_E <= Rs2_D;
        end
    end


endmodule

// 执行到内存阶段缓冲区（E->M）
module buffer_E_M(
        // 时钟和复位
        input clk,
        input rst,
        input valid_E,
        input ready_M,

        // 控制信号输入
        input RegWrite_E,
        input [1:0] ResultSrc_E,
        input MemWrite_E,
        input MemRead_E,
        input [2:0] funct3_E,
        input ebreak_E,
        input [4:0] type_E,

        // 数据通路输入
        input [31:0] ALUResult_E,
        input [31:0] WriteData_E,
        input [4:0] Rd_E,
        input [31:0] PC_reg_E,
        input [31:0] imme_E,

        // 控制信号输出
        output reg RegWrite_M,
        output reg [1:0] ResultSrc_M,
        output reg MemWrite_M,
        output reg MemRead_M,
        output reg [2:0] funct3_M,
        output reg ebreak_M,
        output reg [4:0] type_M,

        // 数据通路输出
        output reg [31:0] ALUResult_M,
        output reg [31:0] WriteData_M,
        output reg [4:0] Rd_M,
        output reg [31:0] PC_reg_M,
        output reg [31:0] imme_M
    );

    // 控制和数据通路信号统一处理
    always @(posedge clk) begin
        if (rst) begin
            // 控制信号复位
            RegWrite_M <= 1'b0;
            ResultSrc_M <= 2'b0;
            MemWrite_M <= 1'b0;
            MemRead_M <= 1'b0;
            funct3_M <= 3'b0;
            ebreak_M <= 1'b0;
            type_M <= 5'b0;

            // 数据通路复位
            ALUResult_M <= 32'b0;
            WriteData_M <= 32'b0;
            Rd_M <= 5'b0;
            PC_reg_M <= 32'h80000000;
            imme_M <= 32'b0;
        end
        else if (valid_E & ready_M) begin
            // 控制信号赋值
            RegWrite_M <= RegWrite_E;
            ResultSrc_M <= ResultSrc_E;
            MemWrite_M <= MemWrite_E;
            MemRead_M <= MemRead_E;
            funct3_M <= funct3_E;
            ebreak_M <= ebreak_E;
            type_M <= type_E;

            // 数据通路赋值
            ALUResult_M <= ALUResult_E;
            WriteData_M <= WriteData_E;
            Rd_M <= Rd_E;
            PC_reg_M <= PC_reg_E;
            imme_M <= imme_E;
        end
    end

endmodule

// 内存到写回阶段缓冲区（M->W）
module buffer_M_W(
        // 时钟和复位
        input clk,
        input rst,
        input valid_M,
        input ready_W,

        // 控制信号输入
        input RegWrite_M,
        input [1:0] ResultSrc_M,
        input [2:0] funct3_M,
        input ebreak_M,
        input [4:0] type_M,

        // 数据通路输入
        input [31:0] ALUResult_M,
        input [31:0] ReadData_M,
        input [31:0] PC_reg_M,
        input [4:0] Rd_M,
        input [31:0] imme_M,

        // 控制信号输出
        output reg RegWrite_W,
        output reg [1:0] ResultSrc_W,
        output reg [2:0] funct3_W,
        output reg ebreak_W,
        output reg [4:0] type_W,

        // 数据通路输出
        output reg [31:0] ALUResult_W,
        output reg [31:0] ReadData_W,
        output reg [4:0] Rd_W,
        output reg [31:0] PC_reg_W,
        output reg [31:0] imme_W
    );

    // 控制和数据通路信号统一处理
    always @(posedge clk) begin
        if (rst) begin
            // 控制信号复位
            RegWrite_W <= 1'b0;
            ResultSrc_W <= 2'b0;
            funct3_W <= 3'b0;
            ebreak_W <= 1'b0;
            type_W <= 5'b0;

            // 数据通路复位
            ALUResult_W <= 32'b0;
            ReadData_W <= 32'b0;
            Rd_W <= 5'b0;
            PC_reg_W <= 32'h8000_0000; // 注意：此寄存器有特殊的复位值
            imme_W <= 32'b0;
        end
        else if (valid_M & ready_W) begin
            // 控制信号赋值
            RegWrite_W <= RegWrite_M;
            ResultSrc_W <= ResultSrc_M;
            funct3_W <= funct3_M;
            ebreak_W <= ebreak_M;
            type_W <= type_M;

            // 数据通路赋值
            ALUResult_W <= ALUResult_M;
            ReadData_W <= ReadData_M;
            Rd_W <= Rd_M;
            PC_reg_W <= PC_reg_M;
            imme_W <= imme_M;
        end
    end

endmodule

