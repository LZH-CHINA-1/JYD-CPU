`include "define.v"
module mulcu_decoder(
        input [31:0] instr,
        input ALU_ZERO,
        output [3:0] alu_op,
        output ebreak,
        output [4:0] Rs1,
        output [4:0] Rs2,
        output [4:0] Rd,
        output jal,
        output jalr,
        output branch,
        output reg_wen,
        output reg_ren,
        output ALU_DB_Src,
        output reg [1:0] Reg_Src,
        output mem_wen,
        output mem_ren,
        output auipc,
        output [2:0] funct3,
        `ifdef RV32M
        output mulsign,
        `endif
        output [6:0] opcode
    );

    wire funct7;
    `ifdef RV32M
        assign mulsign=(instr[25])&&(opcode==`R_type);
    `endif
    assign opcode=instr[6:0];
    assign funct3=instr[14:12];
    assign funct7=instr[30];
    assign Rs1=instr[19:15];
    assign Rs2=instr[24:20];
    assign Rd =instr[11:7];

    wire beq,bne,blt,bge,bltu,bgeu;

    assign beq=(opcode==`B_type)&&(funct3==3'b000);
    assign bne=(opcode==`B_type)&&(funct3==3'b001);
    assign blt=(opcode==`B_type)&&(funct3==3'b100);
    assign bge=(opcode==`B_type)&&(funct3==3'b101);
    assign bltu=(opcode==`B_type)&&(funct3==3'b110);
    assign bgeu=(opcode==`B_type)&&(funct3==3'b111);

    //wire beq_true,bne_true,blt_true,bge_true,bltu_true,bgeu_true;
/*
    assign beq_true=(beq & ALU_ZERO);
    assign bne_true=(bne & ~ALU_ZERO);
    assign blt_true=(blt & ALU_DC[0]);
    assign bge_true=(bge & ~ALU_DC[0]);
    assign bltu_true=(bltu & ALU_DC[0]);
    assign bgeu_true=(bgeu & ~ALU_DC[0]);
*/
    assign branch=( beq | bne | blt | bge| bltu | bgeu);

    assign jal=(opcode==`jal);
    assign jalr=(opcode==`jalr);

    assign auipc=(opcode==`auipc);

    assign reg_wen=~(opcode==`store | opcode==`B_type | opcode==7'b0000000);
    assign reg_ren=1'b1;

    assign ALU_DB_Src=(opcode==`B_type | opcode==`R_type)?1'b1:1'b0;

    assign mem_wen=(opcode==`store)?1'b1:1'b0;
    assign mem_ren=(opcode==`load )?1'b1:1'b0;

    //00:来自ALU 01:来自data 10:来自PC 11:来自imme
    always@(*) begin
        case(opcode)
            `R_type:
                Reg_Src=2'b00;//ALU
            `I_type:
                Reg_Src=2'b00;//ALU
            `load:
                Reg_Src=2'b01;//data
            `jal:
                Reg_Src=2'b10;//PC
            `jalr:
                Reg_Src=2'b10;//PC
            `lui:
                Reg_Src=2'b11;//imme
            `auipc:
                Reg_Src=2'b00;//ALU
            default:
                Reg_Src=2'b00;//ALU

        endcase
    end

    ALU_decoder ALU_decoder(
                    .funct3(funct3),
                    .funct7(funct7),
                    .opcode(opcode),
                    .alu_op(alu_op)
                );


endmodule

module ALU_decoder(
        input [2:0] funct3,
        input  funct7,
        input [6:0] opcode,
        output reg [3:0] alu_op
    );
    reg [3:0] calcu_op;
    always@(*) begin
        case(opcode)
            `R_type:
                alu_op=calcu_op;
            `I_type:
                alu_op=calcu_op;
            `store:
                alu_op=`ADD;
            `load:
                alu_op=`ADD;
            `B_type:
                alu_op=(funct3[1])?`SLTU:`SLT;///000,001,100,101,110,111
            `auipc:
                alu_op=`ADD;
            default:
                alu_op=4'b0000;
        endcase
    end

    always @(*) begin
        case (funct3)
            3'b000:
                calcu_op=(opcode==`R_type) ? ((funct7)?`SUB:`ADD) : `ADD;
            3'b001: 
                calcu_op=`SLL;
            3'b010:
                calcu_op=`SLT;
            3'b011:
                calcu_op=`SLTU;
            3'b100:
                calcu_op=`XOR;
            3'b101:
                calcu_op=(funct7)?`SRA:`SRL;
            3'b110:
                calcu_op=`OR;
            3'b111:
                calcu_op=`AND;
            default:
                calcu_op=4'b0000;
        endcase

    end

endmodule


module Imme_decoder(
        //input [6:0] opcode,
        input [31:0] instr,
        output [31:0] imme,
        output I_type,
        output U_type,
        output J_type,
        output B_type,
        output S_type,
        output ebreak
    );
    wire [31:0] I_imme,U_imme,J_imme,B_imme,S_imme;

    assign I_type=(instr[6:0]==`jalr) | (instr[6:0]==`load) | (instr[6:0]==`I_type);
    assign U_type=(instr[6:0]==`lui) | (instr[6:0]==`auipc);
    assign J_type=(instr[6:0]==`jal);
    assign B_type=(instr[6:0]==`B_type);
    assign S_type=(instr[6:0]==`store);
    assign ebreak=(instr==32'h00100073 || instr==32'h0000006f)?1:0; // ebreak or j 0
    assign I_imme={{20{instr[31]}},instr[31:20]};
    assign U_imme={instr[31:12],{12{1'b0}}};
    assign J_imme={{12{instr[31]}},instr[19:12],instr[20],instr[30:21],1'b0};
    assign B_imme={{20{instr[31]}},instr[7],instr[30:25],instr[11:8],1'b0};
    assign S_imme={{20{instr[31]}},instr[31:25],instr[11:7]};

    assign imme= I_type?I_imme :
           U_type?U_imme :
           J_type?J_imme :
           B_type?B_imme :
           S_type?S_imme : 32'd0;
endmodule
