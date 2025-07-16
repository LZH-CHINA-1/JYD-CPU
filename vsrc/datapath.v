`include "define.v"
`include "pipeline_config.v"

`timescale 1ns / 1ps
module datapath(
        input clk,
        input rst,

        // AXI4-Lite 指令接口 (Master)
        output [31:0]   inst_axi_araddr,
        output          inst_axi_arvalid,
        input           inst_axi_arready,
        input [31:0]    inst_axi_rdata,
        input [1:0]     inst_axi_rresp,
        input           inst_axi_rvalid,
        output          inst_axi_rready,

        // AXI4-Lite 数据接口 (Master)
        // 读通道
        output [31:0]   data_axi_araddr,
        output          data_axi_arvalid,
        input           data_axi_arready,
        input [31:0]    data_axi_rdata,
        input [1:0]     data_axi_rresp,
        input           data_axi_rvalid,
        output          data_axi_rready,
        // 写通道
        output [31:0]   data_axi_awaddr,
        output          data_axi_awvalid,
        input           data_axi_awready,
        output [31:0]   data_axi_wdata,
        output [3:0]    data_axi_wstrb,
        output          data_axi_wvalid,
        input           data_axi_wready,
        input [1:0]     data_axi_bresp,
        input           data_axi_bvalid,
        output          data_axi_bready,

        output [31:0] PC_W,
        output valid_W_out,
        output ebreak
    );
    assign valid_W_out = valid_W;
    // AXI4Lite 接口信号

    wire [31:0] instr_F;
    wire [31:0] ReadData_M;
    wire MemWrite_M;
    wire MemRead_M;

    // 指令AXI4Lite接口连接
    assign inst_axi_araddr = PC_reg_F;          // 使用PC作为指令地址
    assign inst_axi_arvalid = ready_F;          // 当F阶段有效时发起读请求
    assign inst_axi_rready = 1'b1;              // 始终准备接收指令数据
    assign instr_F = inst_axi_rdata;            // 从AXI读取的指令数据

    // 数据AXI4Lite接口连接
    // 读通道
    assign data_axi_araddr = ALUResult_M;       // 使用ALU结果作为数据地址
    assign data_axi_arvalid = MemRead_M;        // 当需要读内存时发起读请求
    assign data_axi_rready = ready_W;              // WB准备好时可以接收数据
    assign ReadData_M = data_axi_rdata;         // 从AXI读取的数据

    // 写通道
    assign data_axi_awaddr = ALUResult_M;       // 使用ALU结果作为写地址
    assign data_axi_awvalid = MemWrite_M;       // 当需要写内存时发起写请求
    assign data_axi_wdata = mem_data_out;       // 写数据
    assign data_axi_wstrb = wmask;         // 写字节使能，从wmask转换
    assign data_axi_wvalid = MemWrite_M;        // 写数据有效信号
    assign data_axi_bready = 1'b1;              // 始终准备接收写响应


    //--------------------------
    // 控制信号
    wire RegWrite_D;
    wire [3:0] ALUControl_D, ALUControl_E;
    wire [4:0] Rs1_D, Rs2_D, Rs1_E, Rs2_E, Rd_D, Rd_W, Rd_M, Rd_E;
    wire Branch_D;
    wire MemWrite_D, MemWrite_E;
    wire MemRead_D, MemRead_E;
    wire ALU_DB_Src_D;
    wire [1:0] ResultSrc_D, ResultSrc_E, ResultSrc_M, ResultSrc_W;
    wire [2:0] funct3_D, funct3_E, funct3_M;
    wire reg_ren_D, reg_ren_E;
    wire RegWrite_E, RegWrite_M, RegWrite_W;
    wire auipc_D;
    wire jal_D, jal_E;
    wire jalr_D, jalr_E;
    wire Branch_E;
    wire ALU_DB_Src_E;
    wire auipc_E;
    wire [6:0] opcode_D, opcode_E;
    wire [2:0] funct3_W;
    wire ALU_OverFlow;
    wire ebreak_D, ebreak_E, ebreak_M, ebreak_W;

    // 指令类型信号
    wire [4:0] type_D, type_E, type_M, type_W;

    wire ALU_ZERO;

    // 数据信号
    wire [31:0] imme_D, imme_E, imme_M, imme_W;
    wire [31:0] ALUResult_W, ALUResult_M;
    wire [31:0] rdata1_D, rdata2_D, rdata1_E, rdata2_E, rdata2_M;
    wire [31:0] PC_reg_D, PC_reg_E;
    wire [31:0] instr_D;
    wire [31:0] rdata_W;
    wire [31:0] PC_reg_M, PC_reg_W;
    wire [31:0] ALU_DA, ALU_DB;
    wire [31:0] ALUResult_E_RAW;
    reg [31:0] ALUResult_E;


    `ifdef RV32M
    wire mulsign_D;
        
    `endif
    wire valid_F, valid_D, valid_E, valid_M, valid_W;
    wire ready_F, ready_D, ready_E, ready_M, ready_W;


        reg [31:0] instr_F_r, instr_D_r, instr_E_r, instr_M_r, instr_W_r;
    

    valid_ctrl u_valid_ctrl(
                   .clk             	(clk              ),
                   .rst             	(rst              ),
                   //.arready         	(data_axi_arready          ),
                   .rvalid          	(data_axi_rvalid           ),
                   .load_M          	(MemRead_M           ),
                   .load_E          	(MemRead_E           ),
                   .stall           	(stall_D            ),
                   .Pre_Wrong        	(Pre_Wrong         ),
                   .valid_F         	(valid_F          ),
                   .valid_D         	(valid_D          ),
                   .valid_E         	(valid_E          ),
                   .valid_M         	(valid_M          ),
                   .valid_W         	(valid_W          ),
                   .ready_F         	(ready_F          ),
                   .ready_D         	(ready_D          ),
                   .ready_E         	(ready_E          ),
                   .ready_M         	(ready_M          ),
                   .ready_W         	(ready_W          )
               );


    //--------------------------------------------------------------------------------

    // Decoder generate control signal
`ifdef RV32M
wire mulsign_E;
     `endif
    mulcu_decoder mulcu1(
                      .instr(instr_D),
                      .ALU_ZERO(ALU_ZERO),
                      .alu_op(ALUControl_D),
                      .ebreak(ebreak_D),
                      .Rs1(Rs1_D),
                      .Rs2(Rs2_D),
                      .Rd(Rd_D),
                      .branch(Branch_D),
                      .reg_wen(RegWrite_D),
                      .reg_ren(reg_ren_D),
                      .ALU_DB_Src(ALU_DB_Src_D),
                      .Reg_Src(ResultSrc_D),
                      .mem_wen(MemWrite_D),
                      .mem_ren(MemRead_D),
                      .auipc(auipc_D),
                      .jal(jal_D),
                      .jalr(jalr_D),
                      .funct3(funct3_D),
                        `ifdef RV32M
                        .mulsign(mulsign_D),
                        `endif
                      .opcode(opcode_D)
                  );

    Imme_decoder u_Imme_decoder(
                     .instr  	(instr_D   ),
                     .imme   	(imme_D    ),
                     .I_type  	(type_D[0]   ),
                     .U_type  	(type_D[1]   ),
                     .J_type  	(type_D[2]   ),
                     .B_type  	(type_D[3]   ),
                     .S_type  	(type_D[4]   ),
                     .ebreak 	(ebreak_D  )
                 );

    // output declaration of module RAWdetect_forward
    wire stall_D;
    wire [31:0] forward_rs1;
    wire [31:0] forward_rs2;
    wire valid_forward_rs1;
    wire valid_forward_rs2;

    RAWdetect_forward u_RAWdetect_forward(
                            .clk               	(clk                ),
                            .rst               	(rst                ),
                          .type_D            	(type_D             ),
                          .type_E            	(type_E             ),
                          .type_M            	(type_M             ),
                          .type_W            	(type_W             ),
                          .rs1_D             	(Rs1_D              ),
                          .rs2_D             	(Rs2_D              ),
                          .rd_E              	(Rd_E               ),
                          .rd_M              	(Rd_M               ),
                          .rd_W              	(Rd_W               ),
                          .valid_E           	(valid_E            ),
                          .ready_E           	(ready_E            ),
                          .valid_M           	(valid_M            ),
                          .ready_M           	(ready_M            ),
                          .valid_W           	(valid_W            ),
                          .ready_W           	(ready_W            ),
                          .load_E            	(MemRead_E             ),
                          .load_M            	(MemRead_M             ),
                          .stall_D           	(stall_D            ),
                          .ALUResult_E       	(ALUResult_E        ),
                          .ALUResult_M       	(ALUResult_M        ),
                          .rdata_M           	(rdata_M            ),
                          .wdata             	(wdata              ),
                          .forward_rs1       	(forward_rs1        ),
                          .forward_rs2       	(forward_rs2        ),
                          .valid_forward_rs1 	(valid_forward_rs1  ),
                          .valid_forward_rs2 	(valid_forward_rs2  )
                      );


    // 转发到ID阶段的源操作数
     wire [31:0] src1, src2;
     assign src1 = (valid_forward_rs1) ? forward_rs1 : rdata1_E;
     assign src2 = (valid_forward_rs2) ? forward_rs2 : rdata2_E;

    
    //译码阶段，计算分支指令的目标地址
    wire [31:0] branch_target;
    assign branch_target = PC_reg_D + imme_D;

    buffer_F_D u_buffer_F_D(
                   .clk            	(clk             ),
                   .rst            	(rst         ),

                   .instr_F        	(instr_F         ),
                   .PC_reg_F       	(PC_reg_F        ),
                   .predict_F      	(predict_F       ),

                   .instr_D        	(instr_D         ),
                   .PC_reg_D       	(PC_reg_D        ),
                   .predict_D      	(predict_D       ),

                   .valid_F        	    (valid_F         ),
                   .ready_D        	    (ready_D         )
               );

    buffer_D_E u_buffer_D_E(
                   .clk          	(clk           ),
                   .rst          	(rst        ),
                   .valid_D      	(valid_D       ),
                   .ready_E      	(ready_E       ),

                   // 控制信号
                   .RegWrite_D   	(RegWrite_D    ),
                   .ResultSrc_D  	(ResultSrc_D ),
                   .MemWrite_D   	(MemWrite_D   ),
                   .MemRead_D    	(MemRead_D     ),
                   .jal_D       	(jal_D       ),
                   .jalr_D     	(jalr_D      ),
                   .Branch_D     	(Branch_D    ),
                   .ALUControl_D 	(ALUControl_D  ),
                   .ALUSrc_D     	(ALU_DB_Src_D      ),
                   .auipc_D      	(auipc_D       ),
                   .funct3_D     	(funct3_D      ),
                   .reg_ren_D    	(reg_ren_D     ),
                   .opcode_D     	(opcode_D      ),
                   .predict_D    	(predict_D     ),
                   .ebreak_D     	(ebreak_D      ),
                   .type_D       	(type_D        ),
`ifdef RV32M
                   .mulsign_D    	(mulsign_D     ),
`endif

                   // 数据
                   .PC_reg_D       (PC_reg_D        ),
                   .imme_D         (imme_D          ),
                   .rdata1_D       (rdata1_D        ),
                   .rdata2_D       (rdata2_D        ),
                   .Rd_D           (Rd_D            ),
                   .Rs1_D          (Rs1_D           ),
                   .Rs2_D          (Rs2_D           ),

                   // 控制信号输出
                   .RegWrite_E   	(RegWrite_E    ),
                   .ResultSrc_E  	(ResultSrc_E   ),
                   .MemWrite_E   	(MemWrite_E    ),
                   .MemRead_E    	(MemRead_E     ),
                   .jal_E       	(jal_E        ),
                   .jalr_E     	(jalr_E      ),
                   .Branch_E     	(Branch_E      ),
                   .ALUControl_E 	(ALUControl_E  ),
                   .ALUSrc_E     	(ALU_DB_Src_E      ),
                   .auipc_E      	(auipc_E       ),
                   .funct3_E     	(funct3_E      ),
                   .reg_ren_E    	(reg_ren_E     ),
                   .opcode_E     	(opcode_E      ),
                   .predict_E    	(predict_E     ),
                   .ebreak_E     	(ebreak_E      ),
                   .type_E       	(type_E        ),
`ifdef RV32M
                   .mulsign_E    	(mulsign_E     ),
`endif

                   // 数据输出
                   .PC_reg_E       (PC_reg_E        ),
                   .imme_E         (imme_E          ),
                   .rdata1_E       (rdata1_E        ),
                   .rdata2_E       (rdata2_E        ),
                   .Rd_E           (Rd_E            ),
                   .Rs1_E          (Rs1_E           ),
                   .Rs2_E          (Rs2_E           )
               );

    buffer_E_M u_buffer_E_M(
                   .clk            	(clk             ),
                   .rst            	(rst             ),
                   .valid_E        	(valid_E         ),
                   .ready_M        	(ready_M         ),

                   // 控制信号输入
                   .RegWrite_E       (RegWrite_E      ),
                   .ResultSrc_E      (ResultSrc_E     ),
                   .MemWrite_E       (MemWrite_E      ),
                   .MemRead_E        (MemRead_E       ),
                   .funct3_E         (funct3_E        ),
                   .ebreak_E         (ebreak_E        ),
                   .type_E           (type_E          ),

                   // 数据输入
                   .ALUResult_E    	(ALUResult_E     ),
                   .WriteData_E    	(src2        ),
                   .Rd_E           	(Rd_E            ),
                   .PC_reg_E 	      (PC_reg_E        ),
                   .imme_E         	(imme_E          ),

                   // 控制信号输出
                   .RegWrite_M       (RegWrite_M      ),
                   .ResultSrc_M      (ResultSrc_M     ),
                   .MemWrite_M       (MemWrite_M      ),
                   .MemRead_M        (MemRead_M       ),
                   .funct3_M         (funct3_M        ),
                   .ebreak_M         (ebreak_M        ),
                   .type_M           (type_M          ),

                   // 数据输出
                   .ALUResult_M    	(ALUResult_M     ),
                   .WriteData_M    	(rdata2_M        ),
                   .Rd_M           	(Rd_M            ),
                   .PC_reg_M 	      (PC_reg_M        ),
                   .imme_M         	(imme_M          )
               );

    buffer_M_W u_buffer_M_W(
                   .clk            	(clk             ),
                   .rst            	(rst    ),
                   .valid_M        	(valid_M         ),
                   .ready_W        	(ready_W         ),

                   // 控制信号输入
                   .RegWrite_M       (RegWrite_M      ),
                   .ResultSrc_M      (ResultSrc_M     ),
                   .funct3_M         (funct3_M        ),
                   .ebreak_M         (ebreak_M        ),
                   .type_M           (type_M          ),

                   // 数据输入
                   .ALUResult_M    	(ALUResult_M     ),
                   .ReadData_M    	  (rdata_M      ),
                   .PC_reg_M 	      (PC_reg_M        ),
                   .Rd_M           	(Rd_M            ),
                   .imme_M         	(imme_M          ),

                   // 控制信号输出
                   .RegWrite_W       (RegWrite_W      ),
                   .ResultSrc_W      (ResultSrc_W     ),
                   .funct3_W         (funct3_W        ),
                   .ebreak_W         (ebreak_W        ),
                   .type_W           (type_W          ),

                   // 数据输出
                   .ALUResult_W    	(ALUResult_W     ),
                   .ReadData_W    	  (rdata_W      ),
                   .Rd_W           	(Rd_W            ),
                   .PC_reg_W 	      (PC_reg_W        ),
                   .imme_W         	(imme_W          )
               );


    //--------------------------------------------------------------------------------------------


    reg [31:0] PC_src;
    wire [31:0] snpc,PC_jump,PC_jalr;

    // BTB相关信号
    wire btb_hit;
    wire [31:0] btb_target_addr;
    wire btb_update;
    wire [31:0] btb_update_target;

    // BTB实例化
    BTB u_BTB(
            .clk(clk),
            .rst(rst),
            .valid_in(btb_update),           // ID阶段译出分支指令可更新BTB
            .branch_PC(PC_reg_D),            // 当前执行的分支指令PC
            .branch_target(btb_update_target), // 实际的分支目标地址
            .PC_in(PC_reg_F),                // IF阶段的PC
            .hit(btb_hit),                   // BTB命中信号
            .target_addr(btb_target_addr)    // 预测的分支目标地址
        );
    wire [31:0] PC_reg_F;
    PC PC_1(
           .clk(clk),
           .rst(rst),
           .PC_src(PC_src),
           .PC_reg(PC_reg_F),
           .valid_in(ready_D)
       );
    wire Jump_sign;
    // PC计算
    assign snpc = PC_reg_F + 4; // static next pc
    assign PC_jump = PC_reg_E + imme_E;
    assign PC_jalr = imme_E + ALU_DA;
    assign Jump_sign = jalr_E | jal_E | (Branch_E & branch_true);

    // BTB更新逻辑 - ID阶段译码后可以更新
    assign btb_update = (Branch_D & valid_D) | jal_D;
    assign btb_update_target = branch_target;

    wire Pre_Wrong;
    wire Pre_Wrong_valid = Pre_Wrong & valid_E; // 预测错误且E阶段有效

`ifdef Predict

    // 使用BTB进行分支预测
    wire [31:0] PC_predict_path, PC_correction_path;

    // // 分支预测状态机（方向预测）
    // reg [1:0] state;
    // wire predict_ctrl = (state[1] == 1'b1); // MSB 为 1 表示预测跳转

    // // 两位饱和计数器更新逻辑
    // always @(posedge clk or posedge rst) begin
    //     if (rst) begin
    //         state <= 2'b00; // 初始化为强烈不跳转
    //     end
    //     else if (Branch_E) begin // 仅在 EX 阶段为分支指令时更新状态
    //         case (state)
    //             2'b00: // 强烈不跳转
    //                 state <= Pre_Wrong ? 2'b01 : 2'b00;
    //             2'b01: // 弱不跳转
    //                 state <= Pre_Wrong ? 2'b10 : 2'b00;
    //             2'b10: // 弱跳转
    //                 state <= Pre_Wrong ? 2'b01 : 2'b11;
    //             2'b11: // 强烈跳转
    //                 state <= Pre_Wrong ? 2'b10 : 2'b11;
    //         endcase
    //     end
    // end

    //静态预测
    wire predict_ctrl;
    assign predict_ctrl = instr_F[31];

    // 基于BTB和方向预测器进行预测
    // 如果BTB命中，使用方向预测器判断是否跳转
    wire predict_D, predict_E;
    wire predict_F;
    assign predict_F = btb_hit && predict_ctrl;

    assign Pre_Wrong = predict_E ^ Jump_sign;

    // 正常路径：预测正确时的PC选择
    // 当BTB命中且方向预测为跳转时，使用BTB预测的目标地址
    assign PC_predict_path = (btb_hit && predict_ctrl) ? btb_target_addr : snpc;

    // 修正路径：预测错误时的PC选择
    wire [31:0] PC_next_E;
    assign PC_next_E = PC_reg_E + 32'd4; // 执行阶段PC+4
    assign PC_correction_path = Jump_sign ? (jalr_E ? PC_jalr : PC_jump) : PC_next_E;

    // 最终PC选择
    always@(*) begin
        PC_src = Pre_Wrong_valid ? PC_correction_path : PC_predict_path;
    end
`else
    assign PC_src=(Jump_sign)?((jalr_E)?PC_jalr:PC_jump):snpc;
    assign Pre_Wrong=Jump_sign; //不使用分支预测
`endif


    //-----------------EX stage----------------
    assign ALU_DA=(reg_ren_E)?((auipc_E)? PC_reg_E:src1):32'b0;
    assign ALU_DB=(ALU_DB_Src_E)?src2:imme_E;
    ALU ALU1(
            .ALU_DA(ALU_DA),
            .ALU_DB(ALU_DB),
            .ALU_CTL(ALUControl_E),
            .ALU_ZERO(ALU_ZERO),
            .ALU_OverFlow(ALU_OverFlow),
            `ifdef RV32M
            .mulsign(mulsign_E), 
            `endif
            .ALU_DC(ALUResult_E_RAW)

        );

    always @(*) begin
        case(ResultSrc_E)
            2'b00:
                ALUResult_E=ALUResult_E_RAW;
            2'b10:
                ALUResult_E=PC_reg_E+32'd4;
            2'b11:
                ALUResult_E=imme_E;
            default:
                ALUResult_E=ALUResult_E_RAW;
        endcase
    end

    wire beq,bne,blt,bge,bltu,bgeu;
    assign beq=(opcode_E==`B_type)&&(funct3_E==3'b000);
    assign bne=(opcode_E==`B_type)&&(funct3_E==3'b001);
    assign blt=(opcode_E==`B_type)&&(funct3_E==3'b100);
    assign bge=(opcode_E==`B_type)&&(funct3_E==3'b101);
    assign bltu=(opcode_E==`B_type)&&(funct3_E==3'b110);
    assign bgeu=(opcode_E==`B_type)&&(funct3_E==3'b111);

    wire bne_true,beq_true,blt_true,bge_true,bltu_true,bgeu_true;

    assign beq_true=(beq & ALU_ZERO);
    assign bne_true=(bne & ~ALU_ZERO);
    assign blt_true=(blt & ALUResult_E[0]);
    assign bge_true=(bge & ~ALUResult_E[0]);
    assign bltu_true=(bltu & ALUResult_E[0]);
    assign bgeu_true=(bgeu & ~ALUResult_E[0]);
    wire branch_true;
    assign branch_true=(beq_true | bne_true | blt_true | bge_true | bltu_true | bgeu_true);

    //-----------------load store stage----------------
    reg [31:0] rdata_M;
    wire [31:0] mem_data_out = rdata2_M;

    always @(*) begin
        case(funct3_M)
            3'b000:
                rdata_M={{24{ReadData_M[7]}},ReadData_M[7:0]};
            3'b001:
                rdata_M={{16{ReadData_M[15]}},ReadData_M[15:0]};
            3'b010:
                rdata_M=ReadData_M;
            3'b100:
                rdata_M={24'b0,ReadData_M[7:0]};
            3'b101:
                rdata_M={16'b0,ReadData_M[15:0]};
            default:
                rdata_M=ReadData_M;
        endcase
    end

    reg [3:0] wmask;
    always @(*) begin
        case(funct3_M[1:0])
            2'b00:
                wmask=4'h1;
            2'b01:
                wmask=4'h3;
            2'b10:
                wmask=4'hF;
            default:
                wmask=4'h0;
        endcase
    end


    //-----------------Write Back stage----------------
    //-----------------------------------------

    reg  [31:0] wdata;
    assign ebreak=ebreak_W;
    assign PC_W=PC_reg_W;
    RegisterFile u_RegisterFile(


                     .clk    	(clk     ),
                     .rst    	(rst     ),
                     .wdata  	(wdata   ),
                     .waddr  	(Rd_W   ),
                     .wen    	(RegWrite_W     ),
                     .raddr1 	(Rs1_D  ),
                     .raddr2 	(Rs2_D  ),
                     .rdata1 	(rdata1_D  ),
                     .rdata2 	(rdata2_D  ),
                     .ren    	(reg_ren_D    )
                 );


    always @(*) begin
        case(ResultSrc_W)
            2'b01:
                wdata=rdata_W;
            default:
                wdata=ALUResult_W;
        endcase
    end




endmodule
