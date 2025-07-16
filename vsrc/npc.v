`include "define.v"
`include "pipeline_config.v"
`timescale 1ns / 1ps
`ifdef SIMULATION
module npc(
    `else
module myCPU(
    `endif
`ifdef SIMULATION
        input clk,
        input rst
`else
        input wire cpu_clk,
        input wire cpu_rst,

        // Interface to IROM
        output wire [31:0] irom_addr,
        input wire [31:0] irom_data,

        // Interface to DRAM & peripheral
        output wire [31:0] perip_addr,
        output wire perip_wen,
        output wire [1:0] perip_mask,
        output wire [31:0] perip_wdata,
        input wire [31:0] perip_rdata
`endif
    );

        wire [31:0] ALU_DC;
    wire [31:0] instr;
    wire [31:0] PC_reg;
    wire [31:0] mem_data_in;
    wire [31:0] mem_data_out;
    wire [31:0] mem_addr;
    wire mem_wen;
    wire mem_ren;
    wire [31:0] PC_reg_WB; // for test
    wire [7:0] wmask;
`ifndef SIMULATION
    wire rst;
    wire clk;
    assign clk = cpu_clk;
    assign rst = cpu_rst;
    assign irom_addr = PC_reg;

    assign instr = irom_data;
    assign perip_addr = mem_addr;
    assign perip_wen = mem_wen;
    assign perip_wdata = mem_data_out;

    assign mem_data_in = perip_rdata;

     wire ReadData_M_valid; // 增加该信号
    datapath_wrapper datapath1(
        .clk(clk),
        .rst(rst),
        .instr_F(instr),
        .ReadData_M(mem_data_in),
        .mem_data_out(mem_data_out),
        .mem_addr(mem_addr),
        .MemWrite_M(mem_wen),
        .MemRead_M(mem_ren),
        .ALUResult_E(ALU_DC),
        .PC_reg_F(PC_reg),
        .wmask(perip_mask),
        .ReadData_M_valid(ReadData_M_valid) // 增加该信号
    );
    reg ReadData_M_valid_reg;
    always @(posedge clk) begin
        if(rst) begin
            ReadData_M_valid_reg <= 1'b0;
        end else begin
            ReadData_M_valid_reg <= mem_ren & (~ReadData_M_valid);
        end
    end
    assign ReadData_M_valid = ReadData_M_valid_reg;

`endif


   
    // output declaration of module memory
`ifdef SIMULATION
 wire stop_sim;

    datapath datapath1(
                 .clk(clk),
                 .rst(rst),

                 // AXI4-Lite 指令接口
                 .inst_axi_araddr(inst_axi_araddr),
                 .inst_axi_arvalid(inst_axi_arvalid),
                 .inst_axi_arready(inst_axi_arready),
                 .inst_axi_rdata(inst_axi_rdata),
                 .inst_axi_rresp(inst_axi_rresp),
                 .inst_axi_rvalid(inst_axi_rvalid),
                 .inst_axi_rready(inst_axi_rready),

                 // AXI4-Lite 数据接口
                 .data_axi_araddr(data_axi_araddr),
                 .data_axi_arvalid(data_axi_arvalid),
                 .data_axi_arready(data_axi_arready),
                 .data_axi_rdata(data_axi_rdata),
                 .data_axi_rresp(data_axi_rresp),
                 .data_axi_rvalid(data_axi_rvalid),
                 .data_axi_rready(data_axi_rready),
                 .data_axi_awaddr(data_axi_awaddr),
                 .data_axi_awvalid(data_axi_awvalid),
                 .data_axi_awready(data_axi_awready),
                 .data_axi_wdata(data_axi_wdata),
                 .data_axi_wstrb(data_axi_wstrb),
                 .data_axi_wvalid(data_axi_wvalid),
                 .data_axi_wready(data_axi_wready),
                 .data_axi_bresp(data_axi_bresp),
                 .data_axi_bvalid(data_axi_bvalid),
                 .data_axi_bready(data_axi_bready),
                 .PC_W(PC_W),
                 .valid_W_out(valid_W),
                 .ebreak(stop_sim)
             );
 wire [31:0] inst_axi_araddr;
    wire inst_axi_arvalid;
    wire inst_axi_arready;
    wire [31:0] inst_axi_rdata;
    wire [1:0] inst_axi_rresp;
    wire inst_axi_rvalid;
    wire inst_axi_rready;

    wire [31:0] data_axi_araddr;
    wire data_axi_arvalid;
    wire data_axi_arready;
    wire [31:0] data_axi_rdata;
    wire [1:0] data_axi_rresp;
    wire data_axi_rvalid;
    wire data_axi_rready;
    wire [31:0] data_axi_awaddr;
    wire data_axi_awvalid;
    wire data_axi_awready;
    wire [31:0] data_axi_wdata;
    wire [3:0] data_axi_wstrb;
    wire data_axi_wvalid;
    wire data_axi_wready;
    wire [1:0] data_axi_bresp;
    wire data_axi_bvalid;
    wire data_axi_bready;

    // 模拟AXI响应信号
    assign inst_axi_arready = 1'b1;
    assign inst_axi_rvalid = inst_axi_arvalid;
    assign inst_axi_rresp = 2'b00;
    assign data_axi_arready = 1'b1;
    assign data_axi_rvalid = data_axi_rvalid_reg;
    assign data_axi_rresp = 2'b00;
    assign data_axi_awready = 1'b1;
    assign data_axi_wready = 1'b1;
    assign data_axi_bvalid = 1'b1;
    assign data_axi_bresp = 2'b00;
    memory #(.IS_IF(0)) u_memory(
               .raddr 	(data_axi_araddr  ),     // 使用AXI数据地址
               .waddr 	(data_axi_awaddr  ),     // 使用AXI写地址
               .wdata 	(data_axi_wdata   ),     // 使用AXI写数据
               .wmask 	({4'h0, data_axi_wstrb}  ),
               .wen   	(data_axi_awvalid ),     // 使用AXI写有效信号
               .valid 	(data_axi_arvalid | data_axi_awvalid ), // 读或写有效
               .rdata 	(data_axi_rdata  )
           );
    
    reg data_axi_rvalid_reg;
    always @(posedge clk) begin
        if(rst) begin
            data_axi_rvalid_reg <= 0;
        end else begin
            data_axi_rvalid_reg <= data_axi_arvalid; //延迟一个周期读出数据
        end
    end

    memory #(.IS_IF(1)) u_instr(
               .raddr 	(inst_axi_araddr  ),     // 使用AXI指令地址
               .waddr 	(32'b0  ),               // 指令内存不写
               .wdata 	(32'b0  ),
               .wmask 	(8'h00  ),               // 指令内存不写
               .wen   	(1'b0    ),
               .valid 	(inst_axi_arvalid ),     // 使用AXI指令读有效
               .rdata 	(inst_axi_rdata  ) 
           );

    wire [31:0] PC_W;
    wire valid_W;
    export "DPI-C" function get_pc_inst;
               function void get_pc_inst();
                   output int cpu_pc;
                   output int cpu_inst;
                   cpu_pc = PC_W;
                   cpu_inst = inst_axi_rdata;
               endfunction
    export "DPI-C" function get_validW;
                function void get_validW();
                     output byte validW;
                     validW = {7'b0,valid_W};
                endfunction

    import "DPI-C" function void ebreak();
                always @ (posedge clk) begin
                    if(stop_sim) begin
                        $display("EBREAK triggered at PC: %h", PC_W);
                        ebreak();
                    end
                end
`endif

endmodule

`ifndef SIMULATION
module datapath_wrapper(
    input clk,
    input rst,
    input [31:0] instr_F,
    input [31:0] ReadData_M,
    input ReadData_M_valid, // 增加该信号
    output [31:0] mem_data_out,
    output [31:0] mem_addr,
    output MemWrite_M,
    output MemRead_M,
    output reg [1:0] wmask,
    output reg [31:0] ALUResult_E,
    output [31:0] PC_reg_F
);

    wire [3:0] data_axi_wstrb;
    datapath u_datapath(
        .clk              	(clk               ),
        .rst              	(rst               ),
        .inst_axi_araddr  	(PC_reg_F   ),
        .inst_axi_arvalid 	(  ),
        .inst_axi_arready 	( 1'b1 ),
        .inst_axi_rdata   	(instr_F    ),
        .inst_axi_rresp   	( 2'b00   ),
        .inst_axi_rvalid  	(1'b1   ),
        .inst_axi_rready  	(   ),

        .data_axi_araddr  	(mem_addr   ),
        .data_axi_arvalid 	(MemRead_M  ),
        .data_axi_arready 	(1'b1  ),
        .data_axi_rdata   	(ReadData_M    ),
        .data_axi_rresp   	(2'b00    ),
        .data_axi_rvalid  	(ReadData_M_valid   ),
        .data_axi_rready  	(   ),
        .data_axi_awaddr  	(mem_addr   ),
        .data_axi_awvalid 	(MemWrite_M  ),
        .data_axi_awready 	(1'b1  ),
        .data_axi_wdata   	(mem_data_out    ),
        .data_axi_wstrb   	(data_axi_wstrb    ),
        .data_axi_wready  	(1'b1   ),
        .data_axi_bresp   	(2'b00    ),
        .data_axi_bvalid  	(1'b1   ),
        .data_axi_bready  	(   )
    );
    always @(*) begin
        case (data_axi_wstrb)
            4'b0001: wmask = 2'b00; // Byte
            4'b0011: wmask = 2'b01; // Half-word
            4'b1111: wmask = 2'b10; // Word
            default: wmask = 2'b11; // No write
        endcase
    end


endmodule
`endif