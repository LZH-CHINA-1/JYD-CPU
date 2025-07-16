// Branch Target Buffer
// 用于建立PC值与跳转指令及其跳转地址之间的映射关系
module BTB(
        input wire clk,
        input wire rst,

        /* 时序逻辑， 更新buffer */
        input wire valid_in, // 是否有新的跳转指令
        input wire [31:0] branch_PC, // 跳转指令的PC值
        input wire [31:0] branch_target, // 跳转指令的目标地址

        /* 组合逻辑 */
        input wire [31:0] PC_in, // 需要判断的PC值
        output wire hit, // 是否命中跳转
        output wire [31:0] target_addr // 跳转目标地址
    );

    // BTB参数定义
    parameter BTB_SIZE = 128;  // BTB表大小，必须是2的幂
    parameter INDEX_WIDTH = $clog2(BTB_SIZE);  // 索引位宽
    parameter TAG_WIDTH = 32 - INDEX_WIDTH - 2;  // 标签位宽 (PC - 索引位 - 字节偏移)

    // BTB表项定义
    reg [TAG_WIDTH-1:0] btb_tags [BTB_SIZE-1:0];  // 标签数组
    reg [31:0] btb_targets [BTB_SIZE-1:0];        // 目标地址数组
    reg btb_valid [BTB_SIZE-1:0];                 // 有效位数组

    // Wire信号声明
    wire [INDEX_WIDTH-1:0] lookup_index;
    wire [TAG_WIDTH-1:0] lookup_tag;
    wire [INDEX_WIDTH-1:0] update_index;
    wire [TAG_WIDTH-1:0] update_tag;
    wire tag_match;
    wire entry_valid;

    // Wire信号赋值
    // BTB索引计算 (使用PC的低位，忽略字节偏移)
    assign lookup_index = PC_in[INDEX_WIDTH+1:2];
    assign lookup_tag = PC_in[31:INDEX_WIDTH+2];

    assign update_index = branch_PC[INDEX_WIDTH+1:2];
    assign update_tag = branch_PC[31:INDEX_WIDTH+2];

    // 查找逻辑 - 判断当前PC是否命中BTB中的跳转指令
    assign tag_match = (btb_tags[lookup_index] == lookup_tag);
    assign entry_valid = btb_valid[lookup_index];

    // 输出赋值
    assign hit = tag_match && entry_valid;
    assign target_addr = (hit) ? btb_targets[lookup_index] : 32'b0;

    // BTB更新逻辑
    integer i;
    always @(posedge clk) begin
        if (rst) begin
            // 复位时清空BTB表
            for (i = 0; i < BTB_SIZE; i = i + 1) begin
                btb_valid[i] <= 1'b0;
                btb_tags[i] <= {TAG_WIDTH{1'b0}};
                btb_targets[i] <= 32'b0;
            end
        end
        else if (valid_in) begin
            // 有新的跳转指令时，更新BTB表
            // $display("Updating BTB: PC = %h, Target = %h", branch_PC, branch_target);
            btb_valid[update_index] <= 1'b1;
            btb_tags[update_index] <= update_tag;
            btb_targets[update_index] <= branch_target;
        end
    end

    always @(hit) begin
        if(hit) begin
            // $display("BTB Hit: PC = %h, Target = %h", PC_in, target_addr);
        end
    end

    // 未使用信号位注释
    // PC_in[1:0] 和 branch_PC[1:0] 未使用是正常的，因为是字节偏移
    /* verilator lint_off UNUSEDSIGNAL */

endmodule
