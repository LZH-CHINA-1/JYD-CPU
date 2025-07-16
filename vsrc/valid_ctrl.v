`include "pipeline_config.v"
module valid_ctrl(
        input          clk,
        input          rst,

        input rvalid, // AXI4-LITE READ CHANNEL
        input load_M,
        input load_E,
        input stall,
        input Pre_Wrong, // 分支预测错误

        output valid_F,
        output valid_D,
        output valid_E,
        output valid_M,
        output valid_W,
        output ready_F,
        output ready_D,
        output ready_E,
        output ready_M,
        output ready_W

    );
    reg valid_F_reg, valid_D_reg, valid_E_reg, valid_M_reg, valid_W_reg;

    always @(posedge clk) begin
        if (rst) begin
            valid_D_reg <= 1'b0;
        end else begin
            if (Pre_Wrong & valid_E) begin
                valid_D_reg <= 1'b0;
            end else if(ready_D || (stall & valid_D_reg)) begin
                valid_D_reg <= valid_F; 
            end else if(ready_E) begin
                valid_D_reg <= 1'b0; // 如果D级未ready，取消valid
            end
        end
    end

    always @(posedge clk) begin
        if (rst) begin
            valid_E_reg <= 1'b0;
        end else begin
            if(Pre_Wrong & valid_E & ready_M) begin
                valid_E_reg <= 1'b0;
            end else if (ready_E) begin
                valid_E_reg <= valid_D;
            end else if(ready_M) begin
                valid_E_reg <= 1'b0; // 如果E级未ready，取消valid
            end
        end

    end

    reg [1:0] M_state;
    parameter M_IDLE = 2'b00, M_LOAD = 2'b01, M_READY = 2'b10;
    always @(posedge clk) begin
        if (rst) begin
            M_state <= M_IDLE;
        end else begin
            case (M_state)
                M_IDLE: begin
                    case ({valid_E, load_E})
                        2'b10: M_state <= M_READY;
                        2'b11: M_state <= M_LOAD;
                        default: M_state <= M_IDLE;
                    endcase
                end
                M_LOAD: begin
                    case ({rvalid, ready_W, valid_E, load_E})
                        4'b1111: M_state <= M_LOAD;
                        4'b1110: M_state <= M_READY;
                        4'b1101, 4'b1100: M_state <= M_IDLE;
                        4'b1000, 4'b1001, 4'b1010, 4'b1011: M_state <= M_READY;
                        default: M_state <= M_LOAD; // 其他情况保持
                    endcase
                end
                M_READY: begin
                    case ({ready_W, valid_E, load_E})
                        3'b111: M_state <= M_LOAD;
                        3'b110: M_state <= M_READY;
                        3'b101, 3'b100: M_state <= M_IDLE;
                        default: M_state <= M_READY; 
                    endcase
                end
                default: M_state <= M_IDLE;
            endcase           
        end
    end

    always @(posedge clk) begin
        if (rst) begin
            valid_W_reg <= 1'b0;
        end else begin
            if(ready_W) begin
                valid_W_reg <= valid_M;
            end
        end
    end

    assign valid_F = ~rst;
    assign valid_D = valid_D_reg & (~stall);
    assign valid_E = valid_E_reg;
    assign valid_M = (M_state == M_READY) || ((M_state == M_LOAD) && rvalid);
    assign valid_W = valid_W_reg;

    assign ready_W = ~rst; // 总是能单周期写回
    assign ready_M = (M_state == M_IDLE) || (valid_M & ready_W);
    assign ready_E = ready_M;
    assign ready_D = (~stall) & ready_E;
    assign ready_F = ready_D; 

endmodule
