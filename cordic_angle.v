`timescale 1ns/100ps
module cordic_angle
#(
parameter C_DIN_WIDTH       = 16 ,
parameter C_ANGLE_WIDTH     = 16 ,
parameter C_ITERATION_TIMES = 16 ,
parameter C_PIPELINE        = 0
)
(
input                             I_clk      ,
input      [C_DIN_WIDTH-1:0]      I_data_i   ,
input      [C_DIN_WIDTH-1:0]      I_data_q   ,
input                             I_data_v   ,
output reg [C_ANGLE_WIDTH-1:0]    O_angle    ,
output reg                        O_angle_v
);

localparam C_SHIFT_WIDTH = F_cal_width(C_DIN_WIDTH);
localparam C_ITER_WIDTH = F_cal_width(C_ITERATION_TIMES);

integer S_i;
wire [C_ANGLE_WIDTH-1:0] S_pi_pos = 3.141592653589793 * (2**(C_ANGLE_WIDTH-3));
wire [C_ANGLE_WIDTH-1:0] S_pi_neg = -3.141592653589793 * (2**(C_ANGLE_WIDTH-3));
reg [C_DIN_WIDTH-1:0] S_data_merge = 0;
reg [C_SHIFT_WIDTH-1:0] S_msb = 0;
reg [C_DIN_WIDTH-1:0] S_data_i_inv = 0;
reg [C_DIN_WIDTH-1:0] S_data_i = 0;
reg [C_DIN_WIDTH-1:0] S_data_q = 0;
reg [C_DIN_WIDTH-1:0] S_data_i_d = 0;
reg [C_DIN_WIDTH-1:0] S_data_q_d = 0;
reg [C_ANGLE_WIDTH-1:0] S_pi = 0;
//pipeline
wire [C_ANGLE_WIDTH-1:0] S_atan_step_pipeline [31:0];
reg [C_DIN_WIDTH-1:0] S_data_i_shift [C_ITERATION_TIMES:0];
reg [C_DIN_WIDTH-1:0] S_data_q_shift [C_ITERATION_TIMES:0];
reg [C_ANGLE_WIDTH-1:0] S_angle_pipeline [C_ITERATION_TIMES:0];
reg [C_ITERATION_TIMES+2:0] S_data_v_slr = 0;
reg [C_ITERATION_TIMES*2+5:0] S_quad_slr = 0;
//non-pipeline
wire [C_ANGLE_WIDTH*32-1:0] S_atan_step;
reg [1:0] S_quad = 0;
reg S_iter_v = 0;
reg [C_ITER_WIDTH-1:0] S_iter_cnt = 0;
reg [C_ITER_WIDTH-1:0] S_iter_cnt_d = 0;
reg S_iter_over = 0;
reg S_data_v = 0;
reg S_data_v_d = 0;
reg signed [C_DIN_WIDTH-1:0] S_data_i_iter = 0;
reg signed [C_DIN_WIDTH-1:0] S_data_q_iter = 0;
reg [C_ANGLE_WIDTH*32-1:0] S_atan_step_slr;
reg [C_ANGLE_WIDTH-1:0] S_angle = 0;

always @(posedge I_clk)
begin
    if(I_data_i[C_DIN_WIDTH-1] && I_data_q[C_DIN_WIDTH-1])
        S_data_merge <= I_data_i & I_data_q;
    else if(I_data_i[C_DIN_WIDTH-1]==1'b0 && I_data_q[C_DIN_WIDTH-1]==1'b0)
        S_data_merge <= I_data_i | I_data_q;
    else if(I_data_i[C_DIN_WIDTH-1] ^ I_data_q[C_DIN_WIDTH-1])
        S_data_merge <= I_data_i ^ I_data_q;
    
    for(S_i=0;S_i<C_DIN_WIDTH-1;S_i=S_i+1)
        if(S_data_merge[C_DIN_WIDTH-1] != S_data_merge[S_i])
            S_msb <= C_DIN_WIDTH-3-S_i;
    S_data_i_inv <= ~I_data_i + 'd1;
    S_data_i <= I_data_i;
    S_data_q <= I_data_q;
    S_data_i_d <= S_data_i[C_DIN_WIDTH-1] ? S_data_i_inv : S_data_i;
    S_data_q_d <= S_data_q;
end

generate
if(C_PIPELINE==1)
begin:pipeline

always @(posedge I_clk)
begin
    S_pi <= S_quad_slr[C_ITERATION_TIMES*2+2] ? S_pi_neg : S_pi_pos;
    O_angle_v <= S_data_v_slr[C_ITERATION_TIMES+2];
    if(S_data_v_slr[C_ITERATION_TIMES+2])
    begin
        if(S_quad_slr[C_ITERATION_TIMES*2+5])
            O_angle <= S_pi - S_angle_pipeline[C_ITERATION_TIMES];
        else
            O_angle <= S_angle_pipeline[C_ITERATION_TIMES];
    end
end

end
else
begin:non_pipeline

always @(posedge I_clk)
begin
    S_pi <= S_quad[0] ? S_pi_neg : S_pi_pos;
    O_angle_v <= S_iter_over;
    if(S_iter_over)
    begin
        if(S_quad[1])
            O_angle <= S_pi - S_angle;
        else
            O_angle <= S_angle;
    end
end

end
endgenerate

//--------------------------------
//pipeline
//--------------------------------
assign S_atan_step_pipeline[0]  = 0.785398163397448 * (2**(C_ANGLE_WIDTH-3));
assign S_atan_step_pipeline[1]  = 0.463647609000806 * (2**(C_ANGLE_WIDTH-3));
assign S_atan_step_pipeline[2]  = 0.244978663126864 * (2**(C_ANGLE_WIDTH-3));
assign S_atan_step_pipeline[3]  = 0.124354994546761 * (2**(C_ANGLE_WIDTH-3));
assign S_atan_step_pipeline[4]  = 0.062418809995957 * (2**(C_ANGLE_WIDTH-3));
assign S_atan_step_pipeline[5]  = 0.031239833430268 * (2**(C_ANGLE_WIDTH-3));
assign S_atan_step_pipeline[6]  = 0.015623728620477 * (2**(C_ANGLE_WIDTH-3));
assign S_atan_step_pipeline[7]  = 0.007812341060101 * (2**(C_ANGLE_WIDTH-3));
assign S_atan_step_pipeline[8]  = 0.003906230131967 * (2**(C_ANGLE_WIDTH-3));
assign S_atan_step_pipeline[9]  = 0.001953122516479 * (2**(C_ANGLE_WIDTH-3));
assign S_atan_step_pipeline[10] = 0.000976562189559 * (2**(C_ANGLE_WIDTH-3));
assign S_atan_step_pipeline[11] = 0.000488281211195 * (2**(C_ANGLE_WIDTH-3));
assign S_atan_step_pipeline[12] = 0.000244140620149 * (2**(C_ANGLE_WIDTH-3));
assign S_atan_step_pipeline[13] = 0.000122070311894 * (2**(C_ANGLE_WIDTH-3));
assign S_atan_step_pipeline[14] = 0.000061035156174 * (2**(C_ANGLE_WIDTH-3));
assign S_atan_step_pipeline[15] = 0.000030517578116 * (2**(C_ANGLE_WIDTH-3));
assign S_atan_step_pipeline[16] = 0.000015258789061 * (2**(C_ANGLE_WIDTH-3));
assign S_atan_step_pipeline[17] = 0.000007629394531 * (2**(C_ANGLE_WIDTH-3));
assign S_atan_step_pipeline[18] = 0.000003814697266 * (2**(C_ANGLE_WIDTH-3));
assign S_atan_step_pipeline[19] = 0.000001907348633 * (2**(C_ANGLE_WIDTH-3));
assign S_atan_step_pipeline[20] = 0.000000953674316 * (2**(C_ANGLE_WIDTH-3));
assign S_atan_step_pipeline[21] = 0.000000476837158 * (2**(C_ANGLE_WIDTH-3));
assign S_atan_step_pipeline[22] = 0.000000238418579 * (2**(C_ANGLE_WIDTH-3));
assign S_atan_step_pipeline[23] = 0.000000119209290 * (2**(C_ANGLE_WIDTH-3));
assign S_atan_step_pipeline[24] = 0.000000059604645 * (2**(C_ANGLE_WIDTH-3));
assign S_atan_step_pipeline[25] = 0.000000029802322 * (2**(C_ANGLE_WIDTH-3));
assign S_atan_step_pipeline[26] = 0.000000014901161 * (2**(C_ANGLE_WIDTH-3));
assign S_atan_step_pipeline[27] = 0.000000007450581 * (2**(C_ANGLE_WIDTH-3));
assign S_atan_step_pipeline[28] = 0.000000003725290 * (2**(C_ANGLE_WIDTH-3));
assign S_atan_step_pipeline[29] = 0.000000001862645 * (2**(C_ANGLE_WIDTH-3));
assign S_atan_step_pipeline[30] = 0.000000000931323 * (2**(C_ANGLE_WIDTH-3));
assign S_atan_step_pipeline[31] = 0.000000000465661 * (2**(C_ANGLE_WIDTH-3));

genvar gen_i;
generate
for(gen_i=0;gen_i<C_ITERATION_TIMES;gen_i=gen_i+1)
begin:parallel_iteration

always @(posedge I_clk)
begin
    if(!S_data_q_shift[gen_i][C_DIN_WIDTH-1])
    begin
        S_data_i_shift[gen_i+1] <= S_data_i_shift[gen_i] + {{gen_i{S_data_q_shift[gen_i][C_DIN_WIDTH-1]}},S_data_q_shift[gen_i][C_DIN_WIDTH-1:gen_i]};
        S_data_q_shift[gen_i+1] <= S_data_q_shift[gen_i] - {{gen_i{S_data_i_shift[gen_i][C_DIN_WIDTH-1]}},S_data_i_shift[gen_i][C_DIN_WIDTH-1:gen_i]};
        S_angle_pipeline[gen_i+1] <= S_angle_pipeline[gen_i] + S_atan_step_pipeline[gen_i];
    end
    else
    begin
        S_data_i_shift[gen_i+1] <= S_data_i_shift[gen_i] - {{gen_i{S_data_q_shift[gen_i][C_DIN_WIDTH-1]}},S_data_q_shift[gen_i][C_DIN_WIDTH-1:gen_i]};
        S_data_q_shift[gen_i+1] <= S_data_q_shift[gen_i] + {{gen_i{S_data_i_shift[gen_i][C_DIN_WIDTH-1]}},S_data_i_shift[gen_i][C_DIN_WIDTH-1:gen_i]};
        S_angle_pipeline[gen_i+1] <= S_angle_pipeline[gen_i] - S_atan_step_pipeline[gen_i];
    end
end

end
endgenerate

always @(posedge I_clk)
begin
    S_data_i_shift[0] <= S_data_i_d << S_msb;
    S_data_q_shift[0] <= S_data_q_d << S_msb;
    S_angle_pipeline[0] <= 'd0;
    S_quad_slr <= {S_quad_slr[C_ITERATION_TIMES*2+3:0],I_data_i[C_DIN_WIDTH-1],I_data_q[C_DIN_WIDTH-1]};
    S_data_v_slr <= {S_data_v_slr[C_ITERATION_TIMES+1:0],I_data_v};
end

//-------------------------------
//non-pipeline
//-------------------------------
assign S_atan_step[C_ANGLE_WIDTH*0 +:C_ANGLE_WIDTH]  = 0.785398163397448 * (2**(C_ANGLE_WIDTH-3));
assign S_atan_step[C_ANGLE_WIDTH*1 +:C_ANGLE_WIDTH]  = 0.463647609000806 * (2**(C_ANGLE_WIDTH-3));
assign S_atan_step[C_ANGLE_WIDTH*2 +:C_ANGLE_WIDTH]  = 0.244978663126864 * (2**(C_ANGLE_WIDTH-3));
assign S_atan_step[C_ANGLE_WIDTH*3 +:C_ANGLE_WIDTH]  = 0.124354994546761 * (2**(C_ANGLE_WIDTH-3));
assign S_atan_step[C_ANGLE_WIDTH*4 +:C_ANGLE_WIDTH]  = 0.062418809995957 * (2**(C_ANGLE_WIDTH-3));
assign S_atan_step[C_ANGLE_WIDTH*5 +:C_ANGLE_WIDTH]  = 0.031239833430268 * (2**(C_ANGLE_WIDTH-3));
assign S_atan_step[C_ANGLE_WIDTH*6 +:C_ANGLE_WIDTH]  = 0.015623728620477 * (2**(C_ANGLE_WIDTH-3));
assign S_atan_step[C_ANGLE_WIDTH*7 +:C_ANGLE_WIDTH]  = 0.007812341060101 * (2**(C_ANGLE_WIDTH-3));
assign S_atan_step[C_ANGLE_WIDTH*8 +:C_ANGLE_WIDTH]  = 0.003906230131967 * (2**(C_ANGLE_WIDTH-3));
assign S_atan_step[C_ANGLE_WIDTH*9 +:C_ANGLE_WIDTH]  = 0.001953122516479 * (2**(C_ANGLE_WIDTH-3));
assign S_atan_step[C_ANGLE_WIDTH*10+:C_ANGLE_WIDTH] = 0.000976562189559 * (2**(C_ANGLE_WIDTH-3));
assign S_atan_step[C_ANGLE_WIDTH*11+:C_ANGLE_WIDTH] = 0.000488281211195 * (2**(C_ANGLE_WIDTH-3));
assign S_atan_step[C_ANGLE_WIDTH*12+:C_ANGLE_WIDTH] = 0.000244140620149 * (2**(C_ANGLE_WIDTH-3));
assign S_atan_step[C_ANGLE_WIDTH*13+:C_ANGLE_WIDTH] = 0.000122070311894 * (2**(C_ANGLE_WIDTH-3));
assign S_atan_step[C_ANGLE_WIDTH*14+:C_ANGLE_WIDTH] = 0.000061035156174 * (2**(C_ANGLE_WIDTH-3));
assign S_atan_step[C_ANGLE_WIDTH*15+:C_ANGLE_WIDTH] = 0.000030517578116 * (2**(C_ANGLE_WIDTH-3));
assign S_atan_step[C_ANGLE_WIDTH*16+:C_ANGLE_WIDTH] = 0.000015258789061 * (2**(C_ANGLE_WIDTH-3));
assign S_atan_step[C_ANGLE_WIDTH*17+:C_ANGLE_WIDTH] = 0.000007629394531 * (2**(C_ANGLE_WIDTH-3));
assign S_atan_step[C_ANGLE_WIDTH*18+:C_ANGLE_WIDTH] = 0.000003814697266 * (2**(C_ANGLE_WIDTH-3));
assign S_atan_step[C_ANGLE_WIDTH*19+:C_ANGLE_WIDTH] = 0.000001907348633 * (2**(C_ANGLE_WIDTH-3));
assign S_atan_step[C_ANGLE_WIDTH*20+:C_ANGLE_WIDTH] = 0.000000953674316 * (2**(C_ANGLE_WIDTH-3));
assign S_atan_step[C_ANGLE_WIDTH*21+:C_ANGLE_WIDTH] = 0.000000476837158 * (2**(C_ANGLE_WIDTH-3));
assign S_atan_step[C_ANGLE_WIDTH*22+:C_ANGLE_WIDTH] = 0.000000238418579 * (2**(C_ANGLE_WIDTH-3));
assign S_atan_step[C_ANGLE_WIDTH*23+:C_ANGLE_WIDTH] = 0.000000119209290 * (2**(C_ANGLE_WIDTH-3));
assign S_atan_step[C_ANGLE_WIDTH*24+:C_ANGLE_WIDTH] = 0.000000059604645 * (2**(C_ANGLE_WIDTH-3));
assign S_atan_step[C_ANGLE_WIDTH*25+:C_ANGLE_WIDTH] = 0.000000029802322 * (2**(C_ANGLE_WIDTH-3));
assign S_atan_step[C_ANGLE_WIDTH*26+:C_ANGLE_WIDTH] = 0.000000014901161 * (2**(C_ANGLE_WIDTH-3));
assign S_atan_step[C_ANGLE_WIDTH*27+:C_ANGLE_WIDTH] = 0.000000007450581 * (2**(C_ANGLE_WIDTH-3));
assign S_atan_step[C_ANGLE_WIDTH*28+:C_ANGLE_WIDTH] = 0.000000003725290 * (2**(C_ANGLE_WIDTH-3));
assign S_atan_step[C_ANGLE_WIDTH*29+:C_ANGLE_WIDTH] = 0.000000001862645 * (2**(C_ANGLE_WIDTH-3));
assign S_atan_step[C_ANGLE_WIDTH*30+:C_ANGLE_WIDTH] = 0.000000000931323 * (2**(C_ANGLE_WIDTH-3));
assign S_atan_step[C_ANGLE_WIDTH*31+:C_ANGLE_WIDTH] = 0.000000000465661 * (2**(C_ANGLE_WIDTH-3));

always @(posedge I_clk)
begin
    S_data_v   <= I_data_v;
    S_data_v_d <= S_data_v;
    if(I_data_v)
    begin
        S_quad <= {I_data_i[C_DIN_WIDTH-1],I_data_q[C_DIN_WIDTH-1]};
    end
    
    if(S_data_v)
        S_iter_v <= 'd1;
    else if(S_iter_cnt == C_ITERATION_TIMES)
        S_iter_v <= 'd0;
    
    if(S_iter_v)    
        S_iter_cnt <= S_iter_cnt + 'd1;
    else
        S_iter_cnt <= 'd0;
    S_iter_over <= S_iter_cnt == C_ITERATION_TIMES;
    S_iter_cnt_d <= S_iter_cnt;
end

always @(posedge I_clk)
begin
    if(S_data_v_d)
    begin
        S_data_i_iter <= S_data_i_d << S_msb;
        S_data_q_iter <= S_data_q_d << S_msb;
        S_angle <= 'd0;
        S_atan_step_slr <= S_atan_step;
    end
    else
    begin
        if(!S_data_q_iter[C_DIN_WIDTH-1])
        begin
            S_data_i_iter <= S_data_i_iter + (S_data_q_iter>>>S_iter_cnt_d);
            S_data_q_iter <= S_data_q_iter - (S_data_i_iter>>>S_iter_cnt_d);
            S_angle <= S_angle + S_atan_step_slr[C_ANGLE_WIDTH-1:0];
        end
        else
        begin
            S_data_i_iter <= S_data_i_iter - (S_data_q_iter>>>S_iter_cnt_d);
            S_data_q_iter <= S_data_q_iter + (S_data_i_iter>>>S_iter_cnt_d);
            S_angle <= S_angle - S_atan_step_slr[C_ANGLE_WIDTH-1:0];
        end
        S_atan_step_slr <= S_atan_step_slr >> C_ANGLE_WIDTH;
    end
end


function integer F_cal_width;
input integer I_data;
integer i;
begin
    for(i=1;(2**i)<=I_data;i=i+1)
    F_cal_width = i;
    F_cal_width = i;
end
endfunction

endmodule


