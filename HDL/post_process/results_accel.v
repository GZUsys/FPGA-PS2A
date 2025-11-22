`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 2025/10/15 15:40:38
// Design Name: 
// Module Name: results_accel
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////


module results_accel # (
    parameter FIFO_DATA_WIDTH = 32,
    parameter AXIS_DATA_WIDTH = 32,
    parameter FIFO_DEPTH = 4,
    parameter QEU_LEN = 8
)(
    input clk,
    input dc1_rst,
    input dc2_rst,
    //dc1
    input  [31:0]    curr_qeu_id1,
    input  [31:0]    curr_ref_id1,
    input  [31:0]    curr_minval1,
    input  [31:0]    curr_position1,
    input            results_val1,
    //dc2
    input  [31:0]    curr_qeu_id2,
    input  [31:0]    curr_ref_id2,
    input  [31:0]    curr_minval2,
    input  [31:0]    curr_position2,
    input            results_val2,

    input tx_results,


    input [31:0]ref_seg_number1,
    input [31:0]ref_seg_number2,


    // Output AXI Stream
    input  wire                             SINK_AXIS_clk,
    input  wire                             SINK_AXIS_rst,
    output wire                             SINK_AXIS_tuser,
    output wire                             SINK_AXIS_tvalid,
    input  wire                             SINK_AXIS_tready,
    output wire                             SINK_AXIS_tlast,
    output wire [AXIS_DATA_WIDTH - 1:0]     SINK_AXIS_tdata,
    output wire qeu_next,
    output [31:0] dbug_ref_seg_cnt1,
    output [31:0] dbug_ref_seg_cnt2,
    output [3:0] dbug_rs_cnt1,
    output [3:0] dbug_rs_cnt2,
    output [31:0] dbug_dc1_qid3,
    output [31:0] dbug_dc1_qid6,
    output [31:0] dbug_dc2_qid3,
    output [31:0] dbug_dc2_qid6,
    output [31:0] dbug_tx_qid3,
    output [31:0] dbug_tx_qid6,
    output wire w_sink_fifo_not_empty,
    output wire [$clog2(FIFO_DEPTH)-1:0] dbug_write_ptr,
    output wire [$clog2(FIFO_DEPTH)-1:0] dbug_read_ptr
    );


    reg [3:0]rs1_cnt;
    reg [3:0]rs2_cnt;

    reg tx_results_r;
    reg set_result_r;


    reg [31:0] results1_data[31:0];
    reg [31:0] results2_data[31:0];
    reg rs1_done_flag;
    reg rs2_done_flag;

    reg [31:0] tx_data [31:0];
    reg [31:0] tx_data_buf;
    reg tx_en;
    reg [5:0] tx_cnt;

    reg [31:0]ref_seg_cnt1;
    reg [31:0]ref_seg_cnt2;

    reg sink_fifo_stb;
    reg sink_fifo_last;
    reg [31:0]first_qid;

    integer k;

    wire w_sink_fifo_w_stb;
    wire [31:0]w_sink_fifo_w_data;
    wire rst;
    wire w_sink_fifo_r_stb;
    wire [31:0]w_sink_fifo_r_data;
    wire w_fifo_full;

    assign w_sink_fifo_w_stb = sink_fifo_stb;
    assign w_sink_fifo_w_data = tx_data_buf;
    assign w_sink_fifo_r_last = sink_fifo_last;
    assign rst = (dc1_rst | dc2_rst);

    assign qeu_next = (ref_seg_cnt1 == ref_seg_number1 && ref_seg_cnt2 == ref_seg_number2)?1'b1:1'b0;

    assign dbug_rs_cnt1 = rs1_cnt;
    assign dbug_rs_cnt2 = rs2_cnt;
    assign dbug_ref_seg_cnt1 = ref_seg_cnt1;
    assign dbug_ref_seg_cnt2 = ref_seg_cnt2;
    assign dbug_dc1_qid3 = results1_data[0];
    assign dbug_dc1_qid6 = results1_data[28];
    assign dbug_dc2_qid3 = results2_data[0];
    assign dbug_dc2_qid6 = first_qid;
    assign dbug_tx_qid3 = tx_data[0];
    assign dbug_tx_qid6 = {15'd0,rs2_done_flag};


always @(posedge clk) begin
    if(rst)
        tx_results_r <= 0;
    else if(tx_en)
        tx_results_r <= 0;
    else if(tx_results)
        tx_results_r <= 1;
end

always @(posedge clk) begin
    if(rst)
        set_result_r <= 0;
    else if(tx_en)
        set_result_r <= 0;
    else if(ref_seg_cnt1 == ref_seg_number1 && ref_seg_cnt2 == ref_seg_number2)
        set_result_r <= 1;
end 


always @(posedge clk) begin
    if(rst)
        rs1_cnt <= 0;
    else if(results_val1)begin
        if(rs1_cnt == QEU_LEN-1)
            rs1_cnt <= 0;
        else
            rs1_cnt <= rs1_cnt + 1'b1;
    end
end

always @(posedge clk) begin
    if(rst)
        ref_seg_cnt1 <= 0;
    else if(ref_seg_cnt1 == ref_seg_number1 && ref_seg_cnt2 == ref_seg_number2)
        ref_seg_cnt1 <= 0;
    else if(results_val1 == 1 && rs1_cnt == 7)
        ref_seg_cnt1 <= ref_seg_cnt1 + 1;
end

always @(posedge clk) begin
    if(rst)
        rs1_done_flag <= 0;
    else if(ref_seg_cnt1 == ref_seg_number1 && ref_seg_cnt2 == ref_seg_number2)
        rs1_done_flag <= 0;
    else if(results_val1 == 1 && rs1_cnt == 7)
        rs1_done_flag <= 1;
end

always @(posedge clk) begin
    if(rst)
        rs2_done_flag <= 0;
    else if(ref_seg_cnt1 == ref_seg_number1 && ref_seg_cnt2 == ref_seg_number2)
        rs2_done_flag <= 0;
    else if(results_val2 == 1 && rs2_cnt == 7)
        rs2_done_flag <= 1;
end

always @(posedge clk)begin
    if(rst)begin
        results1_data[0] <= 0;  results1_data[8] <= 0; results1_data[16] <= 0; results1_data[24] <= 0;
        results1_data[1] <= 0;  results1_data[9] <= 0; results1_data[17] <= 0; results1_data[25] <= 0;
        results1_data[2] <= 0;  results1_data[10] <= 0; results1_data[18] <= 0; results1_data[26] <= 0;
        results1_data[3] <= 0;  results1_data[11] <= 0; results1_data[19] <= 0; results1_data[27] <= 0;
        results1_data[4] <= 0;  results1_data[12] <= 0; results1_data[20] <= 0; results1_data[28] <= 0;
        results1_data[5] <= 0;  results1_data[13] <= 0; results1_data[21] <= 0; results1_data[29] <= 0;
        results1_data[6] <= 0;  results1_data[14] <= 0; results1_data[22] <= 0; results1_data[30] <= 0;
        results1_data[7] <= 0;  results1_data[15] <= 0; results1_data[23] <= 0; results1_data[31] <= 0;
    end
    
    else if(ref_seg_cnt1 == ref_seg_number1 && ref_seg_cnt2 == ref_seg_number2)begin
        results1_data[0] <= 0;  results1_data[8] <= 0; results1_data[16] <= 0; results1_data[24] <= 0;
        results1_data[1] <= 0;  results1_data[9] <= 0; results1_data[17] <= 0; results1_data[25] <= 0;
        results1_data[2] <= 0;  results1_data[10] <= 0; results1_data[18] <= 0; results1_data[26] <= 0;
        results1_data[3] <= 0;  results1_data[11] <= 0; results1_data[19] <= 0; results1_data[27] <= 0;
        results1_data[4] <= 0;  results1_data[12] <= 0; results1_data[20] <= 0; results1_data[28] <= 0;
        results1_data[5] <= 0;  results1_data[13] <= 0; results1_data[21] <= 0; results1_data[29] <= 0;
        results1_data[6] <= 0;  results1_data[14] <= 0; results1_data[22] <= 0; results1_data[30] <= 0;
        results1_data[7] <= 0;  results1_data[15] <= 0; results1_data[23] <= 0; results1_data[31] <= 0;
    end
    
    else if(results_val1)begin
        if(rs1_cnt == 0 && (results1_data[2] > curr_minval1 || rs1_done_flag == 0))begin
            results1_data[0] <= curr_qeu_id1;
            results1_data[1] <= curr_ref_id1;
            results1_data[2] <= curr_minval1;
            results1_data[3] <= curr_position1;

        end else if(rs1_cnt == 1 && (results1_data[6] > curr_minval1 || rs1_done_flag == 0))begin
            results1_data[4] <= curr_qeu_id1;
            results1_data[5] <= curr_ref_id1;
            results1_data[6] <= curr_minval1;
            results1_data[7] <= curr_position1;

        end else if(rs1_cnt == 2 && (results1_data[10] > curr_minval1 || rs1_done_flag == 0))begin
            results1_data[8] <= curr_qeu_id1;
            results1_data[9] <= curr_ref_id1;
            results1_data[10] <= curr_minval1;
            results1_data[11] <= curr_position1;

        end else if(rs1_cnt == 3 && (results1_data[14] > curr_minval1 || rs1_done_flag == 0))begin
            results1_data[12] <= curr_qeu_id1;
            results1_data[13] <= curr_ref_id1;
            results1_data[14] <= curr_minval1;
            results1_data[15] <= curr_position1;

        end else if(rs1_cnt == 4 && (results1_data[18] > curr_minval1 || rs1_done_flag == 0))begin
            results1_data[16] <= curr_qeu_id1;
            results1_data[17] <= curr_ref_id1;
            results1_data[18] <= curr_minval1;
            results1_data[19] <= curr_position1;

        end else if(rs1_cnt == 5 && (results1_data[22] > curr_minval1 || rs1_done_flag == 0))begin
            results1_data[20] <= curr_qeu_id1;
            results1_data[21] <= curr_ref_id1;
            results1_data[22] <= curr_minval1;
            results1_data[23] <= curr_position1;

        end else if(rs1_cnt == 6 && (results1_data[26] > curr_minval1 || rs1_done_flag == 0))begin
            results1_data[24] <= curr_qeu_id1;
            results1_data[25] <= curr_ref_id1;
            results1_data[26] <= curr_minval1;
            results1_data[27] <= curr_position1;

        end else if(rs1_cnt == 7 && (results1_data[30] > curr_minval1 || rs1_done_flag == 0))begin
            results1_data[28] <= curr_qeu_id1;
            results1_data[29] <= curr_ref_id1;
            results1_data[30] <= curr_minval1;
            results1_data[31] <= curr_position1;

        end
    end
end


//2
always @(posedge clk) begin
    if(rst)
        rs2_cnt <= 0;
    else if(results_val2)begin
        if(rs2_cnt == QEU_LEN-1)
            rs2_cnt <= 0;
        else
            rs2_cnt <= rs2_cnt + 1'b1;
    end
end

always @(posedge clk) begin
    if(rst)
        ref_seg_cnt2 <= 0;
    else if(ref_seg_cnt1 == ref_seg_number1 && ref_seg_cnt2 == ref_seg_number2)
        ref_seg_cnt2 <= 0;
    else if(results_val2 == 1 && rs2_cnt == 7)
        ref_seg_cnt2 <= ref_seg_cnt2 + 1;
end

always @(posedge clk)begin
    if(rst)
        first_qid <= 32'd0;
    else if((results_val2 == 1) && (rs2_cnt == 0) && (rs2_done_flag == 0))
        first_qid <= curr_qeu_id2;
end

always @(posedge clk)begin
    if(rst)begin
        results2_data[0] <= 0;  results2_data[8] <= 0; results2_data[16] <= 0; results2_data[24] <= 0;
        results2_data[1] <= 0;  results2_data[9] <= 0; results2_data[17] <= 0; results2_data[25] <= 0;
        results2_data[2] <= 0;  results2_data[10] <= 0; results2_data[18] <= 0; results2_data[26] <= 0;
        results2_data[3] <= 0;  results2_data[11] <= 0; results2_data[19] <= 0; results2_data[27] <= 0;
        results2_data[4] <= 0;  results2_data[12] <= 0; results2_data[20] <= 0; results2_data[28] <= 0;
        results2_data[5] <= 0;  results2_data[13] <= 0; results2_data[21] <= 0; results2_data[29] <= 0;
        results2_data[6] <= 0;  results2_data[14] <= 0; results2_data[22] <= 0; results2_data[30] <= 0;
        results2_data[7] <= 0;  results2_data[15] <= 0; results2_data[23] <= 0; results2_data[31] <= 0;
    end
    
    else if(ref_seg_cnt1 == ref_seg_number1 && ref_seg_cnt2 == ref_seg_number2)begin
        results2_data[0] <= 0;  results2_data[8] <= 0; results2_data[16] <= 0; results2_data[24] <= 0;
        results2_data[1] <= 0;  results2_data[9] <= 0; results2_data[17] <= 0; results2_data[25] <= 0;
        results2_data[2] <= 0;  results2_data[10] <= 0; results2_data[18] <= 0; results2_data[26] <= 0;
        results2_data[3] <= 0;  results2_data[11] <= 0; results2_data[19] <= 0; results2_data[27] <= 0;
        results2_data[4] <= 0;  results2_data[12] <= 0; results2_data[20] <= 0; results2_data[28] <= 0;
        results2_data[5] <= 0;  results2_data[13] <= 0; results2_data[21] <= 0; results2_data[29] <= 0;
        results2_data[6] <= 0;  results2_data[14] <= 0; results2_data[22] <= 0; results2_data[30] <= 0;
        results2_data[7] <= 0;  results2_data[15] <= 0; results2_data[23] <= 0; results2_data[31] <= 0;
    end
   
    else if(results_val2)begin
        if((rs2_cnt == 0) && ((results2_data[2] > curr_minval2) || (rs2_done_flag == 0)))begin
            results2_data[0] <= curr_qeu_id2;
            results2_data[1] <= curr_ref_id2;
            results2_data[2] <= curr_minval2;
            results2_data[3] <= curr_position2;

        end else if(rs2_cnt == 1 && (results2_data[6] > curr_minval2 || rs2_done_flag == 0))begin
            results2_data[4] <= curr_qeu_id2;
            results2_data[5] <= curr_ref_id2;
            results2_data[6] <= curr_minval2;
            results2_data[7] <= curr_position2;

        end else if(rs2_cnt == 2 && (results2_data[10] > curr_minval2 || rs2_done_flag == 0))begin
            results2_data[8] <= curr_qeu_id2;
            results2_data[9] <= curr_ref_id2;
            results2_data[10] <= curr_minval2;
            results2_data[11] <= curr_position2;

        end else if(rs2_cnt == 3 && (results2_data[14] > curr_minval2 || rs2_done_flag == 0))begin
            results2_data[12] <= curr_qeu_id2;
            results2_data[13] <= curr_ref_id2;
            results2_data[14] <= curr_minval2;
            results2_data[15] <= curr_position2;

        end else if(rs2_cnt == 4 && (results2_data[18] > curr_minval2 || rs2_done_flag == 0))begin
            results2_data[16] <= curr_qeu_id2;
            results2_data[17] <= curr_ref_id2;
            results2_data[18] <= curr_minval2;
            results2_data[19] <= curr_position2;

        end else if(rs2_cnt == 5 && (results2_data[22] > curr_minval2 || rs2_done_flag == 0))begin
            results2_data[20] <= curr_qeu_id2;
            results2_data[21] <= curr_ref_id2;
            results2_data[22] <= curr_minval2;
            results2_data[23] <= curr_position2;

        end else if(rs2_cnt == 6 && (results2_data[26] > curr_minval2 || rs2_done_flag == 0))begin
            results2_data[24] <= curr_qeu_id2;
            results2_data[25] <= curr_ref_id2;
            results2_data[26] <= curr_minval2;
            results2_data[27] <= curr_position2;

        end else if(rs2_cnt == 7 && (results2_data[30] > curr_minval2 || rs2_done_flag == 0))begin
            results2_data[28] <= curr_qeu_id2;
            results2_data[29] <= curr_ref_id2;
            results2_data[30] <= curr_minval2;
            results2_data[31] <= curr_position2;

        end
    end
end


always @(posedge clk) begin
    if(rst) begin
        for(k = 0; k <= 31; k = k + 1)begin
            tx_data[k] <= 0;
        end
    end else if(ref_seg_cnt1 == ref_seg_number1 && ref_seg_cnt2 == ref_seg_number2) begin
        if(results1_data[2] < results2_data[2])begin
            tx_data[0] <= results1_data[0];
            tx_data[1] <= results1_data[1];
            tx_data[2] <= results1_data[2];
            tx_data[3] <= results1_data[3];
        end else begin
            tx_data[0] <= results2_data[0];
            tx_data[1] <= results2_data[1];
            tx_data[2] <= results2_data[2];
            tx_data[3] <= results2_data[3];
        end

        if(results1_data[6] < results2_data[6])begin
            tx_data[4] <= results1_data[4];
            tx_data[5] <= results1_data[5];
            tx_data[6] <= results1_data[6];
            tx_data[7] <= results1_data[7];
        end else begin
            tx_data[4] <= results2_data[4];
            tx_data[5] <= results2_data[5];
            tx_data[6] <= results2_data[6];
            tx_data[7] <= results2_data[7];
        end

        if(results1_data[10] < results2_data[10])begin
            tx_data[8] <= results1_data[8];
            tx_data[9] <= results1_data[9];
            tx_data[10] <= results1_data[10];
            tx_data[11] <= results1_data[11];
        end else begin
            tx_data[8] <= results2_data[8];
            tx_data[9] <= results2_data[9];
            tx_data[10] <= results2_data[10];
            tx_data[11] <= results2_data[11];
        end

        if(results1_data[14] < results2_data[14])begin
            tx_data[12] <= results1_data[12];
            tx_data[13] <= results1_data[13];
            tx_data[14] <= results1_data[14];
            tx_data[15] <= results1_data[15];
        end else begin
            tx_data[12] <= results2_data[12];
            tx_data[13] <= results2_data[13];
            tx_data[14] <= results2_data[14];
            tx_data[15] <= results2_data[15];
        end

        if(results1_data[18] < results2_data[18])begin
            tx_data[16] <= results1_data[16];
            tx_data[17] <= results1_data[17];
            tx_data[18] <= results1_data[18];
            tx_data[19] <= results1_data[19];
        end else begin
            tx_data[16] <= results2_data[16];
            tx_data[17] <= results2_data[17];
            tx_data[18] <= results2_data[18];
            tx_data[19] <= results2_data[19];
        end

        if(results1_data[22] < results2_data[22])begin
            tx_data[20] <= results1_data[20];
            tx_data[21] <= results1_data[21];
            tx_data[22] <= results1_data[22];
            tx_data[23] <= results1_data[23];
        end else begin
            tx_data[20] <= results2_data[20];
            tx_data[21] <= results2_data[21];
            tx_data[22] <= results2_data[22];
            tx_data[23] <= results2_data[23];
        end

        if(results1_data[26] < results2_data[26])begin
            tx_data[24] <= results1_data[24];
            tx_data[25] <= results1_data[25];
            tx_data[26] <= results1_data[26];
            tx_data[27] <= results1_data[27];
        end else begin
            tx_data[24] <= results2_data[24];
            tx_data[25] <= results2_data[25];
            tx_data[26] <= results2_data[26];
            tx_data[27] <= results2_data[27];
        end

        if(results1_data[30] < results2_data[30])begin
            tx_data[28] <= results1_data[28];
            tx_data[29] <= results1_data[29];
            tx_data[30] <= results1_data[30];
            tx_data[31] <= results1_data[31];
        end else begin
            tx_data[28] <= results2_data[28];
            tx_data[29] <= results2_data[29];
            tx_data[30] <= results2_data[30];
            tx_data[31] <= results2_data[31];
        end
    end
end

always @(posedge clk) begin
    if(rst)
        tx_en <= 0;
    else if((!w_fifo_full) && (tx_cnt >= 32))
        tx_en <= 0;
    else if(set_result_r == 1 && tx_results_r == 1)
        tx_en <= 1;
end

always @(posedge clk) begin
    if(rst)begin
        tx_cnt <= 0;
        sink_fifo_stb <= 0;
        tx_data_buf <= 0;
        sink_fifo_last <= 0;
    end
    else if(tx_en == 1) begin
        if(!w_fifo_full)begin
            tx_cnt <= tx_cnt + 1;
            if(tx_cnt < 32) begin
                sink_fifo_stb <= 1;
                tx_data_buf <= tx_data[tx_cnt];
                sink_fifo_last <= 0;
            end else begin
                sink_fifo_stb <= 0;
                tx_data_buf <= 0;
                sink_fifo_last <= 1;
            end
        end
    end
    else begin
        tx_cnt <= 0;
        sink_fifo_stb <= 0;
        tx_data_buf <= 0;
        sink_fifo_last <= 0;
    end

end





(* dont_touch = "true" *)
fifo #(
    .DEPTH              (FIFO_DEPTH),
    .WIDTH              (FIFO_DATA_WIDTH)
) sink_fifo (
    .clk                (clk),
    .rst                (rst),

    .i_fifo_w_stb       (w_sink_fifo_w_stb),
    .i_fifo_w_data      (w_sink_fifo_w_data),
    .o_fifo_full        (w_fifo_full),
    .o_fifo_not_full    (),

    .i_fifo_r_stb       (w_sink_fifo_r_stb),
    .o_fifo_r_data      (w_sink_fifo_r_data),
    //.o_fifo_empty       (w_sink_fifo_empty),
    .o_fifo_not_empty   (w_sink_fifo_not_empty),
    .dbug_write_ptr(dbug_write_ptr),
    .dbug_read_ptr(dbug_read_ptr)
);


// sink FIFO -> AXIS sink
(* dont_touch = "true" *)
fifo_2_axis_adapter #(
    .AXIS_DATA_WIDTH    (AXIS_DATA_WIDTH)
)f2aa(
    .o_fifo_r_stb       (w_sink_fifo_r_stb),
    .i_fifo_data        (w_sink_fifo_r_data),
    .i_fifo_not_empty   (w_sink_fifo_not_empty),
    .i_fifo_last        (w_sink_fifo_r_last),

    .o_axis_tuser       (SINK_AXIS_tuser),
    .o_axis_tdata       (SINK_AXIS_tdata),
    .o_axis_tvalid      (SINK_AXIS_tvalid),
    .i_axis_tready      (SINK_AXIS_tready),
    .o_axis_tlast       (SINK_AXIS_tlast)
);
endmodule
