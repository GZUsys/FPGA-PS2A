/*
Distributed under the MIT license.
Copyright (c) 2022 Elton Shih (beebdev@gmail.com)

Permission is hereby granted, free of charge, to any person obtaining a copy of
this software and associated documentation files (the "Software"), to deal in
the Software without restriction, including without limitation the rights to
use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies
of the Software, and to permit persons to whom the Software is furnished to do
so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

/*
 * Author: Elton Shih (beebdev@gmail.com)
 * Description: Core implementation for pipelined subsequence DTW algorithm
 *
 * Changes:     Author         Description
 *  03/31/2022  Elton Shih     Initial Commit
 */

`timescale 1ns / 1ps

module dtw_core #(
    parameter WIDTH         = 16,   // Data width
    parameter AXIS_WIDTH    = 32,   // AXI data width
    parameter SQG_SIZE      = 250,  // Squiggle size
    parameter REF_INIT      = 0,
    parameter REFMEM_PTR_WIDTH = 15,
    parameter QEUMEM_PTR_WIDTH = 11
)(
    // Main DTW signals
    input   wire                    clk,
    input   wire                    rst,
    output  reg                     busy,               // Idle: 0, busy: 1

    //ref mem signals
    output reg ref_mem_read,
    output reg [REFMEM_PTR_WIDTH-1:0]ref_read_addr,
    input [WIDTH-1:0] dataout_ref,
    input [AXIS_WIDTH-1 : 0]ref_len,
    input ref_load_done,

    //qeu mem sigsnals
    //output reg qeu_mem_read,
    output reg [REFMEM_PTR_WIDTH-1:0]qeu_read_addr,
    input [WIDTH-1:0] dataout_qeu,
    input qeu_load_done,
   // output reg qeu_next,

    // Sink FIFO signals
    output  reg                     results_val,     // Sink FIFO Write enable
    //output  reg [31:0]              results_data,     // Sink FIFO Data
    output  reg  [31:0]         curr_qeu_id,
    output  reg  [31:0]    curr_ref_id,
    output  [WIDTH-1:0]    curr_minval,
    output  [31:0]         curr_position,

    // debug signals
    output  wire [2:0]              dbg_state,
    //output  wire [REFMEM_PTR_WIDTH-1:0]             dbg_ref_read_addr,

    output  wire [31:0]             dbg_cycle_counter,
    output  wire [31:0]             dbg_nquery,
    output  reg  [31:0]             dbg_first_qid
);

/* ===============================
 * local parameters
 * =============================== */

// FSM states
localparam [2:0] // n states
    IDLE = 0,
    //REF_LOAD = 1,
    DTW_Q_INIT = 1,
    DTW_Q_RUN = 2,
    DTW_Q_DONE = 3,
    DTW_Q_WAIT1 = 4,
    DTW_Q_WAIT2 = 5;

/* ===============================
 * registers/wires
 * =============================== */
reg r_load_done;

reg [3:0]qeu_cnt;

// DTW datapath signals
reg                 dp_rst;             // dp core reset
reg                 dp_running;         // dp core run enable
wire                dp_done;            // dp core done

//wire [WIDTH-1:0]    curr_minval;        // Current minimum value
//wire [31:0]         curr_position;      // Current best match position

// FSM state
reg [2:0] r_state;

// Others
//reg [2:0] stall_counter;
reg [31:0] r_dbg_nquery;

/* ===============================
 * submodules
 * =============================== */

// DTW datapath
(* dont_touch = "true" *)
dtw_core_datapath #(
    .width      (WIDTH),
    .SQG_SIZE   (SQG_SIZE)
) inst_dtw_core_datapath (
    .clk            (clk),
    .rst            (dp_rst),
    .running        (dp_running),
    .Input_squiggle (dataout_qeu),//qeu
    .Rword          (dataout_ref),
    .ref_len        (ref_len),
    .minval         (curr_minval),
    .position       (curr_position),
    .done           (dp_done),

    // debug
    .dbg_cycle_counter (dbg_cycle_counter)
);

/* ===============================
 * asynchronous logic
 * =============================== */
//assign src_fifo_clear = r_src_fifo_clear;
assign dbg_state = r_state;
//assign dbg_ref_read_addr = ref_read_addr;
assign dbg_nquery = r_dbg_nquery;
//assign dbg_curr_qeu_id = curr_qeu_id;

/* ===============================
 * synchronous logic
 * =============================== */
// FSM State change
always @(posedge clk) begin
    if (rst) begin
        r_state <= IDLE;
    end else begin
        case (r_state)
        IDLE: begin
            if (ref_load_done & qeu_load_done) begin
                r_state <= DTW_Q_INIT;
            end else begin
                r_state <= IDLE;
            end
        end
        DTW_Q_INIT: begin
            r_state <= DTW_Q_RUN;
        end
        DTW_Q_RUN: begin
            if (!dp_done) begin
                r_state <= DTW_Q_RUN;
            end else begin
                r_state <= DTW_Q_DONE;
            end
        end
        DTW_Q_DONE: begin
            r_state <= DTW_Q_WAIT1;
        end
        DTW_Q_WAIT1: begin
            r_state <= DTW_Q_WAIT2;
        end
        DTW_Q_WAIT2: begin
            r_state <= IDLE;
        end
        endcase
    end
end

// FSM output
always @(posedge clk) begin
    case (r_state)
    IDLE: begin
        busy                <= 0;
        //qeu_mem_read       <= 0;
        results_val      <= 0;
        ref_read_addr           <= 0;
        dp_rst              <= 1;
        dp_running          <= 0;
        //stall_counter       <= 0;
        ref_mem_read            <= 0;
       // r_src_fifo_clear    <= 1;
        //sink_fifo_last      <= 0;
        curr_qeu_id            <= 0;
        curr_ref_id            <= 0;
    end
    DTW_Q_INIT:begin
        busy               <= 1;
        curr_qeu_id        <= dataout_qeu;
        curr_ref_id        <= dataout_ref;
        ref_mem_read       <= 1;
        ref_read_addr      <= ref_read_addr + 1;
        //qeu_mem_read       <= 1;
        qeu_read_addr      <= qeu_read_addr + 1;
        dp_rst             <= 0;
        //stall_counter      <= 0;
        dp_running         <= 1;
        if(qeu_cnt == 0) begin
            dbg_first_qid <= dataout_qeu;
        end
    end
    DTW_Q_RUN: begin
        busy               <= 1;
        results_val        <= 0;
        dp_rst             <= 0;
        //stall_counter      <= 0;
        ref_mem_read       <= 1;
        ref_read_addr      <= ref_read_addr + 1;
        //dp_running          <= 1;
        if (ref_read_addr < SQG_SIZE + 2) begin//next q_id
            // Query loading
                qeu_read_addr  <= qeu_read_addr + 1;
                //qeu_mem_read   <= 1;
                dp_running     <= 1;
        end else begin
            // Query loaded
            //qeu_mem_read       <= 0;
            dp_running          <= 1;
        end
    end
    DTW_Q_DONE: begin
        busy            <= 1;
        //qeu_mem_read   <= 0;
        dp_rst          <= 0;
        dp_running      <= 0;
        ref_mem_read        <= 0;
        ref_read_addr           <= 0;

        // Serialize output
        //add ref id
        results_val <= 1;
        if(qeu_cnt == 7) begin
            qeu_read_addr <= 0;
        end
    end
    DTW_Q_WAIT1:begin
        busy            <= 1;
        //qeu_mem_read   <= 0;
        dp_rst          <= 0;
        dp_running      <= 0;
        ref_mem_read        <= 0;
        ref_read_addr           <= 0;

        // Serialize output
        //add ref id
        results_val <= 0;
    end
    DTW_Q_WAIT2:begin
        busy            <= 1;
        //qeu_mem_read   <= 0;
        dp_rst          <= 0;
        dp_running      <= 0;
        ref_mem_read        <= 0;
        ref_read_addr           <= 0;

        // Serialize output
        //add ref id
        results_val <= 0;
    end
    endcase
end

always @(posedge clk) begin
    if(rst)
        qeu_cnt <= 0;
    else if(qeu_cnt == 8)
        qeu_cnt <= 0;
    else if(r_state == DTW_Q_DONE)
        qeu_cnt <= qeu_cnt + 1;
end
endmodule