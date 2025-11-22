/* MIT License

Copyright (c) 2022 Po Jui Shih
Copyright (c) 2022 Hassaan Saadat
Copyright (c) 2022 Sri Parameswaran
Copyright (c) 2022 Hasindu Gamaarachchi

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE. */

`timescale 1ps / 1ps

`define MAJOR_VERSION       1
`define MINOR_VERSION       0
`define REVISION            0

`define MAJOR_RANGE         31:28
`define MINOR_RANGE         27:20
`define REVISION_RANGE      19:16
`define VERSION_PAD_RANGE   15:0

module dtw_accel #(
    parameter ADDR_WIDTH            = 16,
    parameter DATA_WIDTH            = 32,

    parameter AXIS_DATA_WIDTH       = 32,
    parameter AXIS_KEEP_WIDTH       = (AXIS_DATA_WIDTH / 8),
    parameter AXIS_DATA_USER_WIDTH  = 0,
    parameter FIFO_DATA_WIDTH       = AXIS_DATA_WIDTH,
    parameter FIFO_DEPTH            = 4,
    parameter INVERT_AXI_RESET      = 1,
    parameter INVERT_AXIS_RESET     = 1
)(
    input  wire                             S_AXI_clk,
    input  wire                             S_AXI_rst,

    // Write Address Channel
    input  wire                             S_AXI_awvalid,
    input  wire [ADDR_WIDTH - 1: 0]         S_AXI_awaddr,
    output wire                             S_AXI_awready,

    // Write Data Channel
    input  wire                             S_AXI_wvalid,
    output wire                             S_AXI_wready,
    input  wire [DATA_WIDTH - 1: 0]         S_AXI_wdata,

    // Write Response Channel
    output wire                             S_AXI_bvalid,
    input  wire                             S_AXI_bready,
    output wire [1:0]                       S_AXI_bresp,

    // Read Address Channel
    input  wire                             S_AXI_arvalid,
    output wire                             S_AXI_arready,
    input  wire [ADDR_WIDTH - 1: 0]         S_AXI_araddr,

    // Read Data Channel
    output wire                             S_AXI_rvalid,
    input  wire                             S_AXI_rready,
    output wire [1:0]                       S_AXI_rresp,
    output wire [DATA_WIDTH - 1: 0]         S_AXI_rdata,

    // AXI Stream

    // Input ref AXI Stream
    input  wire                             REF_AXIS_clk,//SRC_AXIS_clk,
    input  wire                             REF_AXIS_rst,//SRC_AXIS_rst,
    input  wire                             REF_AXIS_tuser,//SRC_AXIS_tuser,
    input  wire                             REF_AXIS_tvalid,//SRC_AXIS_tvalid,
    output wire                             REF_AXIS_tready,//SRC_AXIS_tready,
    input  wire                             REF_AXIS_tlast,//SRC_AXIS_tlast,
    input  wire [AXIS_DATA_WIDTH - 1:0]     REF_AXIS_tdata,//SRC_AXIS_tdata,

    // Input qeury AXI Stream
    input  wire                             QEU_AXIS_clk,
    input  wire                             QEU_AXIS_rst,
    input  wire                             QEU_AXIS_tuser,
    input  wire                             QEU_AXIS_tvalid,
    output wire                             QEU_AXIS_tready,
    input  wire                             QEU_AXIS_tlast,
    input  wire [AXIS_DATA_WIDTH - 1:0]     QEU_AXIS_tdata,

    //output results
    output reg tx_results,
    output                            results_val,
    output  [31:0]         curr_qeu_id,
    output  [31:0]    curr_ref_id,
    output  [31:0]    o_curr_minval,
    output  [31:0]         curr_position,
    output  reg [DATA_WIDTH - 1 : 0] ref_seg_number,
    output wire w_dtw_core_rst,
    input   qeu_next,
    input   [31:0]dbug_ref_seg_cnt,
    input   [3:0] dbug_rut_rscnt,
    input   [31:0] dbug_qid3,
    input   [31:0] dbug_qid6,
    input   [31:0] dbug_txdata,
    input dbug_sink_fifo_not_empty,
    input [1:0]dbug_reult_fifo_addr
);

/* ===============================
 * local parameters
 * =============================== */
// Address Map
localparam  REG_CONTROL      = 0;
localparam  REG_STATUS       = 1;
localparam  REG_REF_LEN      = 2;
localparam  REG_VERSION      = 3;
localparam  REG_KEY          = 4;
localparam  REG_REF_ADDR     = 5;
localparam  REG_REF_DIN      = 6;
localparam  REG_REF_DOUT     = 7;
localparam  REG_CYCLE_CNT    = 8;
localparam  REG_CORE_REF_ADDR= 9;
localparam  REG_NQUERY       = 10;
localparam  REG_CURR_QID     = 11;
localparam  REG_REF_SEG_NUMBER   = 12;
localparam  REG_REF_MEM_WRITE_ADDR   = 13;
localparam  REG_QEU_MEM_WRITE_ADDR   = 14;
localparam  REG_RUT_ADDR   = 15;
localparam  REG_RUT_RSCNT_ADDR   = 16;
localparam  REG_DBUG_QID3_ADDR   = 17;
localparam  REG_DBUG_QID6_ADDR   = 18;
localparam  REG_DBUG_TXDATA_ADDR   = 19;
localparam  REG_CURR_RID     = 20;
localparam  REG_DBUG_RU_FIFO_ADDR  = 21;
localparam  REG_DBUG_FIRST_QID  = 22;




localparam  integer ADDR_LSB = (DATA_WIDTH / 32) + 1;
localparam  integer ADDR_BITS = 5;

localparam  MAX_ADDR = REG_KEY;

localparam REFMEM_PTR_WIDTH = 15;
localparam QEUMEM_PTR_WIDTH = 11;

localparam SQG_SIZE = 200;
localparam QEU_LEN  = 8;

/* ===============================
 * registers/wires
 * =============================== */
// User Interface
wire                            w_axi_rst;
wire                            ref_axis_rst;
wire                            qeu_axis_rst;
wire  [ADDR_WIDTH - 1 : 0]      w_reg_address;
reg                             r_reg_invalid_addr;

wire                            w_reg_in_rdy;
reg                             r_reg_in_ack_stb;
wire  [DATA_WIDTH - 1 : 0]      w_reg_in_data;

wire                            w_reg_out_req;
reg                             r_reg_out_rdy_stb;
reg   [DATA_WIDTH - 1 : 0]      r_reg_out_data;


wire [15:0]curr_minval;

// DTW accel
reg   [DATA_WIDTH - 1 : 0]      r_control;
wire  [DATA_WIDTH - 1 : 0]      w_status;
reg   [DATA_WIDTH - 1 : 0]      r_ref_len;
wire  [DATA_WIDTH - 1 : 0]      w_version;
wire  [DATA_WIDTH - 1 : 0]      w_key;
reg   [DATA_WIDTH - 1 : 0]      r_dbg_ref_addr;
reg   [DATA_WIDTH - 1 : 0]      r_dbg_ref_din;
wire  [DATA_WIDTH - 1 : 0]      w_dbg_ref_dout;
wire  [DATA_WIDTH - 1 : 0]      w_dtw_core_cycle_counter;


// Control Register bits
//reg dtw_core_rst;


// Status Register bits
wire                            w_dtw_core_busy;
//wire                            w_dtw_core_load_done;

// REF Src FIFO
wire                            ref_src_fifo_clear;//w_src_fifo_clear;
wire  [FIFO_DATA_WIDTH - 1:0]   ref_src_fifo_w_data;//w_src_fifo_w_data;
wire                            ref_src_fifo_w_stb;//w_src_fifo_w_stb;
wire                            ref_src_fifo_full;//w_src_fifo_full;
wire                            ref_src_fifo_not_full;//w_src_fifo_not_full;

wire  [FIFO_DATA_WIDTH - 1:0]   ref_src_fifo_r_data;//w_src_fifo_r_data;
wire                            ref_src_fifo_r_stb;//w_src_fifo_r_stb;
wire                            ref_src_fifo_empty;//w_src_fifo_empty;
wire                            ref_src_fifo_not_empty;//w_src_fifo_not_empty;

// QEU Src FIFO
wire                            qeu_src_fifo_clear;
wire  [FIFO_DATA_WIDTH - 1:0]   qeu_src_fifo_w_data;
wire                            qeu_src_fifo_w_stb;
wire                            qeu_src_fifo_full;
wire                            qeu_src_fifo_not_full;

wire  [FIFO_DATA_WIDTH - 1:0]   qeu_src_fifo_r_data;
wire                            qeu_src_fifo_r_stb;
wire                            qeu_src_fifo_empty;
wire                            qeu_src_fifo_not_empty;
/*
wire  [FIFO_DATA_WIDTH - 1:0]   w_sink_fifo_r_data;
wire                            w_sink_fifo_r_stb;
wire                            w_sink_fifo_empty;
wire                            w_sink_fifo_not_empty;
*/
// dtw core debug
wire  [2:0]                     w_dtw_core_state;
wire  [REFMEM_PTR_WIDTH-1:0]   w_dtw_core_addr_ref;
wire  [31:0]                    w_dtw_core_nquery;
//wire  [31:0]                    w_dtw_core_curr_qid;


//ref mem signals
wire ref_mem_read;
wire [REFMEM_PTR_WIDTH-1:0]ref_read_addr;
wire [15:0]dataout_ref;
wire [DATA_WIDTH - 1 : 0]ref_len;
wire ref_load_done;

    //qeu mem sigsnals
//wire qeu_mem_read;
wire [QEUMEM_PTR_WIDTH-1:0]qeu_read_addr;
wire [15:0]dataout_qeu;
wire qeu_load_done;
//wire qeu_next;

wire qeu_mem_busy;
wire ref_mem_busy;

wire w_ref_load_cmd;
wire w_qeu_load_cmd;

//reg ref_load_cmd;
//reg qeu_load_cmd;

wire [1:0]dbug_qeu_load_state;
wire [1:0]dbug_ref_load_state;

wire [10:0]dbug_qeu_mem_write_addr;
wire [14:0]dbug_ref_mem_write_addr;

wire [31:0]dbg_first_qid;

//wire [15:0] dbug_qeu_mem0;
//wire [15:0] dbug_qeu_mem1;
//wire [15:0] dbug_qeu_mem2;

//wire [15:0] dbug_ref_mem0;
//wire [15:0] dbug_ref_mem1;
//wire [15:0] dbug_ref_mem2;



/* ===============================
 * initialization
 * =============================== */
initial begin
    r_control <= 0;
    r_ref_len <= 4000;
end

/* ===============================
 * submodules
 * =============================== */
// Convert AXI Slave bus to a simple register/address strobe
(* dont_touch = "true" *)
axi_lite_slave #(
    .ADDR_WIDTH         (ADDR_WIDTH),
    .DATA_WIDTH         (DATA_WIDTH)
) axi_lite_reg_interface (
    .clk                (S_AXI_clk),
    .rst                (w_axi_rst),

    .i_awvalid          (S_AXI_awvalid),
    .i_awaddr           (S_AXI_awaddr),
    .o_awready          (S_AXI_awready),

    .i_wvalid           (S_AXI_wvalid),
    .o_wready           (S_AXI_wready),
    .i_wdata            (S_AXI_wdata),

    .o_bvalid           (S_AXI_bvalid),
    .i_bready           (S_AXI_bready),
    .o_bresp            (S_AXI_bresp),

    .i_arvalid          (S_AXI_arvalid),
    .o_arready          (S_AXI_arready),
    .i_araddr           (S_AXI_araddr),

    .o_rvalid           (S_AXI_rvalid),
    .i_rready           (S_AXI_rready),
    .o_rresp            (S_AXI_rresp),
    .o_rdata            (S_AXI_rdata),


    // Register Interface
    .o_reg_address      (w_reg_address),
    .i_reg_invalid_addr (r_reg_invalid_addr),

    // From Master
    .o_reg_in_rdy       (w_reg_in_rdy),
    .i_reg_in_ack_stb   (r_reg_in_ack_stb),
    .o_reg_in_data      (w_reg_in_data),

    // To Master
    .o_reg_out_req      (w_reg_out_req),
    .i_reg_out_rdy_stb  (r_reg_out_rdy_stb),
    .i_reg_out_data     (r_reg_out_data)
);


// ref AXIS src -> src FIFO
(* dont_touch = "true" *)
axis_2_fifo_adapter #(
    .AXIS_DATA_WIDTH    (AXIS_DATA_WIDTH)
) ref_a2fa (
    .i_axis_tuser       (REF_AXIS_tuser),
    .i_axis_tvalid      (REF_AXIS_tvalid),
    .o_axis_tready      (REF_AXIS_tready),
    .i_axis_tlast       (REF_AXIS_tlast),
    .i_axis_tdata       (REF_AXIS_tdata),

    .o_fifo_data        (ref_src_fifo_w_data),
    .o_fifo_w_stb       (ref_src_fifo_w_stb),
    .i_fifo_not_full    (ref_src_fifo_not_full)
);

(* dont_touch = "true" *)
fifo #(
    .DEPTH              (FIFO_DEPTH),
    .WIDTH              (FIFO_DATA_WIDTH)
) src_ref_fifo (
    .clk                (REF_AXIS_clk),
    .rst                (ref_axis_rst | ref_src_fifo_clear),

    .i_fifo_w_stb       (ref_src_fifo_w_stb),
    .i_fifo_w_data      (ref_src_fifo_w_data),
    .o_fifo_full        (ref_src_fifo_full),
    .o_fifo_not_full    (ref_src_fifo_not_full),

    .i_fifo_r_stb       (ref_src_fifo_r_stb),
    .o_fifo_r_data      (ref_src_fifo_r_data),
    .o_fifo_empty       (ref_src_fifo_empty),
    .o_fifo_not_empty   (ref_src_fifo_not_empty)
);

(* dont_touch = "true" *)
ref_mem #(
    .WIDTH              (16),
    .AXIS_WIDTH         (AXIS_DATA_WIDTH),
    .REF_INIT           (0),
    .REFMEM_PTR_WIDTH   (REFMEM_PTR_WIDTH)
) uu_ref_mem (

    // Src FIFO signals
    .clk(S_AXI_clk),
    .rst(w_dtw_core_rst),
    .load_cmd(w_ref_load_cmd),
    .ref_len(r_ref_len),
    .src_fifo_clear(ref_src_fifo_clear),     // Src FIFO Clear signal
    .src_fifo_rden(ref_src_fifo_r_stb),      // Src FIFO Read enable
    .src_fifo_empty(ref_src_fifo_empty),     // Src FIFO Empty
    .src_fifo_data(ref_src_fifo_r_data[15:0]),      // Src FIFO Data

    //mem signals
    .ref_mem_read(ref_mem_read),
    .ref_read_addr(ref_read_addr),
    .dataout_ref(dataout_ref),
    .ref_len_out(ref_len),
    .mem_busy(ref_mem_busy),
    .ref_load_done(ref_load_done),
    .dbug_ref_mem_write_addr(dbug_ref_mem_write_addr),//[14:0]
    .dbug_ref_load_state(dbug_ref_load_state)//[1:0]
    //.dbug_ref_mem0(dbug_ref_mem0),
    //.dbug_ref_mem1(dbug_ref_mem1),
    //.dbug_ref_mem2(dbug_ref_mem2)
    );


// qeu AXIS src -> src FIFO
(* dont_touch = "true" *)
axis_2_fifo_adapter #(
    .AXIS_DATA_WIDTH    (AXIS_DATA_WIDTH)
) qeu_a2fa (
    .i_axis_tuser       (QEU_AXIS_tuser),
    .i_axis_tvalid      (QEU_AXIS_tvalid),
    .o_axis_tready      (QEU_AXIS_tready),
    .i_axis_tlast       (QEU_AXIS_tlast),
    .i_axis_tdata       (QEU_AXIS_tdata),

    .o_fifo_data        (qeu_src_fifo_w_data),
    .o_fifo_w_stb       (qeu_src_fifo_w_stb),
    .i_fifo_not_full    (qeu_src_fifo_not_full)
);

(* dont_touch = "true" *)
fifo #(
    .DEPTH              (FIFO_DEPTH),
    .WIDTH              (FIFO_DATA_WIDTH)
) src_qeu_fifo (
    .clk                (QEU_AXIS_clk),
    .rst                (qeu_axis_rst | qeu_src_fifo_clear),

    .i_fifo_w_stb       (qeu_src_fifo_w_stb),
    .i_fifo_w_data      (qeu_src_fifo_w_data),
    .o_fifo_full        (qeu_src_fifo_full),
    .o_fifo_not_full    (qeu_src_fifo_not_full),

    .i_fifo_r_stb       (qeu_src_fifo_r_stb),
    .o_fifo_r_data      (qeu_src_fifo_r_data),
    .o_fifo_empty       (qeu_src_fifo_empty),
    .o_fifo_not_empty   (qeu_src_fifo_not_empty)
);

(* dont_touch = "true" *)
qeu_mem #(
    .WIDTH(16),   // Data width
    .AXIS_WIDTH(AXIS_DATA_WIDTH),   // AXI data width
    .SQG_SIZE(SQG_SIZE),  // Squiggle size
    .QEU_LEN(QEU_LEN),  
    .MEM_INIT(0),
    .QEUMEM_PTR_WIDTH(QEUMEM_PTR_WIDTH)
) uu_qeu_mem(
    // Src FIFO signals
    .clk(S_AXI_clk),
    .rst(w_dtw_core_rst),
    .load_cmd(w_qeu_load_cmd),
    .qeu_next(qeu_next),
    .src_fifo_clear(qeu_src_fifo_clear),     // Src FIFO Clear signal
    .src_fifo_rden(qeu_src_fifo_r_stb),      // Src FIFO Read enable
    .src_fifo_empty(qeu_src_fifo_empty),     // Src FIFO Empty
    .src_fifo_data(qeu_src_fifo_r_data[15:0]),      // Src FIFO Data

    //mem signals
    //.qeu_mem_read(qeu_mem_read),
    .qeu_read_addr(qeu_read_addr),
    .dataout_qeu(dataout_qeu),
    .mem_busy(qeu_mem_busy),
    .qeu_load_done(qeu_load_done),
    .dbug_qeu_mem_write_addr(dbug_qeu_mem_write_addr),//[10:0]
    .dbug_qeu_load_state(dbug_qeu_load_state)
    //.dbug_qeu_mem0(dbug_qeu_mem0),
    //.dbug_qeu_mem1(dbug_qeu_mem1),
    //.dbug_qeu_mem2(dbug_qeu_mem2)
    );




// DTW core
(* dont_touch = "true" *)
dtw_core #(
    .WIDTH(16),   // Data width
    .AXIS_WIDTH(AXIS_DATA_WIDTH),   // AXI data width
    .SQG_SIZE(SQG_SIZE),  // Squiggle size
    .REF_INIT(0),
    .REFMEM_PTR_WIDTH(REFMEM_PTR_WIDTH),
    .QEUMEM_PTR_WIDTH(QEUMEM_PTR_WIDTH)
) dc (
    // Main DTW signals
    .clk(S_AXI_clk),
    .rst(w_dtw_core_rst),
    .busy(w_dtw_core_busy),               // Idle: 0, busy: 1

    //ref mem signals
    .ref_mem_read(ref_mem_read),
    .ref_read_addr(ref_read_addr),
    .dataout_ref(dataout_ref),
    .ref_len(ref_len),
    .ref_load_done(ref_load_done),

    //qeu mem sigsnals
    //.qeu_mem_read(qeu_mem_read),
    .qeu_read_addr(qeu_read_addr),
    .dataout_qeu(dataout_qeu),
    .qeu_load_done(qeu_load_done),
    //.qeu_next(qeu_next),

    // Sink FIFO signals
    .results_val(results_val),     // Sink FIFO Write enable
    .curr_qeu_id(curr_qeu_id),
    .curr_ref_id(curr_ref_id),
    .curr_minval(curr_minval),
    .curr_position(curr_position),

    // debug signals
    .dbg_state(w_dtw_core_state),
    .dbg_cycle_counter(w_dtw_core_cycle_counter),
    .dbg_nquery(w_dtw_core_nquery),
    .dbg_first_qid(dbg_first_qid)
    //.dbg_curr_qeu_id(w_dtw_core_curr_qid)
);



/* ===============================
 * asynchronous logic
 * =============================== */
assign w_axi_rst                        = INVERT_AXI_RESET  ? ~S_AXI_rst    : S_AXI_rst;
assign ref_axis_rst                       = INVERT_AXIS_RESET ? ~REF_AXIS_rst : REF_AXIS_rst;
assign qeu_axis_rst                       = INVERT_AXIS_RESET ? ~QEU_AXIS_rst : QEU_AXIS_rst;
assign w_version[`MAJOR_RANGE]          = `MAJOR_VERSION;
assign w_version[`MINOR_RANGE]          = `MINOR_VERSION;
assign w_version[`REVISION_RANGE]       = `REVISION;
assign w_version[`VERSION_PAD_RANGE]    = 0;
assign w_key                            = 32'h0ca7cafe;

assign o_curr_minval = {16'd0,curr_minval[15:0]};

//assign w_dtw_core_rst                   = dtw_core_rst;
//assign w_ref_load_cmd                   = ref_load_cmd;
//assign w_qeu_load_cmd                   = qeu_load_cmd;

assign w_dtw_core_rst                   = r_control[0];
assign w_ref_load_cmd                   = r_control[1];
assign w_qeu_load_cmd                   = r_control[2];


assign w_status[0]                      = w_dtw_core_busy;
assign w_status[1]                      = qeu_axis_rst|qeu_src_fifo_clear;
assign w_status[2]                      = ref_src_fifo_empty;
assign w_status[3]                      = ref_src_fifo_full;
assign w_status[4]                      = ref_load_done;
assign w_status[5]                      = qeu_load_done;
assign w_status[8:6]                    = w_dtw_core_state;
assign w_status[9]                      = qeu_mem_busy;
assign w_status[10]                     = ref_mem_busy;
assign w_status[12:11]                  = dbug_qeu_load_state;
assign w_status[14:13]                  = dbug_ref_load_state;
assign w_status[15]                     = ref_axis_rst | ref_src_fifo_clear;
//dbug_sink_fifo_not_empty
assign w_status[16]                     = dbug_sink_fifo_not_empty;
assign w_status[31:17]                  = 0;

/* ===============================
 * synchronous logic
 * =============================== */
/*
always @(posedge S_AXI_clk)begin
    if(w_axi_rst)
        ref_load_cmd <= 0;
    else if(w_reg_in_rdy == 1 && (w_reg_address[ADDR_LSB + ADDR_BITS:ADDR_LSB] == REG_CONTROL) && w_reg_in_data[1] == 1)
        ref_load_cmd <= 1;
    else
        ref_load_cmd <= 0;
end

always @(posedge S_AXI_clk)begin
    if(w_axi_rst)
        qeu_load_cmd <= 0;
    else if(w_reg_in_rdy == 1 && w_reg_in_data[2] == 1)
        qeu_load_cmd <= 1;
    else
        qeu_load_cmd <= 0;
end

always @(posedge S_AXI_clk)begin
    if(w_axi_rst)
        dtw_core_rst <= 0;
    else if(w_reg_in_rdy == 1 && w_reg_in_data[0] == 1)
        dtw_core_rst <= 1;
    else
        dtw_core_rst <= 0;
end
*/

always @ (posedge S_AXI_clk) begin
    if(w_axi_rst)
        tx_results <= 0;
    else if(w_reg_in_rdy == 1 && w_reg_address[ADDR_LSB + ADDR_BITS:ADDR_LSB] == REG_CONTROL)
        tx_results <= w_reg_in_data[3];
    else
        tx_results <= 0;
end


always @ (posedge S_AXI_clk) begin
    // De-assert Strobes
    r_reg_in_ack_stb    <=  0;
    r_reg_out_rdy_stb   <=  0;
    r_reg_invalid_addr  <=  0;

    if (w_axi_rst) begin
        r_reg_out_data  <=  0;

        // Reset registers
        r_control       <=  0;
        r_ref_len       <=  0;
        ref_seg_number  <= 32'hffff_ffff;
    end else begin
        if (w_reg_in_rdy) begin
            // M_AXI to here
            case (w_reg_address[ADDR_LSB + ADDR_BITS:ADDR_LSB])
            REG_CONTROL: begin
                r_control <= w_reg_in_data; 
            end
            REG_STATUS: begin
            end
            REG_REF_LEN: begin
                r_ref_len <= w_reg_in_data;
            end
            REG_VERSION: begin
            end
            REG_KEY: begin
            end
            REG_REF_ADDR: begin
                r_dbg_ref_addr <= w_reg_in_data;
            end
            REG_REF_DIN: begin
                r_dbg_ref_din <= w_reg_in_data;
            end
            REG_REF_DOUT: begin
            end
            REG_CYCLE_CNT: begin
            end
            REG_CORE_REF_ADDR: begin
            end
            REG_NQUERY: begin
            end
            REG_CURR_QID: begin
            end
            REG_REF_SEG_NUMBER: begin
                ref_seg_number <= w_reg_in_data;
            end
            REG_REF_MEM_WRITE_ADDR: begin
            end
            REG_QEU_MEM_WRITE_ADDR:begin
            end
            REG_RUT_ADDR:begin
            end
            REG_RUT_RSCNT_ADDR:begin
            end
            REG_DBUG_QID3_ADDR:begin
            end
            REG_DBUG_QID6_ADDR:begin
            end
            REG_DBUG_TXDATA_ADDR:begin
            end
            REG_CURR_RID: begin
            end
            REG_DBUG_RU_FIFO_ADDR:begin
            end
            REG_DBUG_FIRST_QID:begin
            end
            default: begin // unknown address
                $display ("Unknown address: 0x%h", w_reg_address);
                r_reg_invalid_addr <= 1;
            end
            endcase
            r_reg_in_ack_stb <= 1; // Tell AXI Slave we are done with the data
        end else if (w_reg_out_req) begin
            // Here to M_AXI
            case (w_reg_address[ADDR_LSB + ADDR_BITS:ADDR_LSB])
            REG_CONTROL: begin
                r_reg_out_data <= r_control;
            end
            REG_STATUS: begin
                r_reg_out_data <= w_status;
            end
            REG_REF_LEN: begin
                r_reg_out_data <= r_ref_len;
            end
            REG_VERSION: begin
                r_reg_out_data <= w_version;
            end
            REG_KEY: begin
                r_reg_out_data <= w_key;
            end
            REG_REF_ADDR: begin
                r_reg_out_data <= r_dbg_ref_addr;
            end
            REG_REF_DIN: begin
                r_reg_out_data <= r_dbg_ref_din;
            end
            REG_REF_DOUT: begin
                r_reg_out_data <= w_dbg_ref_dout;
            end
            REG_CYCLE_CNT: begin
                r_reg_out_data <= w_dtw_core_cycle_counter;
            end
            REG_CORE_REF_ADDR: begin
                r_reg_out_data <= {12'h0, w_dtw_core_addr_ref};
            end
            REG_NQUERY: begin
                r_reg_out_data <= w_dtw_core_nquery;
            end
            REG_CURR_QID: begin
                r_reg_out_data <= curr_qeu_id;
            end
            REG_REF_SEG_NUMBER: begin
                r_reg_out_data <= ref_seg_number;
            end
            REG_REF_MEM_WRITE_ADDR:begin
                r_reg_out_data <= {17'd0,dbug_ref_mem_write_addr};
            end
            REG_QEU_MEM_WRITE_ADDR:begin
                r_reg_out_data <= {21'd0,dbug_qeu_mem_write_addr};
            end
            REG_RUT_ADDR:begin
                r_reg_out_data <= dbug_ref_seg_cnt;
            end
            REG_RUT_RSCNT_ADDR:begin
                r_reg_out_data <= {12'd0,dbug_rut_rscnt};
            end
            REG_DBUG_QID3_ADDR: begin
                r_reg_out_data <= dbug_qid3;
            end
            REG_DBUG_QID6_ADDR: begin
                r_reg_out_data <= dbug_qid6;
            end
            REG_DBUG_TXDATA_ADDR:begin
                r_reg_out_data <= dbug_txdata;
            end
            REG_CURR_RID: begin
                r_reg_out_data <= curr_ref_id;
            end
            REG_DBUG_RU_FIFO_ADDR:begin
                r_reg_out_data <= {30'd0,dbug_reult_fifo_addr};
            end
            REG_DBUG_FIRST_QID:begin
                r_reg_out_data <= dbg_first_qid;
            end
            default: begin // Unknown address
                r_reg_out_data      <= 32'h00;
                r_reg_invalid_addr  <= 1;
            end
            endcase
            r_reg_out_rdy_stb <= 1; // Tell AXI Slave to send back this packet
        end
    end
end

endmodule