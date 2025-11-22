`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 2025/10/14 00:00:40
// Design Name: 
// Module Name: qeu_mem
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


module qeu_mem #(
    parameter WIDTH         = 16,   // Data width
    parameter AXIS_WIDTH    = 32,   // AXI data width
    parameter SQG_SIZE      = 200,  // Squiggle size
    parameter QEU_LEN       = 8,  
    parameter MEM_INIT      = 0,
    parameter QEUMEM_PTR_WIDTH = 11,
    parameter QEU_SIZE      = (SQG_SIZE+2)*QEU_LEN
)(
    // Src FIFO signals
    input clk,
    input rst,
    input load_cmd,
    input qeu_next,
    //input [AXIS_WIDTH-1 : 0] que_len,
    output  reg                     src_fifo_clear,     // Src FIFO Clear signal
    output  reg                     src_fifo_rden,      // Src FIFO Read enable
    input   wire                    src_fifo_empty,     // Src FIFO Empty
    input   wire [31:0]             src_fifo_data,      // Src FIFO Data

    //mem signals
    //input qeu_mem_read,
    input [QEUMEM_PTR_WIDTH-1:0]qeu_read_addr,
    output [WIDTH-1:0] dataout_qeu,
    output mem_busy,
    output qeu_load_done,
    output [QEUMEM_PTR_WIDTH-1:0]dbug_qeu_mem_write_addr,
    output [1:0] dbug_qeu_load_state
    //output  wire [15:0] dbug_qeu_mem0,
    //output  wire [15:0] dbug_qeu_mem1,
    //output  wire [15:0] dbug_qeu_mem2

    );

wire [QEUMEM_PTR_WIDTH-1:0]addr_qeu1;
wire wren_qeu1;

wire [QEUMEM_PTR_WIDTH-1:0]addr_qeu2;
wire wren_qeu2;

wire [15:0]dataout_qeu1;
wire [15:0]dataout_qeu2;

// FSM states
localparam [1:0] // n states
    IDLE = 0,
    QEU_LOAD = 1,
    LOAD_DONE = 2;

reg mem_w_flag;
reg mem_r_flag;

reg mem1_busy;
reg mem2_busy;

reg mem1_load_done;
reg mem2_load_done;

//reg [1:0]qeu_mem_read_reg;
//reg [3:0]read_mem_cnt;

reg [1:0] load_state;

reg [QEUMEM_PTR_WIDTH-1:0]addr_ref;
wire wren_ref;


assign addr_qeu1 = (mem_w_flag == 0 && load_state == QEU_LOAD)?addr_ref:qeu_read_addr;
assign wren_qeu1 = (mem_w_flag == 0 && load_state == QEU_LOAD)?wren_ref:0;

assign addr_qeu2 = (mem_w_flag == 1 && load_state == QEU_LOAD)?addr_ref:qeu_read_addr;
assign wren_qeu2 = (mem_w_flag == 1 && load_state == QEU_LOAD)?wren_ref:0;

assign wren_ref = (!src_fifo_empty && src_fifo_rden)?1'b1:1'b0;

assign dataout_qeu = (mem_r_flag == 0)?dataout_qeu1:dataout_qeu2;

assign mem_busy = mem1_busy&mem2_busy;

assign qeu_load_done = mem1_load_done|mem2_load_done;

assign dbug_qeu_mem_write_addr = addr_ref;

assign dbug_qeu_load_state = load_state;


// Reference memory
(* dont_touch = "true" *)
dtw_mem #(
    .width      (WIDTH),
    .initalize  (MEM_INIT),
    .ptrWid     (QEUMEM_PTR_WIDTH)
) ref_mem1 (
    .clk          (clk),
    .wen          (wren_qeu1),
    .addr         (addr_qeu1[QEUMEM_PTR_WIDTH-1:0]),
    .din          (src_fifo_data[15:0]),
    .dout         (dataout_qeu1)
    //.dbug_mem0(dbug_qeu_mem0),
    //.dbug_mem1(dbug_qeu_mem1),
    //.dbug_mem2(dbug_qeu_mem2)
);

(* dont_touch = "true" *)
dtw_mem #(
    .width      (WIDTH),
    .initalize  (MEM_INIT),
    .ptrWid     (QEUMEM_PTR_WIDTH)
) ref_mem2 (
    .clk          (clk),
    .wen          (wren_qeu2),
    .addr         (addr_qeu2[QEUMEM_PTR_WIDTH-1:0]),
    .din          (src_fifo_data[15:0]),
    .dout         (dataout_qeu2)
    //.dbug_mem0(),
    //.dbug_mem1(),
    //.dbug_mem2()
);


// FSM State change
always @(posedge clk) begin
    if (rst) begin
        load_state <= IDLE;
    end else begin
        case (load_state)
        IDLE: begin
            if (load_cmd) begin
                if(mem1_busy == 0 || mem2_busy == 0)
                    load_state <= QEU_LOAD;
                else
                    load_state <= IDLE;
        end else begin
            load_state <= IDLE;
        end
        end

        QEU_LOAD: begin
            if (addr_ref < QEU_SIZE) begin
                load_state <= QEU_LOAD;
            end else begin
                load_state <= LOAD_DONE;
            end
        end

        LOAD_DONE:load_state <= IDLE;
        
        endcase
    end
end


always @(posedge clk) begin
    case (load_state)
    IDLE: begin
        addr_ref <= 0;
        //wren_ref <= 0;
        src_fifo_rden <= 0;
        src_fifo_clear <= 1;
    end
    QEU_LOAD: begin
        src_fifo_clear <= 0;
        src_fifo_rden <= 1;
        if (!src_fifo_empty && src_fifo_rden) begin//load ref
            //wren_ref <= 1;
            addr_ref <= addr_ref + 1;
        end 
        /*
        else begin
            wren_ref <= 0;
        end*/
    end
    LOAD_DONE: begin
        src_fifo_clear <= 0;
        addr_ref <= 0;
        //wren_ref <= 0;
        src_fifo_rden <= 0;
    end
    endcase
end
/*
always @(posedge clk)begin
    if(rst)
        qeu_mem_read_reg <= 2'd0;
    else
        qeu_mem_read_reg <= {qeu_mem_read_reg[0],qeu_mem_read};
end

always @(posedge clk)begin
    if(rst)
        read_mem_cnt <= 4'd0;
    else if(read_mem_cnt >= 4'd8)
        read_mem_cnt <= 4'd0;
    else if(qeu_mem_read_reg == 2'b10)
        read_mem_cnt <= read_mem_cnt + 1'b1;
end
*/
always @(posedge clk)begin
    if(rst)
        mem_r_flag <= 1'b0;
    else if(qeu_next == 1)
        mem_r_flag <= ~mem_r_flag;
end

always @(posedge clk) begin
    if(rst)
        mem_w_flag <= 0;
    else if(load_state == LOAD_DONE)
        mem_w_flag <= ~mem_w_flag;
end

always @(posedge clk)begin
    if(rst)
        mem1_busy <= 1'b0;
    else if(load_state == QEU_LOAD && mem_w_flag == 0)
        mem1_busy <= 1'b1;
    else if(qeu_next == 1 && mem_r_flag == 0)
        mem1_busy <= 1'b0;
end

always @(posedge clk)begin
    if(rst)
        mem2_busy <= 1'b0;
    else if(load_state == QEU_LOAD && mem_w_flag == 1)
        mem2_busy <= 1'b1;
    else if(qeu_next == 1 && mem_r_flag == 1)
        mem2_busy <= 1'b0;
end

always @(posedge clk)begin
    if(rst)
        mem1_load_done <= 1'b0;
    else if(load_state == LOAD_DONE && mem_w_flag == 0)
        mem1_load_done <= 1'b1;
    else if(qeu_next == 1 && mem_r_flag == 0)
        mem1_load_done <= 1'b0;
end

always @(posedge clk)begin
    if(rst)
        mem2_load_done <= 1'b0;
    else if(load_state == LOAD_DONE && mem_w_flag == 1)
        mem2_load_done <= 1'b1;
    else if(qeu_next == 1 && mem_r_flag == 1)
        mem2_load_done <= 1'b0;
end

endmodule
