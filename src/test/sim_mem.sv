`ifndef SIM_MEM_SV
`define SIM_MEM_SV

// Fake memory model for iFetch simulation.
// Responds to MemController_Req with instruction data from a flat array.
// The array is filled by the C++ testbench via DPI before simulation starts.
//
// Also contains a fake page walker that responds to PageWalk_Req by doing
// a trivial identity mapping (VA == PA). Virtual memory is disabled in
// most tests so this is only exercised by Test 3.

`timescale 1ns/1ps

// DPI imports — implemented in testbench.cpp
import "DPI-C" function void dpi_mem_read(
    input  longint unsigned addr,
    output int unsigned     data
);

import "DPI-C" function void dpi_mem_write(
    input longint unsigned addr,
    input int unsigned     data
);

import "DPI-C" function void dpi_check_instr(
    input int unsigned cycle,
    input int unsigned slot,
    input int unsigned pc,
    input int unsigned bits,
    input int unsigned fetchID,
    input bit          valid,
    input bit          fault
);

import "DPI-C" function void dpi_test_done(
    input int unsigned test_num,
    input bit          passed
);

// ----------------------------------------------------------------
//  MemController_Req / MemController_Res stub
//  Matches the Chisel-generated port names for iFetch.
// ----------------------------------------------------------------
module SimMemController
#(
    parameter WORD_W    = 32,
    parameter ADDR_W    = 32,
    parameter BURST_LEN = 8        // cache line = 8 x 32-bit words
)
(
    input  wire                 clk,
    input  wire                 rst,

    // MemController_Req (from iFetch)
    input  wire                 req_valid,
    input  wire [ADDR_W-1:0]    req_addr,

    // MemController_Res (to iFetch)
    output reg                  res_valid,
    output reg  [WORD_W-1:0]    res_data,
    output reg                  res_done,
    output reg                  res_busy
);

    // Simple latency model: INIT_LAT cycles for first word, then
    // one word per cycle (burst).
    localparam INIT_LAT = 4;

    typedef enum logic [1:0] {
        MEM_IDLE,
        MEM_WAIT,
        MEM_BURST
    } mem_state_t;

    mem_state_t          state;
    reg [ADDR_W-1:0]     base_addr;
    reg [$clog2(BURST_LEN):0] burst_cnt;
    reg [$clog2(INIT_LAT):0]  lat_cnt;

    always_ff @(posedge clk or posedge rst) begin
        if (rst) begin
            state     <= MEM_IDLE;
            res_valid <= 0;
            res_done  <= 0;
            res_busy  <= 0;
            burst_cnt <= 0;
            lat_cnt   <= 0;
        end else begin
            res_valid <= 0;
            res_done  <= 0;

            case (state)
                MEM_IDLE: begin
                    res_busy <= 0;
                    if (req_valid) begin
                        base_addr <= {req_addr[ADDR_W-1:2], 2'b00};
                        lat_cnt   <= INIT_LAT - 1;
                        burst_cnt <= 0;
                        res_busy  <= 1;
                        state     <= MEM_WAIT;
                    end
                end

                MEM_WAIT: begin
                    res_busy <= 1;
                    if (lat_cnt == 0)
                        state <= MEM_BURST;
                    else
                        lat_cnt <= lat_cnt - 1;
                end

                MEM_BURST: begin
                    res_busy <= 1;
                    begin
                        automatic longint unsigned word_addr =
                            base_addr + (burst_cnt << 2);
                        automatic int unsigned rdata;
                        dpi_mem_read(word_addr, rdata);
                        res_data  <= rdata;
                        res_valid <= 1;
                    end

                    if (burst_cnt == BURST_LEN - 1) begin
                        res_done  <= 1;
                        res_busy  <= 0;
                        burst_cnt <= 0;
                        state     <= MEM_IDLE;
                    end else begin
                        burst_cnt <= burst_cnt + 1;
                    end
                end
            endcase
        end
    end
endmodule


// ----------------------------------------------------------------
//  Fake TLB page walker — identity mapping, zero latency.
//  Activated only when vmem is enabled in Test 3.
// ----------------------------------------------------------------
module SimPageWalker
#(
    parameter ADDR_W = 32
)
(
    input  wire              clk,
    input  wire              rst,

    // PageWalk_Req (from iFetch)
    input  wire              pw_req_valid,
    input  wire [19:0]       pw_req_vpn,
    input  wire [1:0]        pw_req_id,

    // PageWalk_Res (to iFetch)
    output reg               pw_res_valid,
    output reg               pw_res_busy,
    output reg [1:0]         pw_res_rqID,
    output reg [19:0]        pw_res_vpn,
    output reg [21:0]        pw_res_ppn,
    output reg [2:0]         pw_res_rwx,
    output reg               pw_res_user,
    output reg               pw_res_globl,
    output reg               pw_res_isSuperPage,
    output reg               pw_res_pageFault
);
    // One cycle latency identity walk
    always_ff @(posedge clk or posedge rst) begin
        if (rst) begin
            pw_res_valid <= 0;
            pw_res_busy  <= 0;
        end else begin
            pw_res_valid      <= pw_req_valid;
            pw_res_busy       <= pw_req_valid;
            pw_res_rqID       <= pw_req_id;
            pw_res_vpn        <= pw_req_vpn;
            pw_res_ppn        <= {2'b00, pw_req_vpn};  // identity: PA == VA
            pw_res_rwx        <= 3'b101;               // R+X
            pw_res_user       <= 1'b1;
            pw_res_globl      <= 1'b0;
            pw_res_isSuperPage<= 1'b0;
            pw_res_pageFault  <= 1'b0;
        end
    end
endmodule

`endif
