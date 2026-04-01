`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 09/18/2016 09:25:01 PM
// Design Name: 
// Module Name: 6809 Superset module of MC6809 and MC6809E signals
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


module mc6809s(
    input   [7:0] D,
    output  [7:0] DOut,
    output  [15:0] ADDR,
    output  RnW,
    input   CLK4,
    output  BS,
    output  BA,
    input   nIRQ,
    input   nFIRQ,
    input   nNMI,
    output  AVMA,
    output  BUSY,
    output  LIC,
    input   nRESET,
    input   nHALT,
    input   nDMABREQ,
    output  E,
    output  Q,
    output reg [1:0] clk4_cnt,
    output  [111:0] RegData
);

    reg     rE;
    reg     rQ;
    assign  E = rE;
    assign  Q = rQ;
    reg     nCoreRESET;
    reg     rCE_E_FALL;
    reg     rCE_Q_FALL;
    reg [1:0] next_phase;
    
 mc6809i corecpu(.D(D), .DOut(DOut), .ADDR(ADDR), .RnW(RnW), .CLK_ROOT(CLK4), .CE_E_FALL(rCE_E_FALL), .CE_Q_FALL(rCE_Q_FALL), .BS(BS), .BA(BA), .nIRQ(nIRQ), .nFIRQ(nFIRQ), .nNMI(nNMI), .AVMA(AVMA), .BUSY(BUSY), .LIC(LIC), .nRESET(nCoreRESET),
                 .nDMABREQ(nDMABREQ), .nHALT(nHALT), .RegData(RegData) );
                 
 always @(negedge CLK4)
 begin
     rCE_E_FALL <= 1'b0;
     rCE_Q_FALL <= 1'b0;

     if (nRESET == 1'b0)
     begin
         clk4_cnt <= 2'b00;
         rE <= 1'b0;
         rQ <= 1'b0;
         nCoreRESET <= 1'b0;
     end
     else
     begin
         next_phase = clk4_cnt + 2'b01;
         case (next_phase)
             2'b00: begin
                 rE <= 1'b0;
                 rCE_E_FALL <= 1'b1;
             end
             2'b01:
                 rQ <= 1'b1;
             2'b10:
                 rE <= 1'b1;
             2'b11: begin
                 rQ <= 1'b0;
                 rCE_Q_FALL <= 1'b1;
                 nCoreRESET <= 1'b1;
             end
         endcase
         clk4_cnt <= next_phase;
     end
 end
       
    
endmodule
