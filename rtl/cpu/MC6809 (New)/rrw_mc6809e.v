`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: Chuck Benedict
// 
// Create Date:    2026-02-21
// Design Name: 
// Module Name:    rrw_mc6809e 
// Project Name: 
// Target Devices: 
// Tool versions: 
// Description: This module breaks out pins for the MC6809E so that they can be used 
// as if the model were a real part. Compatible with the RRW tool.
//
// Dependencies: 
//
// Revision: 
// Revision 1.0 - File Created
// Additional Comments: 
//
//////////////////////////////////////////////////////////////////////////////////
module rrw_mc6809e(
    output  A0,
    output  A1,
    output  A2,
    output  A3,
    output  A4,
    output  A5,
    output  A6,
    output  A7,
    output  A8,
    output  A9,
    output  A10,
    output  A11,
    output  A12,
    output  A13,
    output  A14,
    output  A15,
    input   DIn7,
    input   DIn6,
    input   DIn5,
    input   DIn4,
    input   DIn3,
    input   DIn2,
    input   DIn1,
    input   DIn0,
    output  DOut0,
    output  DOut1,
    output  DOut2,
    output  DOut3,
    output  DOut4,
    output  DOut5,
    output  DOut6,
    output  DOut7,
    output  DOE0,
    output  DOE1,
    output  DOE2,
    output  DOE3,
    output  DOE4,
    output  DOE5,
    output  DOE6,
    output  DOE7,
    output  RnW,
    input   CLK_ROOT,
    input   CE_E_FALL,
    input   CE_Q_FALL,
    output  BS,
    output  BA,
    input   nIRQ,
    input   nFIRQ,
    input   nNMI,
    output  AVMA,
    output  BUSY,
    output  LIC,
    input 	nHALT,	 
    input   nRESET,
    output  [15:0] A
    );

wire [7:0] D;
assign D = {DIn7, DIn6, DIn5, DIn4, DIn3, DIn2, DIn1, DIn0};

wire [7:0] DOut;
assign {DOut7, DOut6, DOut5, DOut4, DOut3, DOut2, DOut1, DOut0} = DOut;

wire [15:0] ADDR;
assign {A15, A14, A13, A12, A11, A10, A9, A8, A7, A6, A5, A4, A3, A2, A1, A0} = ADDR;
assign A = ADDR;

wire OE = (~RnW) && (~BA) && AVMA;
assign DOE0 = OE;
assign DOE1 = OE;
assign DOE2 = OE;
assign DOE3 = OE;
assign DOE4 = OE;
assign DOE5 = OE;
assign DOE6 = OE;
assign DOE7 = OE;

mc6809e cpucore (.D(D), .DOut(DOut), .ADDR(ADDR), .RnW(RnW), .BS(BS), .BA(BA), .nIRQ(nIRQ), .nFIRQ(nFIRQ),
                .nNMI(nNMI), .AVMA(AVMA), .BUSY(BUSY), .LIC(LIC), .nHALT(nHALT), .nRESET(nRESET),
                .CLK_ROOT(CLK_ROOT), .CE_E_FALL(CE_E_FALL), .CE_Q_FALL(CE_Q_FALL)
                );

endmodule
