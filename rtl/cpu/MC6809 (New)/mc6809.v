`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date:    08:11:34 09/23/2016 
// Design Name: 
// Module Name:    mc6809e 
// Project Name: 
// Target Devices: 
// Tool versions: 
// Description: 
//
// Dependencies: 
//
// Revision: 
// Revision 0.01 - File Created
// Additional Comments: 
//
//////////////////////////////////////////////////////////////////////////////////
module mc6809(
    input   [7:0] D,
    output  [7:0] DOut,
    output  [15:0] ADDR,
    output  RnW,
    output  E,
    output  Q,
    output  BS,
    output  BA,
    input   nIRQ,
    input   nFIRQ,
    input   nNMI,
    input   EXTAL,
    input   XTAL,
    input   nHALT,	 
    input   nRESET,
    input   MRDY,
    input   nDMABREQ
    
    , output  [111:0] RegData

    );

reg [1:0] clk_phase=2'b00;

wire CLK;
assign CLK=EXTAL;

wire   LIC;
wire   BUSY;
wire   AVMA;
reg    rE;
reg    rQ;
assign E = rE;
assign Q = rQ;
reg    rCE_E_FALL;
reg    rCE_Q_FALL;
reg    hold_mrdy;
reg    [1:0] next_phase;

mc6809i cpucore(.D(D), .DOut(DOut), .ADDR(ADDR), .RnW(RnW), .CLK_ROOT(CLK), .CE_E_FALL(rCE_E_FALL), .CE_Q_FALL(rCE_Q_FALL), .BS(BS), .BA(BA), .nIRQ(nIRQ), .nFIRQ(nFIRQ), 
                .nNMI(nNMI), .AVMA(AVMA), .BUSY(BUSY), .LIC(LIC), .nHALT(nHALT), .nRESET(nRESET), .nDMABREQ(nDMABREQ)
                ,.RegData(RegData)
                );

always @(negedge CLK)
begin
    // M6809PM 1.11.11: MRDY stretches only valid memory cycles, specifically E=1/Q=0.
    hold_mrdy = (MRDY == 1'b0) && (AVMA == 1'b1) && (clk_phase == 2'b11);
    next_phase = clk_phase + 2'b01;

    rCE_E_FALL <= 1'b0;
    rCE_Q_FALL <= 1'b0;

    if (!hold_mrdy) begin
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
            end
        end
        clk_phase <= next_phase;
    end
end


endmodule
