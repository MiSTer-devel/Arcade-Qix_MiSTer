//  Interface and timing after John E. Kent's pia6821.vhd

module pia6821
(
    input  logic       clk,
    input  logic       rst,
    input  logic       ce,
    input  logic       cs,
    input  logic       rw,          // 1 = read
    input  logic [1:0] rs,
    input  logic [7:0] din,
    output logic [7:0] dout,
    output logic       irqa,
    output logic       irqb,

    input  logic [7:0] pa_i,
    input  logic       pa_pins,
    output logic [7:0] pa_o,
    output logic [7:0] pa_oe,
    input  logic       ca1,
    input  logic       ca2_i,
    output logic       ca2_o,
    output logic       ca2_oe,

    input  logic [7:0] pb_i,
    output logic [7:0] pb_o,
    output logic [7:0] pb_oe,
    input  logic       cb1,
    input  logic       cb2_i,
    output logic       cb2_o,
    output logic       cb2_oe
);

logic [7:0] ddra, ora, ddrb, orb;
logic [5:0] cra, crb;
logic       irqa1, irqa2, irqb1, irqb2;
logic       ca2_out, cb2_out;
logic       ca2_pulse, cb2_pulse;
logic       ca1_d, ca2_d, cb1_d, cb2_d;

wire wr_en = ce & cs & ~rw;
wire rd_en = ce & cs &  rw;

wire rd_pa  = rd_en & (rs == 2'd0) & cra[2];
wire rd_pb  = rd_en & (rs == 2'd2) & crb[2];
wire wr_pb  = wr_en & (rs == 2'd2) & crb[2];

wire ca1_edge = cra[1] ? (ca1 & ~ca1_d)     : (~ca1 & ca1_d);
wire cb1_edge = crb[1] ? (cb1 & ~cb1_d)     : (~cb1 & cb1_d);
wire ca2_edge = cra[4] ? (ca2_i & ~ca2_d)   : (~ca2_i & ca2_d);
wire cb2_edge = crb[4] ? (cb2_i & ~cb2_d)   : (~cb2_i & cb2_d);

always_ff @(posedge clk) begin
    if (rst) begin
        ddra <= 8'h00; ora <= 8'h00; cra <= 6'h00;
        ddrb <= 8'h00; orb <= 8'h00; crb <= 6'h00;
        irqa1 <= 1'b0; irqa2 <= 1'b0; irqb1 <= 1'b0; irqb2 <= 1'b0;
        ca2_out <= 1'b1; cb2_out <= 1'b1;
        ca2_pulse <= 1'b0; cb2_pulse <= 1'b0;
        ca1_d <= ca1; ca2_d <= ca2_i; cb1_d <= cb1; cb2_d <= cb2_i;
    end else begin
        ca1_d <= ca1; ca2_d <= ca2_i; cb1_d <= cb1; cb2_d <= cb2_i;

        if (rd_pa) begin irqa1 <= 1'b0; irqa2 <= 1'b0; end
        if (rd_pb) begin irqb1 <= 1'b0; irqb2 <= 1'b0; end
        if (ca1_edge)            irqa1 <= 1'b1;
        if (cb1_edge)            irqb1 <= 1'b1;
        if (~cra[5] & ca2_edge)  irqa2 <= 1'b1;
        if (~crb[5] & cb2_edge)  irqb2 <= 1'b1;

        if (cra[5] & ~cra[4]) begin
            if (ce & ca2_pulse) begin ca2_out <= 1'b1; ca2_pulse <= 1'b0; end
            if (~cra[3] & ca1_edge) ca2_out <= 1'b1;
            if (rd_pa) begin ca2_out <= 1'b0; ca2_pulse <= cra[3]; end
        end
        if (crb[5] & ~crb[4]) begin
            if (ce & cb2_pulse) begin cb2_out <= 1'b1; cb2_pulse <= 1'b0; end
            if (~crb[3] & cb1_edge) cb2_out <= 1'b1;
            if (wr_pb) begin cb2_out <= 1'b0; cb2_pulse <= crb[3]; end
        end

        if (wr_en) begin
            case (rs)
                2'd0: if (cra[2]) ora <= din; else ddra <= din;
                2'd1: begin
                    cra <= din[5:0];
                    if (din[5]) begin
                        ca2_out   <= din[4] ? din[3] : 1'b1;
                        ca2_pulse <= 1'b0;
                        irqa2     <= 1'b0;
                    end
                end
                2'd2: if (crb[2]) orb <= din; else ddrb <= din;
                2'd3: begin
                    crb <= din[5:0];
                    if (din[5]) begin
                        cb2_out   <= din[4] ? din[3] : 1'b1;
                        cb2_pulse <= 1'b0;
                        irqb2     <= 1'b0;
                    end
                end
            endcase
        end
    end
end

assign pa_o  = ora;
assign pa_oe = ddra;
assign pb_o  = orb;
assign pb_oe = ddrb;

assign ca2_o  = ca2_out;
assign ca2_oe = cra[5];
assign cb2_o  = cb2_out;
assign cb2_oe = crb[5];

always_comb begin
    case (rs)
        2'd0:    dout = cra[2] ? (pa_pins ? pa_i : (ora & ddra) | (pa_i & ~ddra)) : ddra;
        2'd1:    dout = {irqa1, irqa2 & ~cra[5], cra};
        2'd2:    dout = crb[2] ? ((orb & ddrb) | (pb_i & ~ddrb)) : ddrb;
        default: dout = {irqb1, irqb2 & ~crb[5], crb};
    endcase
end

assign irqa = (irqa1 & cra[0]) | (irqa2 & cra[3] & ~cra[5]);
assign irqb = (irqb1 & crb[0]) | (irqb2 & crb[3] & ~crb[5]);

endmodule
