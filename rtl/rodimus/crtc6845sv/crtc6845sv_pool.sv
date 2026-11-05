// chip_pool_pkg wrapper for the fleet master bus (mc6845_i_t / mc6845_o_t); CURSOR unused, LPSTB low.
module crtc6845sv_pool
(
    input  logic                     CLOCK,
    input  chip_pool_pkg::mc6845_i_t i,
    output chip_pool_pkg::mc6845_o_t o
);

wire  [7:0] dout;
wire        vsync, hsync, vblank, hblank, de;
wire [13:0] ma;
wire  [4:0] ra;

crtc6845sv_mc6845 crtc
(
    .CLOCK(CLOCK), .CLKEN(i.clken), .nRESET(i.reset_n), .ENABLE(i.enable), .R_nW(i.r_nw), .RS(i.rs), .DI(i.di),
    .DO(dout), .VSYNC(vsync), .HSYNC(hsync), .VBLANK(vblank), .HBLANK(hblank), .DE(de), .CURSOR(), .LPSTB(1'b0),
    .MA(ma), .RA(ra)
);

assign o.dout = dout;      assign o.vsync = vsync;     assign o.hsync = hsync;
assign o.vblank = vblank;  assign o.hblank = hblank;   assign o.de = de;
assign o.ma = ma;          assign o.ra = ra;

endmodule
