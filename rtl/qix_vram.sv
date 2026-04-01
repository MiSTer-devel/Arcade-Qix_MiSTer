// qix_vram.sv — 64KB Dual-Port Framebuffer, Address Latch, Scanline Latch
//
// Port A (CPU): muxed between direct ($0000-$7FFF) and latched ($9400) access.
// Port B (Display): read-only scanout with optional cocktail-flip.
// Scanline latch: captured on rising edge of crtc_de.
//
// Uses explicit dpram_dc (altsyncram) to guarantee M10K inference.

module qix_vram (
    input             clk,
    input             flip,

    // CPU direct access
    input  [14:0]     addr,
    input             we,
    input  [7:0]      din,
    output [7:0]      dout,

    // CPU latched access
    input  [7:0]      latch_addr_hi,
    input  [7:0]      latch_addr_lo,
    input             latch_we,
    input  [7:0]      latch_din,
    output [7:0]      latch_dout,

    // Display scanout
    input  [15:0]     display_addr,
    output [7:0]      display_dout,

    // Scanline latch inputs
    input  [13:0]     crtc_ma,
    input  [4:0]      crtc_ra,
    input             crtc_de,
    output reg [7:0]  scanline_latch
);

// ---------------------------------------------------------------------------
// Address mux — CPU direct vs latched (mutually exclusive in caller)
// ---------------------------------------------------------------------------
wire [15:0] cpu_direct_full = {latch_addr_hi[7], addr};
wire [15:0] cpu_latch_full  = {latch_addr_hi, latch_addr_lo};

wire [15:0] cpu_addr_mux  = latch_we ? cpu_latch_full  : cpu_direct_full;
wire [7:0]  cpu_din_mux   = latch_we ? latch_din        : din;
wire        cpu_we_any    = we | latch_we;

// Display address with optional cocktail flip
wire [15:0] disp_addr_eff = flip ? (display_addr ^ 16'hFFFF) : display_addr;

// ---------------------------------------------------------------------------
// 64KB true dual-port RAM via explicit altsyncram wrapper
//   Port A: CPU read/write
//   Port B: Display read-only
// ---------------------------------------------------------------------------
wire [7:0] cpu_q;
wire [7:0] disp_q;

dpram_dc #(.widthad_a(16)) vram_inst (
    .clock_a    (clk),
    .address_a  (cpu_addr_mux),
    .data_a     (cpu_din_mux),
    .wren_a     (cpu_we_any),
    .q_a        (cpu_q),

    .clock_b    (clk),
    .address_b  (disp_addr_eff),
    .data_b     (8'd0),
    .wren_b     (1'b0),
    .q_b        (disp_q)
);

// Both CPU read paths share the same port A output
assign dout         = cpu_q;
assign latch_dout   = cpu_q;
assign display_dout = disp_q;

// ---------------------------------------------------------------------------
// Scanline latch — capture on rising edge of crtc_de
// ---------------------------------------------------------------------------
//reg crtc_de_r;
//always @(posedge clk) begin
//    crtc_de_r <= crtc_de;
//    if (crtc_de && !crtc_de_r)
//        scanline_latch <= {crtc_ma[9:5], crtc_ra[2:0]};
//end


assign scanline_latch = {crtc_ma[9:5], crtc_ra[2:0]};

endmodule