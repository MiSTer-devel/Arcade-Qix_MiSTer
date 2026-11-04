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

    // Slither VRAM access mask ($9401). mask_en=0 => plain whole-byte writes.
    input             mask_en,
    input  [7:0]      vram_mask,

    // CPU direct access
    input  [14:0]     addr,
    input             we,
    input  [7:0]      din,
    output [7:0]      dout,

    // CPU latched access
    input  [7:0]      latch_addr_hi,
    input  [7:0]      latch_addr_lo,
    input             latch_we,
    input             latch_re,
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

wire [15:0] cpu_addr_mux  = (latch_we | latch_re) ? cpu_latch_full  : cpu_direct_full;
wire [7:0]  cpu_din_mux   = latch_we ? latch_din        : din;
wire        cpu_we_any    = we | latch_we;

// ---------------------------------------------------------------------------
// Slither VRAM access mask — read-modify-write blend
//   vram = (vram & ~mask) | (data & mask)
//
// cpu_q is the registered BRAM output for cpu_addr_mux, so the OLD byte is only
// valid a couple of clk after the address settles. The CPU write window is ~16
// clk_20m (6809E at 1.25 MHz), so delaying the write-enable by 2 clk is free and
// keeps CPU timing untouched.
//
// The blend is idempotent — ((old&~m)|(d&m))&~m | (d&m) == (old&~m)|(d&m) — so
// it stays correct even if the write strobe is a level and the RAM re-blends its
// own output for the rest of the window.
// ---------------------------------------------------------------------------
// Two hard constraints, both learned the hard way on HW 2026-08-23:
//
//  1. ⛔ NEVER drive data_a combinationally from cpu_q. That is a comb path out
//     of the RAM straight back into its own write port; Quartus stops treating
//     this as a real M10K and the framebuffer becomes coloured jagged lines on
//     EVERY game. See [[BRAM read mux in data path defeats inference]].
//     The old byte must come from a register.
//
//  2. ⛔ NEVER just delay wren_a. cpu_wr is `cpu_E_fall & ~cpu_RnW` — a ONE-CLOCK
//     pulse (Qix_Video.sv:96), not a held window. A clock later the 6809E has
//     moved cpu_A to the next bus cycle and latch_we has dropped, so cpu_addr_mux
//     changes source. A delayed write lands at the WRONG ADDRESS.
//
// So the sequencer captures address and data at the write pulse and owns the RAM
// port for four clocks. The CPU's next access is >=16 clk away (1.25 MHz core vs
// 20 MHz fabric), so hijacking the port is free.
//
//   st0  write pulse -> capture addr + data
//   st1  drive captured addr (read)
//   st2  q_a valid -> rmw_old
//   st3  drive captured addr, wren=1, data = blend
// No state machine and no address mux is needed: the RAM is ALREADY reading the
// write address for the whole bus cycle before the write lands.
//   latched port - latch_re holds cpu_latch_full on address_a all cycle
//   direct  port - latch_re/latch_we are 0, so address_a = cpu_direct_full
// So cpu_q holds the OLD byte well before cpu_wr pulses at E-fall (16 clk in).
// One registered copy of it is all the blend needs, and the write happens at its
// natural time.
//
// ⛔ rmw_old MUST stay a register. Blending straight from cpu_q is a comb path
// out of the RAM back into its own write port; Quartus stops inferring M10K and
// the framebuffer becomes coloured jagged lines on EVERY game (HW 2026-08-23).
// See [[BRAM read mux in data path defeats inference]].
reg [7:0] rmw_old = 8'd0;
always @(posedge clk) rmw_old <= cpu_q;

wire [15:0] cpu_addr_eff = cpu_addr_mux;
wire [7:0]  cpu_din_eff  = mask_en ? ((rmw_old & ~vram_mask) | (cpu_din_mux & vram_mask))
                                   : cpu_din_mux;
wire        cpu_we_eff   = cpu_we_any;

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
    .address_a  (cpu_addr_eff),
    .data_a     (cpu_din_eff),
    .wren_a     (cpu_we_eff),
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
reg crtc_de_r;
always @(posedge clk) begin
    crtc_de_r <= crtc_de;
    if (crtc_de && !crtc_de_r)
        scanline_latch <= {crtc_ma[9:5], crtc_ra[2:0]};
end

//assign scanline_latch = {crtc_ma[9:5], crtc_ra[2:0]};

endmodule