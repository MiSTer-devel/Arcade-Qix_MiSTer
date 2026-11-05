// qix_vram.sv — 64KB Dual-Port Framebuffer, Address Latch, Scanline Latch
//
// Port A (CPU): muxed between direct ($0000-$7FFF) and latched ($9400) access.
// Port B (Display): read-only scanout with optional cocktail-flip.
// Scanline latch: captured on rising edge of crtc_de.
//
// Storage is a 64K RAM pool block in the dispatcher (dpram_dc = altsyncram, M10K).

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
    output reg [7:0]  scanline_latch,

    // Storage: RAM pool block (64K) in the dispatcher
    output ram_pool_pkg::pram_i_t pr_i,
    input  ram_pool_pkg::pram_o_t pr_o
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
// Two hard constraints (both broke the hardware when ignored):
//
//  1. Never drive the write data combinationally from cpu_q. A comb path out of the RAM straight back into its own
//     write port stops Quartus treating it as a real M10K, and the framebuffer becomes coloured jagged lines on
//     every game. See [[BRAM read mux in data path defeats inference]]. The old byte must come from a register.
//
//  2. Never just delay the write enable. cpu_wr is `cpu_E_fall & ~cpu_RnW`, a one-clock pulse (Qix_Video.sv), not a
//     held window. A clock later the 6809E has moved cpu_A to the next bus cycle and latch_we has dropped, so
//     cpu_addr_mux changes source: a delayed write lands at the wrong address.
//
// No state machine and no address mux is needed: the RAM is already reading the write address for the whole bus
// cycle before the write lands.
//   latched port - latch_re holds cpu_latch_full on the address all cycle
//   direct  port - latch_re / latch_we are 0, so the address = cpu_direct_full
// So cpu_q holds the old byte well before cpu_wr pulses at E-fall (16 clk in). One registered copy of it (rmw_old,
// constraint 1) is all the blend needs, and the write happens at its natural time.
reg [7:0] rmw_old = 8'd0;
always @(posedge clk) rmw_old <= cpu_q;

wire [15:0] cpu_addr_eff = cpu_addr_mux;
wire [7:0]  cpu_din_eff  = mask_en ? ((rmw_old & ~vram_mask) | (cpu_din_mux & vram_mask))
                                   : cpu_din_mux;
wire        cpu_we_eff   = cpu_we_any;

// Display address with optional cocktail flip
wire [15:0] disp_addr_eff = flip ? (display_addr ^ 16'hFFFF) : display_addr;

// ---------------------------------------------------------------------------
// 64KB dual-port RAM (pool block)
//   Port A: CPU read/write
//   Port B: Display read-only
// ---------------------------------------------------------------------------
wire [7:0] cpu_q;
wire [7:0] disp_q;

assign pr_i.a_addr = cpu_addr_eff;
assign pr_i.a_d    = cpu_din_eff;
assign pr_i.a_we   = cpu_we_eff;
assign pr_i.b_addr = disp_addr_eff;
assign pr_i.b_d    = 8'd0;
assign pr_i.b_we   = 1'b0;
assign cpu_q       = pr_o.a_q;
assign disp_q      = pr_o.b_q;

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