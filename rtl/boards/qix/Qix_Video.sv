//============================================================================
//
// Qix Video CPU Board
// Copyright (C) 2026 Rodimus
//
// Hardware: mc6809e + mc6845 CRTC + 64KB VRAM + 1KB palette RAM +
//           1KB NVRAM + 24KB ROM
//
// Responsibilities:
//   - Video CPU (6809E @ ~1.25 MHz, E/Q clocks supplied by top-level)
//   - Framebuffer writes (64KB VRAM at $0000-$7FFF)
//   - Palette management (qix_palette at $9000-$93FF, bank at $8800)
//   - CRTC (mc6845 via qix_display, accessed at $9C00-$9C01)
//   - Shared RAM port B ($8000-$83FF, wired externally to Data CPU)
//   - NVRAM ($8400-$87FF, hiscore interface)
//   - Cross-CPU FIRQ signaling
//
//============================================================================

module Qix_Video (
    input         clk_20m,
    input         reset,
    input         ce_E_fall,    // 6809 E falling edge enable (1 clk_20m pulse)
    input         ce_Q_fall,    // 6809 Q falling edge enable (1 clk_20m pulse)

    // Shared RAM interface (dual-port RAM lives in Qix.sv)
    output [9:0]  shared_addr,
    output [7:0]  shared_dout,
    input  [7:0]  shared_din,
    output        shared_we,
    output        shared_cs_o,

    // Cross-CPU FIRQ
    output        data_firq,    // pulse: video CPU asserts FIRQ on data CPU
    output        video_firq_ack,
    input         video_firq_n, // from data CPU → drives video 6809E nFIRQ

    // Video outputs
    output        hsync,
    output        vsync,
    output        hblank,
    output        vblank,
    output        ce_pix,
    output [7:0]  video_r,
    output [7:0]  video_g,
    output [7:0]  video_b,
    output        crtc_vsync,

    // Screen centering (OSD)
    input  [3:0]  h_center,
    input  [3:0]  v_center,

    // ROM loader (MiSTer ioctl — pre-gated by address range in Qix.sv)
    input  [24:0] ioctl_addr,
    input  [7:0]  ioctl_data,
    input         ioctl_wr,

    // Hiscore / NVRAM interface (hs_address driven by hiscore module)
    input  [15:0] hs_address,
    output [7:0]  hs_data_out,
    input  [7:0]  hs_data_in,
    input         hs_write,

    output chip_pool_pkg::m6809r_i_t m6809_i,
    input  chip_pool_pkg::m6809r_o_t m6809_o,

    input         pause,
    input         flip,
    input  [7:0]  game_id
);

// ---------------------------------------------------------------------------
// 6809E Video CPU
// ---------------------------------------------------------------------------
wire [15:0] cpu_A;
wire [7:0]  cpu_Dout;
wire        cpu_RnW;

assign m6809_i.reset_n = ~reset;
assign m6809_i.halt_n  = ~pause;
assign m6809_i.fall_e  = ce_E_fall;
assign m6809_i.fall_q  = ce_Q_fall;
assign m6809_i.irq_n   = 1'b1;
assign m6809_i.firq_n  = video_firq_n;
assign m6809_i.nmi_n   = 1'b1;
assign cpu_A    = m6809_o.a;
assign cpu_Dout = m6809_o.dout;
assign cpu_RnW  = m6809_o.rw;

// ---------------------------------------------------------------------------
// Write strobe: one-cycle pulse on falling edge of E while RnW is low
// ---------------------------------------------------------------------------
wire cpu_E_fall = ce_E_fall;
wire cpu_wr     = cpu_E_fall & ~cpu_RnW;

// ---------------------------------------------------------------------------
// Address decoder
// ---------------------------------------------------------------------------
wire vram_direct_cs  =  ~cpu_A[15];                                // $0000-$7FFF
wire shared_cs       = (cpu_A[15:10] == 6'b10_0000);               // $8000-$83FF
wire nvram_cs        = (cpu_A[15:10] == 6'b10_0001);               // $8400-$87FF
wire palbank_cs      = (cpu_A[15:10] == 6'b10_0010);               // $8800-$8BFF (VS2)
wire firq_range      = (cpu_A[15:10] == 6'b10_0011);               // $8C00-$8FFF (VS3)
wire firq_assert_cs  = firq_range & ~cpu_A[0];                     // even: assert
wire firq_ack_cs     = firq_range &  cpu_A[0];                     // odd: ack
wire palette_cs      = (cpu_A[15:10] == 6'b10_0100);               // $9000-$93FF

// VS5 chip select: $9400-$97FF (bits [9:2] don't-care, bits [1:0] select function)
wire vs5_cs          = (cpu_A[15:10] == 6'b10_0101);               // $9400-$97FF
wire vram_latch_cs   = vs5_cs & (cpu_A[1:0] == 2'b00);             // xx00: VRAM latch r/w
wire vram_mask_cs    = vs5_cs & (cpu_A[1:0] == 2'b01);             // xx01: VRAM access mask (Slither only)
wire latch_hi_cs     = vs5_cs & (cpu_A[1:0] == 2'b10);             // xx10: addr latch hi
wire latch_lo_cs     = vs5_cs & (cpu_A[1:0] == 2'b11);             // xx11: addr latch lo

wire scanline_cs     = (cpu_A[15:10] == 6'b10_0110);               // $9800 (partial)
wire crtc_range      = (cpu_A[15:10] == 6'b10_0111);               // $9C00-$9FFF (VS7)
wire crtc_bus_cs     = crtc_range;                                 // active for full range
wire rom_cs          = (cpu_A >= 16'hA000);                        // $A000-$FFFF (24KB)

// ---------------------------------------------------------------------------
// Shared RAM outputs (port B wired to dual-port RAM in Qix.sv)
// ---------------------------------------------------------------------------
assign shared_addr = cpu_A[9:0];
assign shared_dout = cpu_Dout;
assign shared_we   = shared_cs & cpu_wr;
assign shared_cs_o = shared_cs;

/// ---------------------------------------------------------------------------
// FIRQ access pulses (from schematic Figure 13, U7/U8):
//   $8C00 (even): assert FIRQ on data CPU
//   $8C01 (odd):  ack video CPU's own FIRQ
// One 20MHz cycle pulse at E-fall. SR latches live in Qix.sv.
// ---------------------------------------------------------------------------
reg firq_assert_pulse;
reg firq_ack_pulse;

always @(posedge clk_20m) begin
    if (reset) begin
        firq_assert_pulse <= 1'b0;
        firq_ack_pulse    <= 1'b0;
    end else begin
        firq_assert_pulse <= firq_assert_cs & cpu_E_fall;
        firq_ack_pulse    <= firq_ack_cs & cpu_E_fall;
    end
end

assign data_firq     = firq_assert_pulse;
assign video_firq_ack = firq_ack_pulse;

// ---------------------------------------------------------------------------
// Zoo Keeper ROM-bank latch (per qix_m.cpp zookeep_state::bankswitch_w)
//
//   $8800 (even mirror): palette bank only — already handled in qix_palette
//                        because palbank_cs covers the full $8800-$8BFF range.
//   $8801 (odd  mirror): videobank ← data[2], palette bank ← data[1:0]
//
// We only need a new latch for the video ROM bank. Gated by is_zookeep so
// non-Zoo Keeper games are unaffected.
// ---------------------------------------------------------------------------
wire is_zookeep = (game_id == 8'h04);

reg vid_bank;
always @(posedge clk_20m) begin
    if (reset)
        vid_bank <= 1'b0;
    else if (is_zookeep & palbank_cs & cpu_A[0] & cpu_wr)
        vid_bank <= cpu_Dout[2];
end

// ---------------------------------------------------------------------------
// VRAM address latch registers (written at $9402/$9403)
// ---------------------------------------------------------------------------
reg [7:0] vram_latch_addr_hi;
reg [7:0] vram_latch_addr_lo;

always @(posedge clk_20m) begin
    if (latch_hi_cs & cpu_wr) vram_latch_addr_hi <= cpu_Dout;
    if (latch_lo_cs & cpu_wr) vram_latch_addr_lo <= cpu_Dout;
end

// always @(posedge clk_20m) begin
//     if (reset) begin
//         vram_latch_addr_hi <= 8'h00;
//         vram_latch_addr_lo <= 8'h00;
//     end else begin
//         if (latch_hi_cs & cpu_wr) vram_latch_addr_hi <= cpu_Dout;
//         if (latch_lo_cs & cpu_wr) vram_latch_addr_lo <= cpu_Dout;
//     end
// end

// ---------------------------------------------------------------------------
// CRTC / display interconnects
// ---------------------------------------------------------------------------
wire [13:0] crtc_ma;
wire [4:0]  crtc_ra;
wire        crtc_de;
wire        vsync_cpu;
wire [15:0] display_addr;
wire [7:0]  display_data;
wire [7:0]  pixel_index;
wire [7:0]  pal_r, pal_g, pal_b;
wire [7:0]  crtc_do_w;

// ---------------------------------------------------------------------------
// qix_vram — 64KB dual-port framebuffer + address latch + scanline latch
// ---------------------------------------------------------------------------
wire [7:0] vram_dout;
wire [7:0] vram_latch_dout;
wire [7:0] scanline_latch;

// ---------------------------------------------------------------------------
// VRAM access mask ($9401) — SLITHER ONLY.
//
// Memory map: "100101--------01  R/W  video RAM access mask [2]" /
//             "[2] Slither only. This hardware feature is NOT present in the
//              Taito games."
// Every VRAM write is blended: vram = (vram & ~mask) | (data & mask).
// The video CPU writes it 39 times (via direct page, DP=$94, `STA <$01`), and
// NOT only with $FF — e.g. #$E0, plus the $E3FD/$E403 clear entry points which
// take the mask from the caller. Leaving it unimplemented writes whole bytes and
// corrupts partial-plane drawing during gameplay.
// ---------------------------------------------------------------------------
wire is_slither = (game_id == 8'h05);

reg [7:0] vram_mask = 8'hFF;
always @(posedge clk_20m) begin
    if (reset)                       vram_mask <= 8'hFF;
    else if (vram_mask_cs & cpu_wr)  vram_mask <= cpu_Dout;
end

qix_vram vram (
    .clk            (clk_20m),
    .flip           (flip),

    // Slither VRAM access mask. Set (1'b0) here to disable it and fall back to
    // whole-byte writes for every game.
    .mask_en        (is_slither),
    .vram_mask      (vram_mask),

    // CPU direct port ($0000-$7FFF)
    .addr           (cpu_A[14:0]),
    .we             (vram_direct_cs & cpu_wr),
    .din            (cpu_Dout),
    .dout           (vram_dout),

    // CPU latched port ($9400)
    .latch_addr_hi  (vram_latch_addr_hi),
    .latch_addr_lo  (vram_latch_addr_lo),
    .latch_we       (vram_latch_cs & cpu_wr),
    .latch_re       (vram_latch_cs & ~cpu_wr),
    .latch_din      (cpu_Dout),
    .latch_dout     (vram_latch_dout),

    // Display scanout
    .display_addr   (display_addr),
    .display_dout   (display_data),

    // Scanline latch inputs from CRTC
    .crtc_ma        (crtc_ma),
    .crtc_ra        (crtc_ra),
    .crtc_de        (crtc_de),
    .scanline_latch (scanline_latch)
);

// ---------------------------------------------------------------------------
// qix_palette — 1KB palette RAM + RRGGBBII decoder
// ---------------------------------------------------------------------------
wire [7:0] pal_cpu_dout;

qix_palette palette (
    .clk         (clk_20m),

    .cpu_addr    (cpu_A[9:0]),
    .cpu_we      (palette_cs & cpu_wr),
    .cpu_din     (cpu_Dout),
    .cpu_dout    (pal_cpu_dout),

    .bank_we     (palbank_cs & cpu_wr),
    .bank_din    (cpu_Dout[1:0]),

    .pixel_index (pixel_index),
    .rgb_r       (pal_r),
    .rgb_g       (pal_g),
    .rgb_b       (pal_b)
);

// ---------------------------------------------------------------------------
// qix_display — CRTC + scanout pipeline
// ---------------------------------------------------------------------------
qix_display display (
    .clk_20m     (clk_20m),
    .reset       (reset),
    .flip        (flip),

    .h_center    (h_center),
    .v_center    (v_center),

    .ce_pix      (ce_pix),
    .hsync       (hsync),
    .vsync       (vsync),
    .vsync_cpu   (vsync_cpu),
    .hblank      (hblank),
    .vblank      (vblank),

    .crtc_ma     (crtc_ma),
    .crtc_ra     (crtc_ra),
    .crtc_de     (crtc_de),

    .display_addr (display_addr),
    .display_data (display_data),

    .pixel_index (pixel_index),
    .rgb_r       (pal_r),
    .rgb_g       (pal_g),
    .rgb_b       (pal_b),

    .video_r     (video_r),
    .video_g     (video_g),
    .video_b     (video_b),

    .crtc_cs     (crtc_bus_cs),
    .crtc_rw     (cpu_RnW),
    .crtc_rs     (cpu_A[0]),
    .crtc_di     (cpu_Dout),
    .crtc_do     (crtc_do_w)
);

assign crtc_vsync = vsync_cpu;

// ---------------------------------------------------------------------------
// NVRAM — 1KB BRAM ($8400-$87FF)
// Port A: Video CPU  |  Port B: hiscore framework
// Explicit dpram_dc for reliable M10K inference.
// ---------------------------------------------------------------------------
wire [7:0] nvram_cpu_dout;
wire [7:0] nvram_hs_dout;

dpram_dc #(.widthad_a(10)) nvram_inst (
    .clock_a    (clk_20m),
    .address_a  (cpu_A[9:0]),
    .data_a     (cpu_Dout),
    .wren_a     (nvram_cs & cpu_wr),
    .q_a        (nvram_cpu_dout),

    .clock_b    (clk_20m),
    .address_b  (hs_address[9:0]),
    .data_b     (hs_data_in),
    .wren_b     (hs_write),
    .q_b        (nvram_hs_dout)
);

assign hs_data_out = nvram_hs_dout;

// ---------------------------------------------------------------------------
// Video ROM — sized for the largest variant on the platform.
//
//   Qix / non-banked: 24KB at $A000-$FFFF      (loaded ioctl $06000-$0BFFF)
//   Zoo Keeper:       32KB total — 8KB bank0 + 16KB fixed + 8KB bank1
//                                                (loaded ioctl $08000-$0FFFF)
//
// BRAM layout for Zoo Keeper:
//   index $0000-$1FFF : bank 0 of $A000-$BFFF  (8KB)
//   index $2000-$5FFF : fixed  $C000-$FFFF     (16KB)
//   index $6000-$7FFF : bank 1 of $A000-$BFFF  (8KB)
//
// CPU read-address mux:
//   Non-Zook:           cpu_A[14:0] - $2000
//   Zook $A000-$BFFF:   bank0 → $0000 + cpu_A[12:0]
//                       bank1 → $6000 + cpu_A[12:0]
//   Zook $C000-$FFFF:   $2000 + cpu_A[13:0]
// ---------------------------------------------------------------------------
reg [7:0] vid_rom [0:32767];                         // 32KB
reg [7:0] rom_dout;

// Zoo Keeper BRAM-index decode for $A000-$FFFF reads:
//   $A000-$BFFF, bank 0  → BRAM $0000-$1FFF  (idx = {2'b00, cpu_A[12:0]})
//   $A000-$BFFF, bank 1  → BRAM $6000-$7FFF  (idx = {2'b11, cpu_A[12:0]})
//   $C000-$FFFF, fixed   → BRAM $2000-$5FFF  (idx = {1'b0, cpu_A[14], cpu_A[13:0]} 
//                                              i.e. bit[14]=0, bit[13]=cpu_A[14] which =1, bits=cpu_A[13:0])
//                                              Simpler: just $2000 + cpu_A[13:0].
reg [14:0] zook_rom_addr;
always @* begin
    if (cpu_A[14] == 1'b1)                        // $C000-$FFFF (fixed)
        zook_rom_addr = 15'h2000 + {1'b0, cpu_A[13:0]};
    else if (vid_bank == 1'b0)                    // $A000-$BFFF bank 0
        zook_rom_addr = {2'b00, cpu_A[12:0]};
    else                                          // $A000-$BFFF bank 1
        zook_rom_addr = {2'b11, cpu_A[12:0]};
end

wire [14:0] rom_cpu_addr = is_zookeep ? zook_rom_addr
                                      : (cpu_A[14:0] - 15'h2000);

// Non-Zook range starts at ioctl $06000 — subtract to base BRAM at 0.
// Zoo Keeper range starts at ioctl $08000 — low 15 bits are already 0-based.
wire [14:0] rom_ioctl_addr = is_zookeep ? ioctl_addr[14:0]
                                        : (ioctl_addr[14:0] - 15'h6000);

always @(posedge clk_20m) begin
    if (ioctl_wr)
        vid_rom[rom_ioctl_addr] <= ioctl_data;
    rom_dout <= vid_rom[rom_cpu_addr];
end

// ---------------------------------------------------------------------------
// CPU data bus read mux — default $FF for open-bus / unimplemented reads
// ---------------------------------------------------------------------------
wire [7:0] cpu_Din =
    vram_direct_cs ? vram_dout            :
    shared_cs      ? shared_din           :
    nvram_cs       ? nvram_cpu_dout       :
    palbank_cs     ? 8'h00                :  // Test
    palette_cs     ? pal_cpu_dout         :
    vram_latch_cs  ? vram_latch_dout      :
    vram_mask_cs   ? vram_mask            :   // $9401 is R/W per the memory map
//    latch_hi_cs    ? vram_latch_addr_hi   :  // Test
//    latch_lo_cs    ? vram_latch_addr_lo   :  // Test
    scanline_cs    ? scanline_latch       :
    crtc_bus_cs    ? crtc_do_w            :
    rom_cs         ? rom_dout             :
    8'hFF;

assign m6809_i.d = cpu_Din;

endmodule
