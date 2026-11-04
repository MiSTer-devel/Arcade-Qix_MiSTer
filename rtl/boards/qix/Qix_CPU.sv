//============================================================================
//
// Qix Data CPU Board
// Copyright (C) 2026 Rodimus
//
// Hardware: mc6809e + 3× PIA6821 + 1× sndPIA0 + 1KB local RAM + 24KB ROM
//
// Responsibilities:
//   - Game logic CPU (6809E @ ~1.25 MHz, E/Q clocks supplied by top-level)
//   - Player 1/2 inputs + coin via PIAs
//   - Audio communication via sndPIA0
//   - Communicates with Video CPU via shared RAM port A + FIRQ cross-signals
//
//============================================================================

module Qix_CPU (
    input         clk_20m,
    input         reset,
    input         ce_E_fall,      // 6809 E falling edge enable (1 clk_20m pulse)
    input         ce_Q_fall,      // 6809 Q falling edge enable (1 clk_20m pulse)

    // Shared RAM — port A of dual-port RAM in Qix.sv
    output [9:0]  shared_addr,
    output [7:0]  shared_din,
    input  [7:0]  shared_dout,
    output        shared_we,
    output        shared_cs_o,

    // FIRQ cross-signals
    output        video_firq,     // assert a on video CPU (active-high pulse)
    output        data_firq_ack,
    input         data_firq_n,    // FIRQ input from video CPU (active-low)

    // Player inputs (active-low, from wrapper)
    input  [7:0]  p1_input,       // PIA0 port A: player 1 joystick + buttons
    input  [7:0]  coin_input,     // PIA0 port B: coin / service switches
    input  [7:0]  spare_input,    // PIA1 port A: spare (unused on base Qix)
    input  [7:0]  in0_input,      // PIA1 port B: spare (unused on base Qix)
    input  [7:0]  p2_input,       // PIA2 port A: player 2 joystick

    // Slither PSG audio — 2x SN76489 hung off PIA1/PIA2 port B (game_id 5 only)
    output [7:0]  sn1_audio,      // signed
    output [7:0]  sn2_audio,      // signed

    // Sound PIA interface (to/from Qix_Sound)
    output [7:0]  snd_data_out,   // sndPIA0 port A output → sound CPU
    input  [7:0]  snd_data_in,    // sndPIA0 port A input ← sound CPU reply
    output [7:0]  snd_vol_out,    // sndPIA0 port B output → stereo volume
    output        snd_irq_to_snd, // CA2 → sound CPU interrupt
    input         snd_irq_from_snd,// CA1 ← sound CPU interrupt
    output        flip_screen,    // CB2 → cocktail flip

    input         crtc_vsync,    // CRTC VSYNC → sndPIA0 CB1 (frame timing)

    // ROM loading (MiSTer ioctl — pre-gated by address range in Qix.sv)
    input  [24:0] ioctl_addr,
    input  [7:0]  ioctl_data,
    input         ioctl_wr,

    // MCU EPROM loading (ioctl_index == 2, 2KB)
    input  [10:0] mcu_rom_addr,
    input  [7:0]  mcu_rom_data,
    input         mcu_rom_wr,

    output chip_pool_pkg::m6809r_i_t m6809_i,
    input  chip_pool_pkg::m6809r_o_t m6809_o,

    input         pause,
    input  [7:0]  game_id,
    // DIAG-REVERT-2026-06-15: START1-seen probe out (delete this line + the comma after game_id to revert)
    output        dbg_start1_led
);

// ---------------------------------------------------------------------------
// 6809E bus signals (declared early; driven by mc6809e instance below)
// ---------------------------------------------------------------------------
wire [15:0] cpu_A;
wire [7:0]  cpu_Dout;
wire        cpu_RnW;
wire [7:0]  cpu_Din;    // read mux output, assigned at bottom
wire        n_irq;      // active-low IRQ to 6809E, driven by sndPIA0

// ---------------------------------------------------------------------------
// Write strobe: one-cycle pulse on falling edge of E while RnW is low
// ---------------------------------------------------------------------------
wire cpu_E_fall = ce_E_fall;
wire cpu_wr     = cpu_E_fall & ~cpu_RnW;

// ---------------------------------------------------------------------------
// Address decoder
//
// Zoo Keeper relocates all data-CPU I/O from $8000-$9FFF to $0000-$1FFF and
// extends ROM from $A000-$FFFF (24KB) to $8000-$FFFF (32KB). Per qix.cpp:
//   "Zookeeper is almost the same, just the top bit of the address is inverted
//    most of the time to make space for more ROM."
// XOR cpu_A[15] with is_zookeep so the existing I/O decoders match unchanged.
// ---------------------------------------------------------------------------
wire is_zookeep = (game_id == 8'h04);
wire [15:0] dec_A = {cpu_A[15] ^ is_zookeep, cpu_A[14:0]};

wire shared_cs      = (dec_A[15:10] == 6'b10_0000);     // Qix $8000-$83FF / Zook $0000-$03FF
wire local_cs       = (dec_A[15:10] == 6'b10_0001);     // Qix $8400-$87FF / Zook $0400-$07FF
wire acia_cs        = (dec_A[15:10] == 6'b10_0010);     // Qix $8800-$8BFF / Zook $0800-$0BFF
wire firq_range     = (dec_A[15:10] == 6'b10_0011);     // Qix $8C00-$8FFF / Zook $0C00-$0FFF
wire firq_assert_cs = firq_range & ~cpu_A[0];           // even addr: assert video FIRQ
wire firq_ack_cs    = firq_range &  cpu_A[0];           // odd  addr: ack data FIRQ
wire sndpia_cs      = (dec_A[15:10] == 6'b10_0100);     // Qix $9000-$93FF / Zook $1000-$13FF
wire pia0_cs        = (dec_A[15:10] == 6'b10_0101);     // Qix $9400-$97FF / Zook $1400-$17FF
wire pia1_cs        = (dec_A[15:10] == 6'b10_0110);     // Qix $9800-$9BFF / Zook $1800-$1BFF
wire pia2_cs        = (dec_A[15:10] == 6'b10_0111);     // Qix $9C00-$9FFF / Zook $1C00-$1FFF
wire rom_cs         = is_zookeep ? cpu_A[15]            // Zook $8000-$FFFF (32KB)
                                 : (cpu_A >= 16'hA000); // Qix  $A000-$FFFF (24KB)

// PIA chip-select: single-cycle pulse, one cycle AFTER E-fall.
//
// mc6809e updates address/rw at CE_E_FALL. Delaying by one 20MHz cycle
// ensures the PIA sees cs for exactly one posedge (write) and one negedge
// (read side-effects / IRQ clearing) per bus cycle. Without this, cs is
// held for the entire E-cycle (~16 posedges at 20 MHz), causing spurious
// PIA writes, missed IRQs, and corrupted CA2/CB2 handshake strobes.
reg ce_E_fall_d;
always @(posedge clk_20m) ce_E_fall_d <= cpu_E_fall;

wire sndpia_en = ce_E_fall_d & sndpia_cs;
wire pia0_en   = ce_E_fall_d & pia0_cs;
wire pia1_en   = ce_E_fall_d & pia1_cs;
wire pia2_en   = ce_E_fall_d & pia2_cs;

// ---------------------------------------------------------------------------
// FIRQ access pulses (from schematic Figure 13, U7/U8):
//   $8C00 (even): assert FIRQ on video CPU
//   $8C01 (odd):  ack data CPU's own FIRQ
// One 20MHz cycle pulse at E-fall. SR latches live in Qix.sv.
// ---------------------------------------------------------------------------
reg firq_assert_pulse;
reg firq_ack_pulse;

always @(posedge clk_20m) begin
    if (reset) begin
        firq_assert_pulse <= 1'b0;
        firq_ack_pulse    <= 1'b0;
    end else begin
        firq_assert_pulse <= cpu_E_fall & firq_assert_cs;
        firq_ack_pulse    <= cpu_E_fall & firq_ack_cs;
    end
end

assign video_firq   = firq_assert_pulse;
assign data_firq_ack = firq_ack_pulse;

// ---------------------------------------------------------------------------
// 6809E Data CPU
// ---------------------------------------------------------------------------
assign m6809_i.reset_n = ~reset;
assign m6809_i.halt_n  = ~pause;
assign m6809_i.fall_e  = ce_E_fall;
assign m6809_i.fall_q  = ce_Q_fall;
assign m6809_i.irq_n   = n_irq;
assign m6809_i.firq_n  = data_firq_n;
assign m6809_i.nmi_n   = 1'b1;
assign m6809_i.d       = cpu_Din;
assign cpu_A    = m6809_o.a;
assign cpu_Dout = m6809_o.dout;
assign cpu_RnW  = m6809_o.rw;

wire is_slither = (game_id == 8'h05);

// ---------------------------------------------------------------------------
// MCU game detection — Space Dungeon, Kram, Electric Yo-Yo, Zoo Keeper
// ---------------------------------------------------------------------------
wire is_mcu_game = (game_id == 8'h02) ||  // Space Dungeon
                   (game_id == 8'h03) ||  // Kram
                   (game_id == 8'h04) ||  // Zoo Keeper
                   (game_id == 8'h06);    // Electric Yo-Yo

// ---------------------------------------------------------------------------
// Shared RAM — port A passthrough (dual-port RAM lives in Qix.sv)
// ---------------------------------------------------------------------------
assign shared_addr = cpu_A[9:0];
assign shared_din  = cpu_Dout;
assign shared_we   = shared_cs & cpu_wr;
assign shared_cs_o = shared_cs;

// ---------------------------------------------------------------------------
// Local RAM — 1KB BRAM ($8400-$87FF)
// ---------------------------------------------------------------------------
reg [7:0] local_ram [0:1023];
initial for (integer i = 0; i < 1024; i = i + 1) local_ram[i] = 8'h00;
reg [7:0] local_ram_dout;

always @(posedge clk_20m) begin
    if (local_cs & cpu_wr)
        local_ram[cpu_A[9:0]] <= cpu_Dout;
    local_ram_dout <= local_ram[cpu_A[9:0]];
end

// ---------------------------------------------------------------------------
// sndPIA0 ($9000-$93FF) — data CPU ↔ sound CPU
//   Port A: command data out to sound CPU
//   Port B: stereo volume (4-bit L + 4-bit R)
//   CA1: snd_irq_from_snd  CA2: snd_irq_to_snd
//   CB2: flip_screen (cocktail)
// ---------------------------------------------------------------------------
wire [7:0] sndpia_dout;
wire [7:0] sndpia_pa_o, sndpia_pa_oe;
wire [7:0] sndpia_pb_o, sndpia_pb_oe;
wire       sndpia_ca2_o, sndpia_ca2_oe;
wire       sndpia_cb2_o, sndpia_cb2_oe;
wire       sndpia_irqa, sndpia_irqb;

pia6821 sndpia0 (
    .clk      (clk_20m),
    .rst      (reset),
    .cs       (sndpia_en),
    .rw       (cpu_RnW),
    .addr     (cpu_A[1:0]),
    .data_in  (cpu_Dout),
    .data_out (sndpia_dout),
    .irqa     (sndpia_irqa),
    .irqb     (sndpia_irqb),
    // Slither: $9000 is not a sound-board PIA. Per schematic DSR 2/2, its port A
    // is J17 = the player-2 / cocktail control panel, and the data CPU's input
    // poll reads the player's buttons from HERE whenever $8400 != 0:
    //     E277: LDX #$9400 / E27A: TST $8400 / E27F: LDX #$9000 / E282: LDA ,X
    // Feeding it the 6802 reply byte leaves every button dead in that mode. No
    // separate P2 panel is mapped yet, so mirror the player-1 byte: unchanged in
    // upright, playable in cocktail.
    // DIAG-REVERT-2026-08-23: original below, uncomment to restore
    // .pa_i     (snd_data_in),
    .pa_i     (is_slither ? p1_input : snd_data_in),
    .pa_o     (sndpia_pa_o),
    .pa_oe    (sndpia_pa_oe),
    .ca1      (snd_irq_from_snd),
    .ca2_i    (1'b1),
    .ca2_o    (sndpia_ca2_o),
    .ca2_oe   (sndpia_ca2_oe),
    .pb_i     (8'hFF),
    .pb_o     (sndpia_pb_o),
    .pb_oe    (sndpia_pb_oe),
    .cb1      (crtc_vsync),  // (1'b0),
    .cb2_i    (1'b1),
    .cb2_o    (sndpia_cb2_o),
    .cb2_oe   (sndpia_cb2_oe)
);

assign snd_data_out   = sndpia_pa_o;
assign snd_vol_out    = sndpia_pb_o;
assign snd_irq_to_snd = sndpia_ca2_o;
assign flip_screen    = sndpia_cb2_o;

// IRQ to data CPU: active-low merge of sndPIA0 IRQA and IRQB
assign n_irq = ~(sndpia_irqa | sndpia_irqb);

// ---------------------------------------------------------------------------
// PIA0 ($9400-$97FF) — player 1 joystick + coin inputs
// ---------------------------------------------------------------------------
wire [7:0] pia0_dout;
wire       pia0_irqa, pia0_irqb;
wire [7:0] pia0_pb_o, pia0_pb_oe;

// MCU games: latched MCU PA output feeds PIA0 PB input, mirroring MAME's
// coin_r which returns m_68705_porta_out (captured by mcu_porta_w callback).
// Initialized to 0xFF (all inactive, active-low) so the 6809 sees no spurious
// coins/credits before the MCU has run its first IRQ handler.
wire [7:0] mcu_pa_out;
reg  [7:0] mcu_porta_cache = 8'hFF;

always @(posedge clk_20m) begin
    if (reset)
        mcu_porta_cache <= 8'hFF;
    else if (is_mcu_game && mcu_pa_wr_stb)
        mcu_porta_cache <= mcu_pa_latch;
end

wire [7:0] pia0_pb_i = is_mcu_game ? mcu_porta_cache : coin_input;

// MCU PA input latch: mirrors MAME's coin_w (PIA0 writepb_handler) — fires ONLY on
// true PB *port* writes (CRB bit2=1), NOT on DDRB writes (CRB bit2=0). $FEE4 stores the
// command to PB then writes DDRB=$FF to the SAME addr $1402; the ungated snoop latched
// that $FF → MCU dispatched on $FF → wrong reply → data reset cascade (ZK black loop).
// CRB-GATE-FIX-2026-06-13: track PIA0 CRB bit2 (offset-3 writes) and gate the latch on it.
reg pia0_crb_psel = 1'b0;   // mirror of PIA0 CRB bit2 (1 = PB port selected)
always @(posedge clk_20m) begin
    if (reset)
        pia0_crb_psel <= 1'b0;
    else if (is_mcu_game && pia0_en && ~cpu_RnW && (cpu_A[1:0] == 2'b11))
        pia0_crb_psel <= cpu_Dout[2];
end

reg [7:0] mcu_pa_cmd = 8'hFF;
always @(posedge clk_20m) begin
    if (reset)
        mcu_pa_cmd <= 8'hFF;
    // CRB-GATE-FIX-2026-06-13: original ungated line below (restore to revert the fix):
    // else if (is_mcu_game && pia0_en && ~cpu_RnW && (cpu_A[1:0] == 2'b10))
    else if (is_mcu_game && pia0_en && ~cpu_RnW && (cpu_A[1:0] == 2'b10) && pia0_crb_psel)
        mcu_pa_cmd <= cpu_Dout;
end

pia6821 pia0 (
    .clk      (clk_20m),
    .rst      (reset),
    .cs       (pia0_en),
    .rw       (cpu_RnW),
    .addr     (cpu_A[1:0]),
    .data_in  (cpu_Dout),
    .data_out (pia0_dout),
    .irqa     (pia0_irqa),
    .irqb     (pia0_irqb),
    .pa_i     (p1_input),
    .pa_o     (),
    .pa_oe    (),
    .ca1      (1'b1),
    .ca2_i    (1'b1),
    .ca2_o    (),
    .ca2_oe   (),
    .pb_i     (pia0_pb_i),
    .pb_o     (pia0_pb_o),
    .pb_oe    (pia0_pb_oe),
    .cb1      (1'b1),
    .cb2_i    (1'b1),
    .cb2_o    (),
    .cb2_oe   ()
);

// DIAG-REVERT-2026-06-15: START1-seen probe (delete this whole block to revert).
// Latches HIGH the first time the 6809 reads PIA0 PA (offset 0) and gets bit4=0
// AND bit7=1 — i.e. ZooKeeper START1 pressed on a real PA-data read. (DDRA=$00 reads
// back as $00 → bit7=0, so a DDR read can't false-trigger.) Sticky; clears on reset.
wire dbg_start1_seen = pia0_en & cpu_RnW & (cpu_A[1:0]==2'b00)
                       & ~pia0_dout[4] & pia0_dout[7];
reg  dbg_start1_latch = 1'b0;
always @(posedge clk_20m) begin
    if (reset)                dbg_start1_latch <= 1'b0;
    else if (dbg_start1_seen) dbg_start1_latch <= 1'b1;
end
assign dbg_start1_led = dbg_start1_latch;

// ---------------------------------------------------------------------------
// PIA1 ($9800-$9BFF) — spare inputs (unused on base Qix)
// ---------------------------------------------------------------------------
wire [7:0] pia1_dout;
wire       pia1_irqa, pia1_irqb;
wire [7:0] pia1_pb_o;     // Slither: SN76489 #1 data bus

// ---------------------------------------------------------------------------
// Slither SN76489 /READY handshake on PIA1/PIA2 CB1.
//
// Slither has no 6802 sound board (qix.cpp: "Slither uses 2 SN76489's for sound
// instead of the 6802+DAC; these are accessed via the PIAs"). The data CPU writes
// each sound byte to PIA port B — CB2 strobes /WE — then polls CRB bit 7 (IRQB1)
// for the chip's /READY rising edge:
//     F9DA: TST ,X / STA ,X / TST $1,X / BPL   ($9802 = PIA1 PB, $9C02 = PIA2 PB)
// With CB1 tied high that poll never exits, so the data CPU never reaches
// $E141 CLR $8016, and the video CPU spins at $E87D waiting for it => blue screen.
// CRB is programmed $26 (b1=1 => low-to-high active, b0=0 => no CPU IRQ), so a
// rising edge on CB1 sets the flag without asserting IRQ.
// ---------------------------------------------------------------------------
reg pia1_crb_psel = 1'b0;
reg pia2_crb_psel = 1'b0;
always @(posedge clk_20m) begin
    if (reset) begin
        pia1_crb_psel <= 1'b0;
        pia2_crb_psel <= 1'b0;
    end else begin
        if (pia1_en & ~cpu_RnW & (cpu_A[1:0] == 2'b11)) pia1_crb_psel <= cpu_Dout[2];
        if (pia2_en & ~cpu_RnW & (cpu_A[1:0] == 2'b11)) pia2_crb_psel <= cpu_Dout[2];
    end
end

wire pia1_pb_wr = pia1_en & ~cpu_RnW & (cpu_A[1:0] == 2'b10) & pia1_crb_psel;
wire pia2_pb_wr = pia2_en & ~cpu_RnW & (cpu_A[1:0] == 2'b10) & pia2_crb_psel;

// DIAG-REVERT-2026-08-23: /READY stand-in that got Slither booting (HW-confirmed
// 2026-08-23, boots to the operator menu). Superseded below by the real SN76489,
// whose ready_o drives CB1. Uncomment this and comment the PSG block to get back
// to the known-good booting state without the PSGs.
// localparam [8:0] SN_BUSY = 9'd320;   // ~16us at 20 MHz
// reg [8:0] sn1_busy = 9'd0;
// reg [8:0] sn2_busy = 9'd0;
// always @(posedge clk_20m) begin
//     if (reset) begin
//         sn1_busy <= 9'd0;
//         sn2_busy <= 9'd0;
//     end else begin
//         if (pia1_pb_wr)             sn1_busy <= SN_BUSY;
//         else if (sn1_busy != 9'd0)  sn1_busy <= sn1_busy - 9'd1;
//         if (pia2_pb_wr)             sn2_busy <= SN_BUSY;
//         else if (sn2_busy != 9'd0)  sn2_busy <= sn2_busy - 9'd1;
//     end
// end
// wire pia1_cb1 = is_slither ? (sn1_busy == 9'd0) : 1'b1;
// wire pia2_cb1 = is_slither ? (sn2_busy == 9'd0) : 1'b1;

// ---------------------------------------------------------------------------
// 2x SN76489 (Slither). Data bus = PIA port B output latch; /WE derived from the
// port-B data write rather than cb2_out, because pia6821.vhd clocks cb2_out on
// the FALLING edge of clk_20m (the mixed-edge pattern qix-fpga-fixes.md flags as
// a synthesis hazard). Same instant, same byte, one clock domain.
//
// ready_o (low while the chip is busy) drives CB1: its rising edge is the
// low-to-high transition the $F9DE poll waits on.
//
// ⚠️ SN_CLK_HZ is a BEST GUESS. SLITHER_CLOCK_OSC is referenced at qix.cpp:737
// but not defined in our (truncated) copy of the driver. The vault records the
// Slither data CPU at OSC/4/4 ~= 1.34 MHz; this assumes the PSGs share that rate.
// If the pitch is wrong, this constant is the only thing to change.
// ---------------------------------------------------------------------------
localparam [24:0] SN_CLK_HZ = 25'd1_340_000;

reg [24:0] sn_acc = 25'd0;
wire       sn_cen = (sn_acc >= 25'd20_000_000);
always @(posedge clk_20m) begin
    if (sn_cen) sn_acc <= sn_acc - 25'd20_000_000 + SN_CLK_HZ;
    else        sn_acc <= sn_acc + SN_CLK_HZ;
end

// SN-WE-HANDSHAKE-2026-09-23: the fixed 88-clk /WE window below was SHORTER than the PSG's
// write-sampling tick (sn76489_latch_ctrl samples on the internal /16 enable = one per ~239 clk),
// so ~63% of writes were dropped while READY still handshook (ready_o is forced '1' whenever
// /CE is high) -> game runs, PSGs silent. Now /WE is held from the port-B write until the PSG
// raises READY, like the real PIA CB2 write-strobe / CB1 handshake. Original below.
// // /WE strobe: 8 clk of setup after the PIA latches port B, then ~88 clk (4.4us)
// // low — several sn_cen edges, so the synchronous PSG cannot miss it.
// reg [6:0] sn1_wr_cnt = 7'd0;
// reg [6:0] sn2_wr_cnt = 7'd0;
// always @(posedge clk_20m) begin
//     if (reset) begin
//         sn1_wr_cnt <= 7'd0;
//         sn2_wr_cnt <= 7'd0;
//     end else begin
//         if (pia1_pb_wr)              sn1_wr_cnt <= 7'd96;
//         else if (sn1_wr_cnt != 7'd0) sn1_wr_cnt <= sn1_wr_cnt - 7'd1;
//         if (pia2_pb_wr)              sn2_wr_cnt <= 7'd96;
//         else if (sn2_wr_cnt != 7'd0) sn2_wr_cnt <= sn2_wr_cnt - 7'd1;
//     end
// end
// wire sn1_we_n = ~((sn1_wr_cnt != 7'd0) & (sn1_wr_cnt <= 7'd88));
// wire sn2_we_n = ~((sn2_wr_cnt != 7'd0) & (sn2_wr_cnt <= 7'd88));

wire sn1_ready, sn2_ready;

// Per PSG: 8 clk data setup, then /WE low until READY rises (write taken), with a
// ~2k-clk timeout so a PSG that never answers cannot wedge the CPU's CB1 poll.
reg  [3:0]  sn1_setup = 4'd0,  sn2_setup = 4'd0;
reg         sn1_wr_act = 1'b0, sn2_wr_act = 1'b0;
reg  [10:0] sn1_tmo = 11'd0,   sn2_tmo = 11'd0;
always @(posedge clk_20m) begin
    if (reset) begin
        sn1_setup <= 4'd0; sn1_wr_act <= 1'b0; sn1_tmo <= 11'd0;
        sn2_setup <= 4'd0; sn2_wr_act <= 1'b0; sn2_tmo <= 11'd0;
    end else begin
        if (pia1_pb_wr) begin
            sn1_setup <= 4'd8; sn1_wr_act <= 1'b0;
        end else if (sn1_setup != 4'd0) begin
            sn1_setup <= sn1_setup - 4'd1;
            if (sn1_setup == 4'd1) begin sn1_wr_act <= 1'b1; sn1_tmo <= 11'd0; end
        end else if (sn1_wr_act) begin
            sn1_tmo <= sn1_tmo + 11'd1;
            if (sn1_ready || &sn1_tmo) sn1_wr_act <= 1'b0;   // ready_o = ready_q while /CE low
        end
        if (pia2_pb_wr) begin
            sn2_setup <= 4'd8; sn2_wr_act <= 1'b0;
        end else if (sn2_setup != 4'd0) begin
            sn2_setup <= sn2_setup - 4'd1;
            if (sn2_setup == 4'd1) begin sn2_wr_act <= 1'b1; sn2_tmo <= 11'd0; end
        end else if (sn2_wr_act) begin
            sn2_tmo <= sn2_tmo + 11'd1;
            if (sn2_ready || &sn2_tmo) sn2_wr_act <= 1'b0;
        end
    end
end
wire sn1_we_n = ~sn1_wr_act;
wire sn2_we_n = ~sn2_wr_act;

// /CE must be part of the bus access, NOT tied low. sn76489_latch_ctrl.vhd:134
// drives `ready_o <= ready_q when ce_n_i = '0' else '1'`, and ready_q is cleared
// only by the `elsif ce_n_i = '1'` branch — so a permanently-low /CE latches READY
// high after the very first write and never produces another rising edge. Driving
// /CE with the strobe gives one clean high->low->high per write, which is the
// edge the $F9DE poll consumes.
sn76489_wrap sn1 (
    .clock_i    (clk_20m),
    .clock_en_i (sn_cen),
    .res_n_i    (~reset),
    .ce_n_i     (sn1_we_n),
    .we_n_i     (sn1_we_n),
    .ready_o    (sn1_ready),
    .d_i        (pia1_pb_o),
    .aout_o     (sn1_audio)
);

sn76489_wrap sn2 (
    .clock_i    (clk_20m),
    .clock_en_i (sn_cen),
    .res_n_i    (~reset),
    .ce_n_i     (sn2_we_n),
    .we_n_i     (sn2_we_n),
    .ready_o    (sn2_ready),
    .d_i        (pia2_pb_o),
    .aout_o     (sn2_audio)
);

// Non-Slither games keep the original constant, so their CB1 is unchanged.
wire pia1_cb1 = is_slither ? sn1_ready : 1'b1;
wire pia2_cb1 = is_slither ? sn2_ready : 1'b1;

pia6821 pia1 (
    .clk      (clk_20m),
    .rst      (reset),
    .cs       (pia1_en),
    .rw       (cpu_RnW),
    .addr     (cpu_A[1:0]),
    .data_in  (cpu_Dout),
    .data_out (pia1_dout),
    .irqa     (pia1_irqa),
    .irqb     (pia1_irqb),
    .pa_i     (spare_input),
    .pa_o     (),
    .pa_oe    (),
    .ca1      (1'b1),
    .ca2_i    (1'b1),
    .ca2_o    (),
    .ca2_oe   (),
    .pb_i     (in0_input),
    .pb_o     (pia1_pb_o),
    .pb_oe    (),
    // DIAG-REVERT-2026-08-23: original below, uncomment to restore
    // .cb1      (1'b1),
    .cb1      (pia1_cb1),   // DIAG: Slither SN76489 /READY handshake
    .cb2_i    (1'b1),
    .cb2_o    (),
    .cb2_oe   ()
);

// ---------------------------------------------------------------------------
// PIA2 ($9C00-$9FFF) — player 2 joystick + coin counters/lockout
// ---------------------------------------------------------------------------
wire [7:0] pia2_dout;
wire       pia2_irqa, pia2_irqb;
wire [7:0] pia2_pb_o, pia2_pb_oe;

pia6821 pia2 (
    .clk      (clk_20m),
    .rst      (reset),
    .cs       (pia2_en),
    .rw       (cpu_RnW),
    .addr     (cpu_A[1:0]),
    .data_in  (cpu_Dout),
    .data_out (pia2_dout),
    .irqa     (pia2_irqa),
    .irqb     (pia2_irqb),
    .pa_i     (p2_input),
    .pa_o     (),
    .pa_oe    (),
    .ca1      (1'b1),
    .ca2_i    (1'b1),
    .ca2_o    (),
    .ca2_oe   (),
    .pb_i     (8'h00),
    .pb_o     (pia2_pb_o),  // bit 2 → MCU IRQ, bit 3 → MCU PC[3] (coinctrl)
    .pb_oe    (pia2_pb_oe),
    // DIAG-REVERT-2026-08-23: original below, uncomment to restore
    // .cb1      (1'b1),
    .cb1      (pia2_cb1),   // DIAG: Slither SN76489 /READY handshake
    .cb2_i    (1'b1),
    .cb2_o    (),
    .cb2_oe   ()
);

// ---------------------------------------------------------------------------
// Data CPU ROM — sized for the largest variant on the platform.
//
//   Qix / Kram / SDungeon / etc:  24KB at $A000-$FFFF (loaded at ioctl 0..$5FFF)
//   Zoo Keeper:                   32KB at $8000-$FFFF (loaded at ioctl 0..$7FFF)
//
// Array sized to 32KB unconditionally. Non-Zook games leave the low 8KB
// unused (rom_cs never asserts there).
// ---------------------------------------------------------------------------
reg [7:0] data_rom [0:32767];                        // 32KB
reg [7:0] rom_dout;

// CPU read addr:
//   Qix:  $A000→0,    $FFFF→$5FFF  (cpu_A - $2000, low 15 bits)
//   Zook: $8000→0,    $FFFF→$7FFF  (cpu_A[14:0])
wire [14:0] rom_cpu_addr   = is_zookeep ? cpu_A[14:0]
                                        : (cpu_A[14:0] - 15'h2000);
wire [14:0] rom_ioctl_addr = ioctl_addr[14:0];

always @(posedge clk_20m) begin
    if (ioctl_wr)
        data_rom[rom_ioctl_addr] <= ioctl_data;
    rom_dout <= data_rom[rom_cpu_addr];
end

// ---------------------------------------------------------------------------
// MC68705P3 coin-input microcontroller (MCU games only)
//
// Interface per MAME qix_m.cpp:
//   - Data CPU writes PIA0 PB → MCU PA input  (coin_w)
//   - MCU PA output           → PIA0 PB input (coin_r — already wired via
//                                               pia0_pb_i mux above)
//   - MCU PB = (coin & 0x0F) | ((coin & 0x80) >> 3)
//   - MCU PC = (coinctrl & 0x08) | ((coin & 0x70) >> 4)
//   - PIA2 PB[2] → /IRQ (active-low when bit is high)
// ---------------------------------------------------------------------------

// MCU and 6809 run concurrently — no halting needed or wanted.
// The 6809 takes ~40-100µs between asserting mcu_irq_n and returning
// to read the MCU response, which is sufficient for the MCU handler
// to complete at 1 MHz. Halting caused a deadlock: the IRQ line could
// never de-assert while the 6809 was halted.

// 4 MHz enable from 20 MHz: pulse one-in-five clk_20m ticks.
reg [2:0] mcu_ce_div = 3'd0;
reg       mcu_ce_4m  = 1'b0;
always @(posedge clk_20m) begin
    mcu_ce_4m <= 1'b0;
    if (reset) begin
        mcu_ce_div <= 3'd0;
    end else if (mcu_ce_div == 3'd4) begin
        mcu_ce_div <= 3'd0;
        mcu_ce_4m  <= 1'b1;
    end else begin
        mcu_ce_div <= mcu_ce_div + 3'd1;
    end
end

// MCU port inputs derived from coin_input and PIA2 coinctrl.
// PIA2 PB pin values as seen by the MCU. For bits configured as input
// (DDR=0, oe=0), the pin floats high through pull-ups. For output bits,
// the pin reflects the latch. pia2_pb_o reads 0 for input bits in this
// PIA model, so we must explicitly OR-in the pull-up state.
wire [7:0] pia2_pb_pin = pia2_pb_o | ~pia2_pb_oe;

wire [7:0] mcu_pb_in = {3'b000, coin_input[7], coin_input[3:0]};
wire [3:0] mcu_pc_in = {pia2_pb_pin[3], coin_input[6:4]};
// EYY-MCU-IRQ-2026-08-04: the MCU IRQ must come from the PIA2 PB **output
// latch**, not from the pull-up-modified PIN.
//
// MAME (`qix_m.cpp:185-197` coinctrl_w) asserts/clears M68705_IRQ_LINE from
// `BIT(data,2)` — the value the data CPU WROTE. Our `pia2_pb_pin` ORs in
// `~pia2_pb_oe`, so any bit configured as an INPUT reads back as 1. If EYY
// leaves PIA2 DDRB bit 2 as an input, `mcu_irq_n` is then STUCK ASSERTED — and
// the 68705's IRQ is EDGE-triggered, so the handler never re-runs. The MCU
// therefore never rewrites port A, `mcu_porta_cache` never updates, and the
// SERVICE (Test Advance) switch — which for MCU games reaches the data CPU only
// via MCU port A — is dead. Matches the reported symptom exactly: EYY sits in
// the NVRAM setup screen and no button advances it.
//
// Using the latch makes an undriven bit read 0 => IRQ deasserted => a real
// 0->1 write produces a real edge, as on hardware and in MAME.
// Original: wire mcu_irq_n = ~pia2_pb_pin[2];
wire       mcu_irq_n = ~pia2_pb_o[2];

wire [7:0] mcu_pa_latch;
wire       mcu_pa_wr_stb;

mc68705p3 mcu (
    .clk         (clk_20m),
    .ce_4m       (mcu_ce_4m),
    .reset       (reset),
    .irq_n       (mcu_irq_n),
    .pa_in       (mcu_pa_cmd),
    .pa_out      (mcu_pa_out),
    .pa_latch_out(mcu_pa_latch),
    .pa_wr_stb   (mcu_pa_wr_stb),
    .pb_in       (mcu_pb_in),
    .pb_out      (),
    .pb_ddr      (),
    .pc_in       (mcu_pc_in),
    .pc_out      (),
    .pc_ddr      (),
    .rom_wr      (mcu_rom_wr),
    .rom_addr    (mcu_rom_addr),
    .rom_data    (mcu_rom_data)
);

// ---------------------------------------------------------------------------
// CPU data bus read mux — default $FF for open-bus / unimplemented reads
// ---------------------------------------------------------------------------

assign cpu_Din =
    shared_cs  ? shared_dout     :
    local_cs   ? local_ram_dout  :
    acia_cs    ? 8'h02           :
    sndpia_cs  ? sndpia_dout     :
    pia0_cs    ? pia0_dout       :
    pia1_cs    ? pia1_dout       :
    pia2_cs    ? pia2_dout       :
    rom_cs     ? rom_dout        :
    8'hFF;

endmodule
