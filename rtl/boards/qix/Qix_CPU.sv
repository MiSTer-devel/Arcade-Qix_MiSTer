//============================================================================
//
// Qix Data CPU Board
// Copyright (C) 2026 Rodimus
//
// Hardware: mc6809e + 3× PIA6821 + 1× sndPIA0 + 1KB local RAM + 24KB ROM
// (CPU and the four PIAs live in the dispatcher's chip pool; this board drives their buses)
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

    // Slither PSGs — 2x SN76489 hung off PIA1/PIA2 port B (game_id 5 only): pool sn76489[0..1]
    output chip_pool_pkg::sn76489_i_t [1:0] sn_i,
    input  chip_pool_pkg::sn76489_o_t [1:0] sn_o,

    // Sound PIA interface (to/from Qix_Sound)
    output [7:0]  snd_data_out,   // sndPIA0 port A output → sound CPU
    input  [7:0]  snd_data_in,    // sndPIA0 port A input ← sound CPU reply
    output [7:0]  snd_vol_out,    // sndPIA0 port B output → stereo volume
    output        snd_irq_to_snd, // CA2 → sound CPU interrupt
    input         snd_irq_from_snd,// CA1 ← sound CPU interrupt
    output        flip_screen,    // CB2 → cocktail flip

    input         crtc_vsync,    // CRTC VSYNC → sndPIA0 CB1 (frame timing)

    // Data CPU ROM: ROM block 0 in the dispatcher (loaded there from the ROM download)
    output [15:0] rom_a,
    input  [7:0]  rom_d,

    // MCU EPROM loading (ioctl_index == 2, 2KB)
    input  [10:0] mcu_rom_addr,
    input  [7:0]  mcu_rom_data,
    input         mcu_rom_wr,

    output chip_pool_pkg::mc68705p3_i_t mcu_i,     // chip pool: MCU
    input  chip_pool_pkg::mc68705p3_o_t mcu_o,

    output chip_pool_pkg::m6809r_i_t m6809_i,
    input  chip_pool_pkg::m6809r_o_t m6809_o,
    output chip_pool_pkg::pia6821_i_t [3:0] pia_i,   // [0] sndPIA0  [1] PIA0  [2] PIA1  [3] PIA2
    input  chip_pool_pkg::pia6821_o_t [3:0] pia_o,
    output ram_pool_pkg::pram_i_t    lr_pr_i,        // RAM pool block 3: local RAM
    input  ram_pool_pkg::pram_o_t    lr_pr_o,

    input         pause,
    input  [7:0]  game_id
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

// PIA bus strobe: single-cycle pulse, one cycle AFTER E-fall (the PIAs' ce; cs is
// the address decode). The PIA acts on ce & cs: exactly one write / read
// side-effect per bus cycle. Without the strobe, cs is held for the entire
// E-cycle (~16 clocks at 20 MHz): spurious PIA writes, missed IRQs, and corrupted
// CA2/CB2 handshake strobes. The pia*_en pulses below feed this board's own
// register snoops.
reg ce_E_fall_d;
always @(posedge clk_20m) ce_E_fall_d <= cpu_E_fall;

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
// Local RAM — 1KB ($8400-$87FF). Storage: RAM pool block 3 in the dispatcher (port B unused).
// ---------------------------------------------------------------------------
wire [7:0] local_ram_dout;

assign lr_pr_i.a_addr = {6'd0, cpu_A[9:0]};
assign lr_pr_i.a_d    = cpu_Dout;
assign lr_pr_i.a_we   = local_cs & cpu_wr;
assign lr_pr_i.b_addr = 16'd0;
assign lr_pr_i.b_d    = 8'd0;
assign lr_pr_i.b_we   = 1'b0;
assign local_ram_dout = lr_pr_o.a_q;

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

// Pool pia6821[0]. pa_pins = 0: port A output bits read back the latch.
assign pia_i[0].rst     = reset;
assign pia_i[0].ce      = ce_E_fall_d;
assign pia_i[0].cs      = sndpia_cs;
assign pia_i[0].rw      = cpu_RnW;
assign pia_i[0].rs      = cpu_A[1:0];
assign pia_i[0].din     = cpu_Dout;
// Slither: $9000 is not a sound-board PIA. Per schematic DSR 2/2, its port A
// is J17 = the player-2 / cocktail control panel, and the data CPU's input
// poll reads the player's buttons from HERE whenever $8400 != 0:
//     E277: LDX #$9400 / E27A: TST $8400 / E27F: LDX #$9000 / E282: LDA ,X
// Feeding it the 6802 reply byte leaves every button dead in that mode. No
// separate P2 panel is mapped yet, so mirror the player-1 byte: unchanged in
// upright, playable in cocktail.
assign pia_i[0].pa_i    = is_slither ? p1_input : snd_data_in;
assign pia_i[0].pa_pins = 1'b0;
assign pia_i[0].ca1     = snd_irq_from_snd;
assign pia_i[0].ca2_i   = 1'b1;
assign pia_i[0].pb_i    = 8'hFF;
assign pia_i[0].cb1     = crtc_vsync;
assign pia_i[0].cb2_i   = 1'b1;

// The board sees pin levels: port bits read 0 while configured as inputs, CA2 / CB2 0 while inputs.
assign sndpia_dout   = pia_o[0].dout;
assign sndpia_irqa   = pia_o[0].irqa;
assign sndpia_irqb   = pia_o[0].irqb;
assign sndpia_pa_o   = pia_o[0].pa_o & pia_o[0].pa_oe;
assign sndpia_pa_oe  = pia_o[0].pa_oe;
assign sndpia_pb_o   = pia_o[0].pb_o & pia_o[0].pb_oe;
assign sndpia_pb_oe  = pia_o[0].pb_oe;
assign sndpia_ca2_o  = pia_o[0].ca2_o & pia_o[0].ca2_oe;
assign sndpia_ca2_oe = pia_o[0].ca2_oe;
assign sndpia_cb2_o  = pia_o[0].cb2_o & pia_o[0].cb2_oe;
assign sndpia_cb2_oe = pia_o[0].cb2_oe;

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

// MCU games: the MCU's latched port A output feeds the PIA0 PB input (coin_r returns the MCU port A latch).
// Starts at 0xFF (all inactive, active-low) so the 6809 sees no spurious coins / credits before the MCU's
// first IRQ handler.
wire [7:0] mcu_pa_out;
wire [7:0] mcu_pa_latch;         // MC68705P3 port A latch / write strobe (mc68705p3_0, below)
wire       mcu_pa_wr_stb;
reg  [7:0] mcu_porta_cache = 8'hFF;

always @(posedge clk_20m) begin
    if (reset)
        mcu_porta_cache <= 8'hFF;
    else if (is_mcu_game && mcu_pa_wr_stb)
        mcu_porta_cache <= mcu_pa_latch;
end

wire [7:0] pia0_pb_i = is_mcu_game ? mcu_porta_cache : coin_input;

// MCU PA input latch (coin_w): fires only on PB port writes (CRB bit 2 = 1), not on DDRB writes (CRB bit 2 = 0).
// $FEE4 stores the command to PB, then writes DDRB = $FF to the same address $1402; latching that $FF makes the
// MCU dispatch on $FF (wrong reply, data CPU reset loop, Zoo Keeper black screen). So track PIA0 CRB bit 2
// (offset-3 writes) and gate the latch on it.
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
    else if (is_mcu_game && pia0_en && ~cpu_RnW && (cpu_A[1:0] == 2'b10) && pia0_crb_psel)
        mcu_pa_cmd <= cpu_Dout;
end

// Pool pia6821[1]. pa_pins = 1: port A reads its pins, output bits too; the pins are the switches wired-AND the outputs.
assign pia_i[1].rst     = reset;
assign pia_i[1].ce      = ce_E_fall_d;
assign pia_i[1].cs      = pia0_cs;
assign pia_i[1].rw      = cpu_RnW;
assign pia_i[1].rs      = cpu_A[1:0];
assign pia_i[1].din     = cpu_Dout;
assign pia_i[1].pa_i    = p1_input    & (pia_o[1].pa_o | ~pia_o[1].pa_oe);   // switches pull low only: an output driven low stays low
assign pia_i[1].pa_pins = 1'b1;
assign pia_i[1].ca1     = 1'b1;
assign pia_i[1].ca2_i   = 1'b1;
assign pia_i[1].pb_i    = pia0_pb_i;
assign pia_i[1].cb1     = 1'b1;
assign pia_i[1].cb2_i   = 1'b1;

assign pia0_dout  = pia_o[1].dout;
assign pia0_irqa  = pia_o[1].irqa;
assign pia0_irqb  = pia_o[1].irqb;
assign pia0_pb_o  = pia_o[1].pb_o & pia_o[1].pb_oe;   // pin level: 0 while an input
assign pia0_pb_oe = pia_o[1].pb_oe;

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

// ---------------------------------------------------------------------------
// 2x SN76489 (Slither), pool sn76489[0..1] (chips-repo sn76489sv, VARIANT 0). Data bus = PIA port B output latch;
// /WE (= /CE) derived from the port-B data write rather than cb2_out. READY (low for 32 PSG clocks after a write)
// drives CB1: its rising edge is the low-to-high transition the $F9DE poll waits on.
//
// ⚠️ SN_CLK_HZ is a BEST GUESS. SLITHER_CLOCK_OSC is referenced at qix.cpp:737
// but not defined in our (truncated) copy of the driver. The vault records the
// Slither data CPU at OSC/4/4 ~= 1.34 MHz; this assumes the PSGs share that rate.
// If the pitch is wrong, this constant is the only thing to change.
// ---------------------------------------------------------------------------
localparam [24:0] SN_CLK_HZ = 25'd1_340_000;

// Pause freezes the PSGs (accumulator, clock enable and write handshake), as the sound board freezes snd_cen:
// a frozen tone generator holds its level, so a note in progress stops instead of ringing on.
reg [24:0] sn_acc = 25'd0;
wire       sn_cen = (sn_acc >= 25'd20_000_000) & ~pause;
always @(posedge clk_20m) begin
    if (pause)       sn_acc <= sn_acc;
    else if (sn_cen) sn_acc <= sn_acc - 25'd20_000_000 + SN_CLK_HZ;
    else             sn_acc <= sn_acc + SN_CLK_HZ;
end

wire sn1_ready, sn2_ready;
wire [7:0] pia2_pb_o, pia2_pb_oe;   // PIA2 port B (pool pia6821[3], below): PSG 2 data, MCU control

// Per PSG: 8 clk data setup, then /WE low until READY is high (the PSG drops it the clock after it takes the write),
// with a ~2k-clk timeout so a PSG that never answers cannot wedge the CPU's CB1 poll.
reg  [3:0]  sn1_setup = 4'd0,  sn2_setup = 4'd0;
reg         sn1_wr_act = 1'b0, sn2_wr_act = 1'b0;
reg  [10:0] sn1_tmo = 11'd0,   sn2_tmo = 11'd0;
always @(posedge clk_20m) begin
    if (reset) begin
        sn1_setup <= 4'd0; sn1_wr_act <= 1'b0; sn1_tmo <= 11'd0;
        sn2_setup <= 4'd0; sn2_wr_act <= 1'b0; sn2_tmo <= 11'd0;
    end else if (!pause) begin
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

assign sn_i[0].rst  = reset;
assign sn_i[0].ce   = sn_cen;
assign sn_i[0].wr_n = sn1_we_n;
assign sn_i[0].cs_n = sn1_we_n;
assign sn_i[0].din  = pia1_pb_o;
assign sn_i[1].rst  = reset;
assign sn_i[1].ce   = sn_cen;
assign sn_i[1].wr_n = sn2_we_n;
assign sn_i[1].cs_n = sn2_we_n;
assign sn_i[1].din  = pia2_pb_o;
assign sn1_ready    = sn_o[0].ready;
assign sn2_ready    = sn_o[1].ready;

// Non-Slither games keep the original constant, so their CB1 is unchanged.
wire pia1_cb1 = is_slither ? sn1_ready : 1'b1;
wire pia2_cb1 = is_slither ? sn2_ready : 1'b1;

// Pool pia6821[2]. pa_pins = 1: port A reads its pins, output bits too; the pins are the switches wired-AND the outputs.
assign pia_i[2].rst     = reset;
assign pia_i[2].ce      = ce_E_fall_d;
assign pia_i[2].cs      = pia1_cs;
assign pia_i[2].rw      = cpu_RnW;
assign pia_i[2].rs      = cpu_A[1:0];
assign pia_i[2].din     = cpu_Dout;
assign pia_i[2].pa_i    = spare_input & (pia_o[2].pa_o | ~pia_o[2].pa_oe);   // switches pull low only: an output driven low stays low
assign pia_i[2].pa_pins = 1'b1;
assign pia_i[2].ca1     = 1'b1;
assign pia_i[2].ca2_i   = 1'b1;
assign pia_i[2].pb_i    = in0_input;
assign pia_i[2].cb1     = pia1_cb1;   // Slither: PSG 1 READY
assign pia_i[2].cb2_i   = 1'b1;

assign pia1_dout  = pia_o[2].dout;
assign pia1_irqa  = pia_o[2].irqa;
assign pia1_irqb  = pia_o[2].irqb;
assign pia1_pb_o  = pia_o[2].pb_o & pia_o[2].pb_oe;   // pin level: 0 while an input

// ---------------------------------------------------------------------------
// PIA2 ($9C00-$9FFF) — player 2 joystick + coin counters/lockout
// ---------------------------------------------------------------------------
wire [7:0] pia2_dout;
wire       pia2_irqa, pia2_irqb;

// Pool pia6821[3]. pa_pins = 1: port A reads its pins, output bits too; the pins are the switches wired-AND the outputs.
assign pia_i[3].rst     = reset;
assign pia_i[3].ce      = ce_E_fall_d;
assign pia_i[3].cs      = pia2_cs;
assign pia_i[3].rw      = cpu_RnW;
assign pia_i[3].rs      = cpu_A[1:0];
assign pia_i[3].din     = cpu_Dout;
assign pia_i[3].pa_i    = p2_input    & (pia_o[3].pa_o | ~pia_o[3].pa_oe);   // switches pull low only: an output driven low stays low
assign pia_i[3].pa_pins = 1'b1;
assign pia_i[3].ca1     = 1'b1;
assign pia_i[3].ca2_i   = 1'b1;
assign pia_i[3].pb_i    = 8'h00;
assign pia_i[3].cb1     = pia2_cb1;   // Slither: PSG 2 READY
assign pia_i[3].cb2_i   = 1'b1;

assign pia2_dout  = pia_o[3].dout;
assign pia2_irqa  = pia_o[3].irqa;
assign pia2_irqb  = pia_o[3].irqb;
assign pia2_pb_o  = pia_o[3].pb_o & pia_o[3].pb_oe;   // pin level: 0 while an input; bit 2 → MCU IRQ, bit 3 → MCU PC[3] (coinctrl)
assign pia2_pb_oe = pia_o[3].pb_oe;

// ---------------------------------------------------------------------------
// Data CPU ROM — sized for the largest variant on the platform.
//
//   Qix / Kram / SDungeon / etc:  24KB at $A000-$FFFF (loaded at ioctl 0..$5FFF)
//   Zoo Keeper:                   32KB at $8000-$FFFF (loaded at ioctl 0..$7FFF)
//
// Store sized to 32KB (dispatcher ROM block 0). Non-Zook games leave the high 8KB
// unused (rom_cs never asserts there).
// ---------------------------------------------------------------------------
wire [7:0] rom_dout;

// CPU read addr:
//   Qix:  $A000→0,    $FFFF→$5FFF  (cpu_A - $2000, low 15 bits)
//   Zook: $8000→0,    $FFFF→$7FFF  (cpu_A[14:0])
wire [14:0] rom_cpu_addr   = is_zookeep ? cpu_A[14:0]
                                        : (cpu_A[14:0] - 15'h2000);
assign rom_a    = {1'b0, rom_cpu_addr};
assign rom_dout = rom_d;

// ---------------------------------------------------------------------------
// MC68705P3 coin-input microcontroller (MCU games only)
//
// Interface (qix_m.cpp):
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
// the pin reflects the latch. pia2_pb_o reads 0 for input bits (masked
// at the PIA2 bus above), so we must explicitly OR-in the pull-up state.
wire [7:0] pia2_pb_pin = pia2_pb_o | ~pia2_pb_oe;

wire [7:0] mcu_pb_in = {3'b000, coin_input[7], coin_input[3:0]};
wire [3:0] mcu_pc_in = {pia2_pb_pin[3], coin_input[6:4]};
// MCU IRQ = PIA2 PB bit 2 as written (the output latch), not the pin: coinctrl_w drives the IRQ from the written value.
// The pin view reads an input bit as 1 (pull-up), which would hold the edge-triggered IRQ asserted, so the MCU never
// rewrites port A (Electric Yo-Yo then sits in the NVRAM setup screen: Test Advance reaches the data CPU only through
// MCU port A). The latch reads an undriven bit as 0, so a 0->1 write is a real edge.
wire       mcu_irq_n = ~pia2_pb_o[2];


// MC68705P3 — chip pool slot mc68705p3_0 in the dispatcher (clk_20m)
assign mcu_i.reset    = reset;
assign mcu_i.ce_4m    = mcu_ce_4m;
assign mcu_i.irq_n    = mcu_irq_n;
assign mcu_i.pa_in    = mcu_pa_cmd;
assign mcu_i.pb_in    = mcu_pb_in;
assign mcu_i.pc_in    = mcu_pc_in;
assign mcu_i.rom_wr   = mcu_rom_wr;
assign mcu_i.rom_addr = mcu_rom_addr;
assign mcu_i.rom_data = mcu_rom_data;

assign mcu_pa_out     = mcu_o.pa_out;
assign mcu_pa_latch   = mcu_o.pa_latch_out;
assign mcu_pa_wr_stb  = mcu_o.pa_wr_stb;

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
