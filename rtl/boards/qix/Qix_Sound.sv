//============================================================================
//
// Qix Audio Board
// Copyright (C) 2026 Rodimus
//
// Hardware: MC6802 audio CPU (cpu68 core) + 2× PIA6821 (dispatcher chip pool) + 8-bit DAC +
//           discrete stereo volume attenuator
//
// Responsibilities:
//   - Clock enable: 921.6 kHz (fractional divider from 20 MHz, matches real hardware)
//   - Receives sound commands from data CPU via sndPIA1 port A
//   - 8-bit unsigned DAC via sndPIA1 port B
//   - Stereo volume scaling from data CPU sndPIA0 port B
//
//============================================================================

module Qix_Sound (
    input         clk_20m,
    input         reset,

    // Communication with data CPU (via sndPIA0 on data CPU side)
    input  [7:0]  snd_data_in,      // data from data CPU → sndPIA1 port A input
    output [7:0]  snd_data_out,     // sndPIA1 port A output → data CPU
    input         snd_irq_from_cpu, // data CPU sndPIA0 CA2 → our sndPIA1 CA1
    output        snd_irq_to_cpu,   // our sndPIA1 CA2 → data CPU sndPIA0 CA1

    // Volume control (sndPIA0 port B on data CPU, passed through)
    input  [7:0]  vol_data,         // [7:4] = left vol index, [3:0] = right vol index

    // Audio output (signed 16-bit stereo)
    output signed [15:0] audio_l,
    output signed [15:0] audio_r,

    // Sound CPU ROM: ROM block 2 in the dispatcher
    output [15:0] rom_a,
    input  [7:0]  rom_d,

    input         pause,

    // Chip pool: the two sound PIAs live in the dispatcher
    output chip_pool_pkg::pia6821_i_t [1:0] pia_i,   // [0] sndPIA1  [1] sndPIA2
    input  chip_pool_pkg::pia6821_o_t [1:0] pia_o,

    // TMS5220 speech chip: pool tms5220rsws_0 (chips-repo tms5220 + tms5220_rsws, 640 kHz)
    output chip_pool_pkg::tms5220rsws_i_t tms_i,
    input  chip_pool_pkg::tms5220rsws_o_t tms_o
);

// ---------------------------------------------------------------------------
// Audio CPU bus signals
// ---------------------------------------------------------------------------
wire [15:0] snd_A;
wire [7:0]  snd_Dout;
wire [7:0]  snd_Din;           // assigned at bottom by read mux
wire        snd_rw;            // 1 = read, 0 = write (kept for PIA compatibility)
wire        snd_wr;            // active-high write strobe from jt680x

// One-shot power-on reset for cpu68. Fires once at startup to ensure
// the CPU fetches the reset vector. Uses a 2-bit counter with explicit
// initial value — Quartus respects initial values on registers even with
// ALLOW_POWER_UP_DONT_CARE when they are part of active logic.
reg [1:0] snd_por = 2'b11;
always @(posedge clk_20m)
    if (reset)           snd_por <= 2'b11;
    else if (snd_por != 2'b00) snd_por <= snd_por - 2'd1;
wire snd_rst = (snd_por != 2'b00);

// Fractional clock enable: 3.6864 MHz cen from 20 MHz (exact average).
// jt680x wants cen = the crystal rate; its ucode has `cycle_factor: 4`, so the /4 to the
// 6802's 921.6 kHz E clock happens inside the microcode, not here.
// Sound board: 7.3728 MHz oscillator, M6802 at OSC / 2 => 3.6864 MHz crystal, E = 921.6 kHz.
//
// The sound CPU is an MC6802: jotego's jt680x, unmodified, in rtl/cpu/jt680x_6802/ with its 6801.vh / _param.vh /
// .uc generated as 6802 microcode (ucode/6802.yaml: the 6800 per-opcode cycle table; ucode/gen.sh). jotego's own
// 6801 microcode (the enhanced 6800) runs ~8% fast here.
// Clock rates tried on hardware and rejected: 921_600 (4x slow: the /4 is in the microcode), 3_200_000
// (compensated the 6801 timing), 3_133_440 (a 15%-slow test, not a fix).
// History: vault Claude/qix_zookeeper_music_tempo_2026-06-16.md
reg [24:0] snd_acc;
wire snd_cen_raw = (snd_acc >= 25'd20_000_000);
always @(posedge clk_20m) begin
    if (snd_cen_raw)
        snd_acc <= snd_acc - 25'd20_000_000 + 25'd3_686_400;
    else
        snd_acc <= snd_acc + 25'd3_686_400;
end

wire snd_cen = snd_cen_raw & ~pause;
reg snd_cen_d;
always @(posedge clk_20m) snd_cen_d <= snd_cen;

// ---------------------------------------------------------------------------
// Address decoder — qualified by VMA from cpu68
// ---------------------------------------------------------------------------
wire sndpia2_cs_addr = (snd_A[15:13] == 3'b001);   // $2000-$3FFF
wire sndpia1_cs_addr = (snd_A[15:14] == 2'b01);    // $4000-$7FFF
wire rom_cs          = (snd_A >= 16'hD000);        // $D000-$FFFF (12KB)

// ---------------------------------------------------------------------------
// PIA enables
// ---------------------------------------------------------------------------
// The PIAs take snd_cen as ce and the address decode as cs. jt680x's ucode has `cycle_factor: 4`, so one CPU bus
// cycle is 4 snd_cen pulses and each PIA is enabled ~4x per access. That is what works on hardware: a one-pulse-per-
// bus-cycle version (a bus_phase counter committing on phase 3) black-screened every game but Zoo Keeper, which lost
// its sound. Don't change it without a sim that establishes the right phase.

// ---------------------------------------------------------------------------
// 6802 internal RAM — 128 bytes ($0000-$007F)
// cpu68 is a pure CPU core and does NOT include internal RAM.
// ---------------------------------------------------------------------------
reg [7:0] internal_ram [0:127];
wire internal_ram_cs   = (snd_A[15:7] == 9'd0);    // $0000-$007F
//wire [7:0] internal_ram_dout = internal_ram[snd_A[6:0]];

reg [7:0] internal_ram_dout;
always @(posedge clk_20m)
    if (snd_cen_d && internal_ram_cs)
        internal_ram_dout <= internal_ram[snd_A[6:0]];

always @(posedge clk_20m)
    if (snd_cen && snd_wr && internal_ram_cs)
        internal_ram[snd_A[6:0]] <= snd_Dout;

// ---------------------------------------------------------------------------
// jt680x — MC6802 compatible audio CPU (jotego, posedge-clocked with cen)
//   rst      : active-high synchronous reset
//   cen      : clock enable (advances CPU one cycle when high)
//   wr       : active-high write strobe
//   addr     : 16-bit address bus (always valid)
//   irq      : active-high IRQ
//   nmi      : active-high NMI — tie low (not used by Qix)
//   ext_halt : 6301 bus sharing — tie low
//   irq_icf/ocf/tof/sci/cmf/irq2 : 6801/6301-specific — tie low for 6802
// ---------------------------------------------------------------------------
wire snd_irq;
assign snd_rw = ~snd_wr;  // keep snd_rw for PIA compatibility

jt680x audio_cpu (
    .rst        (snd_rst),
    .clk        (clk_20m),
    .cen        (snd_cen),
    .wr         (snd_wr),
    .addr       (snd_A),
    .din        (snd_Din),
    .dout       (snd_Dout),
    .ext_halt   (1'b0),
    .ba         (),
    .irq        (snd_irq),
    .nmi        (1'b0),
    .irq_icf    (1'b0),
    .irq_ocf    (1'b0),
    .irq_tof    (1'b0),
    .irq_sci    (1'b0),
    .irq_cmf    (1'b0),
    .irq2       (1'b0)
);

// ---------------------------------------------------------------------------
// sndPIA1 ($4000-$7FFF) — main sound PIA
//   Port A: bidirectional comm with data CPU
//   Port B: 8-bit DAC value (written by audio CPU)
//   CA1: interrupt from data CPU (data CPU sndPIA0 CA2)
//   CA2: interrupt to data CPU  (data CPU sndPIA0 CA1)
// ---------------------------------------------------------------------------
wire [7:0] sndpia1_dout;
wire [7:0] sndpia1_pa_o,  sndpia1_pa_oe;
wire [7:0] sndpia1_pb_o,  sndpia1_pb_oe;
wire       sndpia1_ca2_o, sndpia1_ca2_oe;
wire       sndpia1_cb2_o, sndpia1_cb2_oe;
wire       sndpia1_irqa,  sndpia1_irqb;

// Pool pia6821[0]. pa_pins = 0: port A output bits read back the latch.
assign pia_i[0].rst     = snd_rst;
assign pia_i[0].ce      = snd_cen;
assign pia_i[0].cs      = sndpia1_cs_addr;
assign pia_i[0].rw      = snd_rw;
assign pia_i[0].rs      = snd_A[1:0];
assign pia_i[0].din     = snd_Dout;
assign pia_i[0].pa_i    = snd_data_in;
assign pia_i[0].pa_pins = 1'b0;
assign pia_i[0].ca1     = snd_irq_from_cpu;
assign pia_i[0].ca2_i   = 1'b1;
assign pia_i[0].pb_i    = 8'h00;
assign pia_i[0].cb1     = 1'b0;
assign pia_i[0].cb2_i   = 1'b1;

// The board sees pin levels: port bits read 0 while configured as inputs, CA2 / CB2 0 while inputs.
assign sndpia1_dout   = pia_o[0].dout;
assign sndpia1_irqa   = pia_o[0].irqa;
assign sndpia1_irqb   = pia_o[0].irqb;
assign sndpia1_pa_o   = pia_o[0].pa_o & pia_o[0].pa_oe;     // to data CPU
assign sndpia1_pa_oe  = pia_o[0].pa_oe;
assign sndpia1_pb_o   = pia_o[0].pb_o & pia_o[0].pb_oe;     // 8-bit DAC value
assign sndpia1_pb_oe  = pia_o[0].pb_oe;
assign sndpia1_ca2_o  = pia_o[0].ca2_o & pia_o[0].ca2_oe;   // to data CPU sndPIA0 CA1
assign sndpia1_ca2_oe = pia_o[0].ca2_oe;
assign sndpia1_cb2_o  = pia_o[0].cb2_o & pia_o[0].cb2_oe;
assign sndpia1_cb2_oe = pia_o[0].cb2_oe;

assign snd_data_out   = sndpia1_pa_o;
assign snd_irq_to_cpu = sndpia1_ca2_o;

// ---------------------------------------------------------------------------
// sndPIA2 ($2000-$3FFF) — TMS5220 speech PIA. The chip is on the board per the schematic; no game is known to use it.
// Wiring:
//   port B ↔ D0-D7
//   CA1    ← /INT    (TMS5200 interrupt)
//   CA2    → /WS     (write strobe)
//   CB1    ← /READY  (ready for next byte)
//   CB2    → /RS     (read strobe)
// ---------------------------------------------------------------------------
wire [7:0] sndpia2_dout;
wire       sndpia2_irqa, sndpia2_irqb;

wire [7:0] sndpia2_pb_o,  sndpia2_pb_oe;
wire       sndpia2_ca2_o, sndpia2_ca2_oe;
wire       sndpia2_cb2_o, sndpia2_cb2_oe;

// TMS5220 outputs
wire [7:0]         tms_dout = tms_o.d_out;
wire               tms_intn = tms_o.int_n;
wire               tms_rdyn = tms_o.ready_n;

// Pool pia6821[1]. pa_pins = 0: port A output bits read back the latch.
assign pia_i[1].rst     = snd_rst;
assign pia_i[1].ce      = snd_cen;
assign pia_i[1].cs      = sndpia2_cs_addr;
assign pia_i[1].rw      = snd_rw;
assign pia_i[1].rs      = snd_A[1:0];
assign pia_i[1].din     = snd_Dout;
assign pia_i[1].pa_i    = 8'hFF;
assign pia_i[1].pa_pins = 1'b0;
assign pia_i[1].ca1     = tms_intn;
assign pia_i[1].ca2_i   = 1'b1;
assign pia_i[1].pb_i    = tms_dout;
assign pia_i[1].cb1     = tms_rdyn;
assign pia_i[1].cb2_i   = 1'b1;

// The board sees pin levels: port bits read 0 while configured as inputs, CA2 / CB2 0 while inputs.
assign sndpia2_dout   = pia_o[1].dout;
assign sndpia2_irqa   = pia_o[1].irqa;
assign sndpia2_irqb   = pia_o[1].irqb;
assign sndpia2_pb_o   = pia_o[1].pb_o & pia_o[1].pb_oe;     // D0-D7 to TMS5200
assign sndpia2_pb_oe  = pia_o[1].pb_oe;
assign sndpia2_ca2_o  = pia_o[1].ca2_o & pia_o[1].ca2_oe;   // /WS to TMS5200
assign sndpia2_ca2_oe = pia_o[1].ca2_oe;
assign sndpia2_cb2_o  = pia_o[1].cb2_o & pia_o[1].cb2_oe;   // /RS to TMS5200
assign sndpia2_cb2_oe = pia_o[1].cb2_oe;

// ---------------------------------------------------------------------------
// TMS5220 pins: /WS = CA2, /RS = CB2, data = port B (pin levels: 0 while a PIA line is an input).
// The chip runs at the board's nominal 640 kHz (RC oscillator, pot-adjustable); pause holds it.
// ---------------------------------------------------------------------------
assign tms_i.reset = snd_rst;
assign tms_i.pause = pause;
assign tms_i.ws_n  = sndpia2_ca2_o;
assign tms_i.rs_n  = sndpia2_cb2_o;
assign tms_i.d_in  = sndpia2_pb_o;

// IRQ to audio CPU: OR of all PIA interrupt outputs (active-high). The reference driver ORs only sndPIA1;
// sndPIA1-only was tried on hardware and changed nothing. Revisit with speech.
assign snd_irq = sndpia1_irqa | sndpia1_irqb | sndpia2_irqa | sndpia2_irqb;

// ---------------------------------------------------------------------------
// Audio ROM — 12KB ($D000-$FFFF): dispatcher ROM block 2 (loaded there from the ROM download).
// CPU read address: snd_A[13:0] - $1000  ($D000→0 .. $FFFF→$2FFF)
// The read register updates only on a sound-CPU bus cycle (snd_cen_d && rom_cs); rom_hold keeps the
// last value in between, so rom_dout matches a read-enabled BRAM output register clock for clock.
// ---------------------------------------------------------------------------
wire [13:0] rom_cpu_addr = snd_A[13:0] - 14'h1000;  // $D000→0
reg        rom_rd_d;
reg  [7:0] rom_hold;
wire [7:0] rom_dout = rom_rd_d ? rom_d : rom_hold;

assign rom_a = {2'b0, rom_cpu_addr};

always @(posedge clk_20m) begin
    rom_rd_d <= snd_cen_d && rom_cs;
    if (rom_rd_d)
        rom_hold <= rom_d;
end

// ---------------------------------------------------------------------------
// CPU data bus read mux — default $FF for unmapped regions
// Internal 6802 RAM ($0000-$007F) handled externally since cpu68 has no
// built-in RAM (unlike the real MC6802 silicon).
// ---------------------------------------------------------------------------

reg [7:0] sndpia1_dout_r;
always @(posedge clk_20m)
    if (snd_cen && sndpia1_cs_addr)
        sndpia1_dout_r <= sndpia1_dout;

assign snd_Din =
    internal_ram_cs ? internal_ram_dout :
    sndpia2_cs_addr ? sndpia2_dout      :
    sndpia1_cs_addr ? sndpia1_dout      :

    rom_cs          ? rom_dout          :
    8'hFF;

// ---------------------------------------------------------------------------
// DAC + Stereo Volume Attenuation
//
// vol_table maps a 4-bit index (from vol_data) to an 8-bit scale factor.
//   Index 0 = full volume (255), index 15 = minimum (24).
//   Derived from the parallel resistor network in qix_a.cpp.
//
// Scaled output = (dac_val × vol_scale) ÷ 256  (upper byte of 16-bit product)
// MiSTer signed 16-bit: {scaled_byte, 8'h00} − 0x8000
// ---------------------------------------------------------------------------
wire [7:0] dac_val = sndpia1_pb_o;

reg [7:0] vol_table [0:15];
initial begin
    vol_table[0]  = 8'd255; vol_table[1]  = 8'd200;
    vol_table[2]  = 8'd160; vol_table[3]  = 8'd140;
    vol_table[4]  = 8'd128; vol_table[5]  = 8'd112;
    vol_table[6]  = 8'd100; vol_table[7]  = 8'd90;
    vol_table[8]  = 8'd80;  vol_table[9]  = 8'd72;
    vol_table[10] = 8'd64;  vol_table[11] = 8'd56;
    vol_table[12] = 8'd48;  vol_table[13] = 8'd40;
    vol_table[14] = 8'd32;  vol_table[15] = 8'd24;
end

wire [7:0] vol_l = vol_table[vol_data[7:4]];
wire [7:0] vol_r = vol_table[vol_data[3:0]];

// DAC + TMS5200 speech mix — summed BEFORE volume attenuator to match
// the Kram schematic (both feed the same resistor summing junction that
// drives the L/R volume network).
//
// - dac_centered : signed 9-bit (±128)  — unsigned 8-bit DAC centered at $80
// - tms_scaled   : signed 9-bit (~±128) — TMS5220 16-bit sample / 256, rounded to nearest (the chip's idle
//   sample -1 gives 0). Starting point: real HW attenuates SPKR through LM324 stages before summing.
// - mixed        : signed 10-bit (±256 headroom)
// - × vol 8-bit unsigned → signed 19-bit product, slice [17:2] for 16-bit
wire signed [8:0]  dac_centered = $signed({1'b0, dac_val}) - 9'sh80;
wire signed [16:0] tms_round    = $signed({tms_o.audio[15], tms_o.audio}) + 17'sd128;
wire signed [8:0]  tms_scaled   = tms_round[16:8];
wire signed [9:0]  mixed        = dac_centered + tms_scaled;

wire signed [18:0] mix_l_scaled = mixed * $signed({2'b0, vol_l});
wire signed [18:0] mix_r_scaled = mixed * $signed({2'b0, vol_r});

assign audio_l = mix_l_scaled[17:2];
assign audio_r = mix_r_scaled[17:2];


endmodule
