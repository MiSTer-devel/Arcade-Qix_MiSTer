//============================================================================
//
// Qix Audio Board
// Copyright (C) 2026 Rodimus
//
// Hardware: MC6802 audio CPU (cpu68 core) + 2× PIA6821 + 8-bit DAC +
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

    // ROM loading (MiSTer ioctl — pre-gated by address range in Qix.sv)
    input  [24:0] ioctl_addr,
    input  [7:0]  ioctl_data,
    input         ioctl_wr,

    input         pause
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

// Fractional clock enable: 921.6 kHz from 20 MHz (exact average)
// Real hardware: 7.3728 MHz xtal / 2 = 3.6864 MHz, M6802 internal /4 = 921.6 kHz
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
// cpu68 sets up bus on negedge when snd_cen=1.
// snd_cen_negedge captures that on the same negedge → cs goes high on the
// immediately following posedge (PIA write) AND the following negedge
// (PIA control logic). One clean pulse per CPU bus cycle.
reg snd_cen_negedge;
always @(negedge clk_20m) snd_cen_negedge <= snd_cen;

wire sndpia2_en = snd_cen & sndpia2_cs_addr;
wire sndpia1_en = snd_cen & sndpia1_cs_addr;

// ---------------------------------------------------------------------------
// TMS5200 clock enable — 625 kHz from 20 MHz (/32)
// Nominal chip rate is 640 kHz from RC oscillator (pot-adjustable);
// 625 kHz is 2.3% slow — well within RC oscillator tolerance, inaudible.
// ---------------------------------------------------------------------------
reg [4:0] tms_div = 5'd0;
always @(posedge clk_20m) tms_div <= tms_div + 5'd1;
wire tms_cen = (tms_div == 5'd0) & ~pause;

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

pia6821_sv sndpia1 (
    .clk      (clk_20m),
    .rst      (snd_rst),
    .cs       (sndpia1_en),
    .rw       (snd_rw),
    .addr     (snd_A[1:0]),
    .data_in  (snd_Dout),
    .data_out (sndpia1_dout),
    .irqa     (sndpia1_irqa),
    .irqb     (sndpia1_irqb),
    .pa_i     (snd_data_in),      // data from data CPU
    .pa_o     (sndpia1_pa_o),
    .pa_oe    (sndpia1_pa_oe),
    .ca1      (snd_irq_from_cpu), // CA2 from data CPU sndPIA0
    .ca2_i    (1'b1),
    .ca2_o    (sndpia1_ca2_o),
    .ca2_oe   (sndpia1_ca2_oe),
    .pb_i     (8'h00),
    .pb_o     (sndpia1_pb_o),     // 8-bit DAC value
    .pb_oe    (sndpia1_pb_oe),
    .cb1      (1'b0),
    .cb2_i    (1'b1),
    .cb2_o    (sndpia1_cb2_o),
    .cb2_oe   (sndpia1_cb2_oe)
);

assign snd_data_out   = sndpia1_pa_o;
assign snd_irq_to_cpu = sndpia1_ca2_o;

// ---------------------------------------------------------------------------
// sndPIA2 ($2000-$3FFF) — TMS5200 speech PIA (Kram's "KRAM!" voice)
// Wiring per Kram schematic:
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

// TMS5200 outputs
wire [7:0]         tms_dout;
wire               tms_intn, tms_rdyn;
wire signed [13:0] tms_spkr;

pia6821_sv sndpia2 (
    .clk      (clk_20m),
    .rst      (snd_rst),
    .cs       (sndpia2_en),
    .rw       (snd_rw),
    .addr     (snd_A[1:0]),
    .data_in  (snd_Dout),
    .data_out (sndpia2_dout),
    .irqa     (sndpia2_irqa),
    .irqb     (sndpia2_irqb),
    .pa_i     (8'hFF),            // port A unused
    .pa_o     (),
    .pa_oe    (),
    .ca1      (tms_intn),         // /INT from TMS5200
    .ca2_i    (1'b1),
    .ca2_o    (sndpia2_ca2_o),    // /WS to TMS5200
    .ca2_oe   (sndpia2_ca2_oe),
    .pb_i     (tms_dout),         // D0-D7 from TMS5200 (status reads)
    .pb_o     (sndpia2_pb_o),     // D0-D7 to TMS5200 (command/data writes)
    .pb_oe    (sndpia2_pb_oe),
    .cb1      (tms_rdyn),         // /READY from TMS5200
    .cb2_i    (1'b1),
    .cb2_o    (sndpia2_cb2_o),    // /RS to TMS5200
    .cb2_oe   (sndpia2_cb2_oe)
);

// ---------------------------------------------------------------------------
// TMS5200 speech chip (using TMS5220 core — functionally equivalent for
// the Speak External + data write path that Kram uses)
// ---------------------------------------------------------------------------
TMS5220 u_tms5200 (
    .I_OSC    (clk_20m),
    .I_ENA    (tms_cen),
    .I_WSn    (sndpia2_ca2_o),
    .I_RSn    (sndpia2_cb2_o),
    .I_DATA   (1'b0),              // VSM serial in — unused on Kram
    .I_TEST   (1'b0),
    .I_DBUS   (sndpia2_pb_o),
    .O_DBUS   (tms_dout),
    .O_RDYn   (tms_rdyn),
    .O_INTn   (tms_intn),
    .O_M0     (),                  // VSM control — unused
    .O_M1     (),
    .O_ADD8   (),
    .O_ADD4   (),
    .O_ADD2   (),
    .O_ADD1   (),
    .O_ROMCLK (),                  // would clock VSM ROM; goes to TP6 only
    .O_T11    (),
    .O_IO     (),
    .O_PRMOUT (),
    .O_SPKR   (tms_spkr)
);

// IRQ to audio CPU: OR of all PIA interrupt outputs (active-high)
assign snd_irq = sndpia1_irqa | sndpia1_irqb | sndpia2_irqa | sndpia2_irqb;

// ---------------------------------------------------------------------------
// Audio ROM — 12KB ($D000-$FFFF) in 12KB BRAM
// Loaded at ioctl_addr $0C000-$0EFFF (gated by Qix.sv).
// CPU read address: snd_A[13:0] - $1000  ($D000→0 .. $FFFF→$2FFF)
// ioctl write address: ioctl_addr[13:0] (bits [13:0] of $0C000-$0EFFF = 0-$2FFF)
// ---------------------------------------------------------------------------
reg [7:0] snd_rom [0:12287];                          // 12KB
reg [7:0] rom_dout;

wire [13:0] rom_cpu_addr   = snd_A[13:0] - 14'h1000;  // $D000→0
wire [13:0] rom_ioctl_addr = ioctl_addr[13:0];

//always @(posedge clk_20m) begin
//    if (ioctl_wr)
//        snd_rom[rom_ioctl_addr] <= ioctl_data;
//    rom_dout <= snd_rom[rom_cpu_addr];
//end

// ROM read/write — posedge; ioctl_wr takes priority, else CPU read (no VMA needed with jt680x)
always @(posedge clk_20m)
    if (ioctl_wr)
        snd_rom[rom_ioctl_addr] <= ioctl_data;
    else if (snd_cen_d && rom_cs)
        rom_dout <= snd_rom[rom_cpu_addr];

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
//   Derived from the parallel resistor network in qix_a.cpp (MAME).
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
// - tms_scaled   : signed 9-bit (~±128) — TMS5200 14-bit SPKR >>> 6
//   (starting point — tune on hardware. Real HW attenuates SPKR through
//    LM324 op-amp stages before summing.)
// - mixed        : signed 10-bit (±256 headroom)
// - × vol 8-bit unsigned → signed 19-bit product, slice [17:2] for 16-bit
wire signed [8:0]  dac_centered = $signed({1'b0, dac_val}) - 9'sh80;
wire signed [8:0]  tms_scaled   = tms_spkr >>> 6;
wire signed [9:0]  mixed        = dac_centered + tms_scaled;

wire signed [18:0] mix_l_scaled = mixed * $signed({2'b0, vol_l});
wire signed [18:0] mix_r_scaled = mixed * $signed({2'b0, vol_r});

assign audio_l = mix_l_scaled[17:2];
assign audio_r = mix_r_scaled[17:2];


endmodule
