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
    output        video_firq,     // assert FIRQ on video CPU (active-high pulse)
    output        data_firq_ack,
    input         data_firq_n,    // FIRQ input from video CPU (active-low)

    // Player inputs (active-low, from wrapper)
    input  [7:0]  p1_input,       // PIA0 port A: player 1 joystick + buttons
    input  [7:0]  coin_input,     // PIA0 port B: coin / service switches
    input  [7:0]  spare_input,    // PIA1 port A: spare (unused on base Qix)
    input  [7:0]  in0_input,      // PIA1 port B: spare (unused on base Qix)
    input  [7:0]  p2_input,       // PIA2 port A: player 2 joystick

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
// ---------------------------------------------------------------------------
wire shared_cs      = (cpu_A[15:10] == 6'b10_0000);   // $8000-$83FF
wire local_cs       = (cpu_A[15:10] == 6'b10_0001);   // $8400-$87FF
wire acia_cs        = (cpu_A[15:10] == 6'b10_0010);   // $8800-$8BFF (open-bus)
wire firq_range     = (cpu_A[15:10] == 6'b10_0011);   // $8C00-$8FFF
wire firq_assert_cs = firq_range & ~cpu_A[0];         // even addr: assert video FIRQ
wire firq_ack_cs    = firq_range &  cpu_A[0];         // odd  addr: ack data FIRQ
wire sndpia_cs      = (cpu_A[15:10] == 6'b10_0100);   // $9000-$93FF
wire pia0_cs        = (cpu_A[15:10] == 6'b10_0101);   // $9400-$97FF
wire pia1_cs        = (cpu_A[15:10] == 6'b10_0110);   // $9800-$9BFF
wire pia2_cs        = (cpu_A[15:10] == 6'b10_0111);   // $9C00-$9FFF
wire rom_cs         = (cpu_A >= 16'hA000);            // $A000-$FFFF (24KB)

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
mc6809e data_cpu (
    .D          (cpu_Din),
    .DOut       (cpu_Dout),
    .ADDR       (cpu_A),
    .RnW        (cpu_RnW),
    .CLK_ROOT   (clk_20m),
    .CE_E_FALL  (ce_E_fall),
    .CE_Q_FALL  (ce_Q_fall),
    .nIRQ   (n_irq),
    .nFIRQ  (data_firq_n),
    .nNMI   (1'b1),
    .BS     (),
    .BA     (),
    .AVMA   (),
    .BUSY   (),
    .LIC    (),
    .nHALT  (~pause & ~mcu_cpu_halt),
    .nRESET (~reset)
);

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
    .pa_i     (snd_data_in),
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

// PIA0 PB input: for MCU games the MCU's PA output drives this; for non-MCU
// games the raw coin_input switch bus drives it directly.
wire [7:0] mcu_pa_out;
wire [7:0] pia0_pb_i = is_mcu_game ? mcu_pa_out : coin_input;

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

// ---------------------------------------------------------------------------
// PIA1 ($9800-$9BFF) — spare inputs (unused on base Qix)
// ---------------------------------------------------------------------------
wire [7:0] pia1_dout;
wire       pia1_irqa, pia1_irqb;

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
    .pb_o     (),
    .pb_oe    (),
    .cb1      (1'b1),
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
    .cb1      (1'b1),
    .cb2_i    (1'b1),
    .cb2_o    (),
    .cb2_oe   ()
);

// ---------------------------------------------------------------------------
// Data CPU ROM — 16KB ($A000-$FFFF) in 24KB BRAM
//
// Loaded at ioctl_addr $00000-$05FFF (gated by Qix.sv).
// CPU read address: cpu_A[13:0]  ($A000→0 .. $FFFF→$5FFF)
// ioctl write address: ioctl_addr[13:0] (0-based, base $00000)
// ---------------------------------------------------------------------------
reg [7:0] data_rom [0:24575];                        // 24KB
reg [7:0] rom_dout;

wire [14:0] rom_cpu_addr   = cpu_A[14:0] - 15'h2000; // $A000→0, $FFFF→$5FFF
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

// Halt 6809 while MCU processes IRQ — gives MCU time to run handler
// before 6809 reads the response. 64 E-cycles ≈ 51µs at 1.25MHz.
reg [6:0] mcu_halt_cnt = 7'd0;
wire      mcu_cpu_halt = (mcu_halt_cnt != 7'd0) & is_mcu_game;

reg mcu_irq_n_r = 1'b1;
always @(posedge clk_20m) mcu_irq_n_r <= mcu_irq_n;

always @(posedge clk_20m) begin
    if (reset) begin
        mcu_halt_cnt <= 7'd0;
    end else if (is_mcu_game & ~mcu_irq_n & mcu_irq_n_r) begin
        mcu_halt_cnt <= 7'd64;
    end else if (mcu_halt_cnt != 7'd0 && ce_E_fall) begin
        mcu_halt_cnt <= mcu_halt_cnt - 7'd1;
    end
end

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
wire [7:0] mcu_pb_in = {3'b000, coin_input[7], coin_input[3:0]};
wire [3:0] mcu_pc_in = {pia2_pb_o[3], coin_input[6:4]};
wire       mcu_irq_n = ~pia2_pb_o[2];

mc68705p3 mcu (
    .clk      (clk_20m),
    .ce_4m    (mcu_ce_4m),
    .reset    (reset),
    .irq_n    (mcu_irq_n),
    .pa_in    (pia0_pb_o),
    .pa_out   (mcu_pa_out),
    .pb_in    (mcu_pb_in),
    .pb_out   (),
    .pb_ddr   (),
    .pc_in    (mcu_pc_in),
    .pc_out   (),
    .pc_ddr   (),
    .rom_wr   (mcu_rom_wr),
    .rom_addr (mcu_rom_addr),
    .rom_data (mcu_rom_data)
);

// ---------------------------------------------------------------------------
// CPU data bus read mux — default $FF for open-bus / unimplemented reads
// ---------------------------------------------------------------------------

// MCU games: intercept PIA0 port B reads and return MCU PA output directly.
// The PIA ignores pb_i when DDRB=FF (output mode), but the game reads port B
// expecting the MCU's response. Bypassing the PIA here delivers mcu_pa_out
// regardless of DDR state. Non-MCU games read PIA0 normally.

// MCU reply latch — captures mcu_pa_out at the end of the halt window,
// guaranteeing the 6809 sees a stable fully-processed reply.
reg [7:0] mcu_reply_latched = 8'h00;

always @(posedge clk_20m) begin
    if (reset)
        mcu_reply_latched <= 8'h00;
    else if (is_mcu_game && mcu_halt_cnt == 7'd1 && ce_E_fall)
        mcu_reply_latched <= mcu_pa_out;
end

wire pia0_pb_read = pia0_cs & cpu_RnW & (cpu_A[1:0] == 2'b10);

assign cpu_Din =
    shared_cs                        ? shared_dout    :
    local_cs                         ? local_ram_dout :
    acia_cs                          ? 8'h02          :
    sndpia_cs                        ? sndpia_dout    :
    (pia0_pb_read & is_mcu_game)     ? mcu_reply_latched :
    pia0_cs                          ? pia0_dout      :
    pia1_cs                          ? pia1_dout      :
    pia2_cs                          ? pia2_dout      :
    rom_cs                           ? rom_dout       :
    8'hFF;

endmodule
