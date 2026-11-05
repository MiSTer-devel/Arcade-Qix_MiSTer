//============================================================================
//
// Qix Platform Top-Level
// Copyright (C) 2026 Rodimus
//
// Wires together Qix_CPU (data), Qix_Video (video), and Qix_Sound (audio).
//
// Responsibilities:
//   - 6809E E/Q clock generation (1.25 MHz quadrature from 20 MHz)
//   - 1KB shared dual-port RAM (port A = data CPU, port B = video CPU)
//   - FIRQ cross-signal routing between CPUs
//   - Sound PIA signal routing between data CPU and audio board
//   - MCU EPROM download (ioctl index 2)
//   - PIA input assembly
//
//============================================================================

module Qix (
    input         clk_20m,
    input         reset,

    input  [7:0]  game_id,     // 00=Qix 01=ComplexX 02=SpaceDungeon 03=Kram 04=ZooKeep 05=Slither 06=ElecYoYo

    // Player inputs (active-low)
    input  [1:0]  coin,
    input  [1:0]  start_buttons,
    input  [3:0]  p1_joystick,    // {R,L,D,U}
    input  [3:0]  p2_joystick,
    input  [7:0]  trak_x,         // Slither trackball position counters: X on PIA2 port A, Y on PIA1 port A
    input  [7:0]  trak_y,
    input         p1_btn1,        // Player 1 Draw Slow (active-low)
    input         p1_btn2,        // Player 1 Draw Fast (active-low)
    input         p2_btn1,        // Player 2 Draw Slow (active-low)
    input         p2_btn2,        // Player 2 Draw Fast (active-low)
    
    input         service,        // Test Advance button (active-low)
    input         service2,
    input         service3,
    input         service4,

    // Video output
    output        video_hsync,
    output        video_vsync,
    output        video_vblank,
    output        video_hblank,
    output        ce_pix,
    output [7:0]  video_r,
    output [7:0]  video_g,
    output [7:0]  video_b,

    // Audio output (signed 16-bit stereo)
    output signed [15:0] sound_l,
    output signed [15:0] sound_r,

    // ROM loading (MiSTer ioctl)
    input  [24:0] ioctl_addr,
    input  [7:0]  ioctl_data,
    input         ioctl_wr,
    input  [7:0]  ioctl_index,

    // Hiscore interface (hs_address driven externally by hiscore module)
    input  [15:0] hs_address,
    input  [7:0]  hs_data_in,
    output [7:0]  hs_data_out,
    input         hs_write,

    output chip_pool_pkg::m6809r_i_t m6809r_0_i,
    input  chip_pool_pkg::m6809r_o_t m6809r_0_o,
    output chip_pool_pkg::m6809r_i_t m6809r_1_i,
    input  chip_pool_pkg::m6809r_o_t m6809r_1_o,
    output chip_pool_pkg::mc6845_i_t mc6845_0_i,
    input  chip_pool_pkg::mc6845_o_t mc6845_0_o,
    output chip_pool_pkg::mc68705p3_i_t mc68705p3_0_i,
    input  chip_pool_pkg::mc68705p3_o_t mc68705p3_0_o,
    output chip_pool_pkg::pia6821_i_t [5:0] pia6821_i,   // [3:0] Qix_CPU (sndPIA0, PIA0-2), [5:4] Qix_Sound (sndPIA1-2)
    input  chip_pool_pkg::pia6821_o_t [5:0] pia6821_o,
    output chip_pool_pkg::sn76489_i_t [1:0] sn76489_i,   // Qix_CPU: Slither's two PSGs (PIA1 / PIA2 port B)
    input  chip_pool_pkg::sn76489_o_t [1:0] sn76489_o,
    output chip_pool_pkg::tms5220rsws_i_t tms5220rsws_0_i,   // Qix_Sound: TMS5220 on sndPIA2
    input  chip_pool_pkg::tms5220rsws_o_t tms5220rsws_0_o,
    output logic [2:0][15:0] rom_a,              // ROM block read addresses: [0] data CPU ROM  [1] video CPU ROM  [2] sound CPU ROM
    input  logic [2:0] [7:0] rom_d,
    output ram_pool_pkg::pram_i_t [4:0] pr_i,   // [0] NVRAM  [1] VRAM  [2] shared RAM  [3] data CPU local RAM  [4] palette RAM
    input  ram_pool_pkg::pram_o_t [4:0] pr_o,

    input         pause,
    input         crt_flip,       // OSD 180 degree flip, XORed into the cocktail flip
    input  [3:0]  h_center,       // OSD screen centering
    input  [3:0]  v_center
);

// ---------------------------------------------------------------------------
// Clock generation — 6809E E/Q quadrature at 1.25 MHz (20 MHz ÷ 16)
//
// clk_div[3:2] phase:  00 → E=0,Q=0
//                      01 → E=0,Q=1  (Q leads E by 90°)
//                      10 → E=1,Q=1
//                      11 → E=1,Q=0
// ---------------------------------------------------------------------------
reg [3:0] clk_div;
always @(posedge clk_20m) clk_div <= clk_div + 4'd1;

wire cpu_E = clk_div[3];
wire cpu_Q = clk_div[3] ^ clk_div[2];

reg ce_E_fall, ce_Q_fall;
always @(posedge clk_20m) begin
    ce_E_fall <= (clk_div == 4'b1111);
    ce_Q_fall <= (clk_div == 4'b1011);
end

// ---------------------------------------------------------------------------
// Shared 1KB dual-port RAM (port A = data CPU, port B = video CPU)
// Storage: RAM pool block 2 in the dispatcher; its port B is clocked on ~clk_20m.
// ---------------------------------------------------------------------------
wire [9:0] cpu_sh_addr;
wire [7:0] cpu_sh_din;
wire [7:0] cpu_sh_dout;
wire        cpu_sh_we;
wire        cpu_sh_cs;

wire [9:0] vid_sh_addr;
wire [7:0] vid_sh_din;
wire [7:0] vid_sh_dout;
wire        vid_sh_we;
wire        vid_sh_cs;

// Gated shared RAM — hold last valid address when CPU is not accessing
// shared range, preventing cross-port noise from random bus traffic.
reg [9:0] cpu_sh_addr_held;
reg [9:0] vid_sh_addr_held;

always @(posedge clk_20m) begin
    if (cpu_sh_cs) cpu_sh_addr_held <= cpu_sh_addr;
    if (vid_sh_cs) vid_sh_addr_held <= vid_sh_addr;
end

assign pr_i[2].a_addr = {6'd0, cpu_sh_cs ? cpu_sh_addr : cpu_sh_addr_held};
assign pr_i[2].a_d    = cpu_sh_din;
assign pr_i[2].a_we   = cpu_sh_we;
assign pr_i[2].b_addr = {6'd0, vid_sh_cs ? vid_sh_addr : vid_sh_addr_held};
assign pr_i[2].b_d    = vid_sh_din;
assign pr_i[2].b_we   = vid_sh_we;
assign cpu_sh_dout    = pr_o[2].a_q;
assign vid_sh_dout    = pr_o[2].b_q;

// ---------------------------------------------------------------------------
// Game flags (ROM download layout per game: dispatcher Shared ROM blocks)
// ---------------------------------------------------------------------------
wire is_slither_top = (game_id == 8'h05);

// MCU EPROM (MC68705P3, 2KB) loaded via ioctl_index == 2
wire        mcu_ioctl_wr   = ioctl_wr & (ioctl_index == 8'd2);
wire [10:0] mcu_ioctl_addr = ioctl_addr[10:0];
// ---------------------------------------------------------------------------
// FIRQ cross-signals (from schematic Figure 13, U7 7474 dual flip-flop)
//
// Two SR latches using async PRE/CLR:
//   data_firq_latch:  SET by video CPU $8C00, CLEAR by data CPU $8C01
//   video_firq_latch: SET by data CPU $8C00, CLEAR by video CPU $8C01
// ---------------------------------------------------------------------------
wire cpu_firq_assert;    // data CPU accessed $8C00 (pulse)
wire cpu_firq_ack;       // data CPU accessed $8C01 (pulse)
wire vid_firq_assert;    // video CPU accessed $8C00 (pulse)
wire vid_firq_ack;       // video CPU accessed $8C01 (pulse)

// FIRQ latches (U7 7474): set by the other CPU's $8C00 access, cleared by the own CPU's $8C01 access
// ---------------------------------------------------------------------------
reg data_firq_latch;
reg video_firq_latch;

always @(posedge clk_20m) begin
    if (reset) begin
        data_firq_latch  <= 1'b0;
        video_firq_latch <= 1'b0;
    end else begin
        if (cpu_firq_ack)         data_firq_latch <= 1'b0;
        else if (vid_firq_assert) data_firq_latch <= 1'b1;

        if (vid_firq_ack)           video_firq_latch <= 1'b0;
        else if (cpu_firq_assert)   video_firq_latch <= 1'b1;
    end
end

wire data_firq_n  = ~data_firq_latch;
wire video_firq_n = ~video_firq_latch;

wire crtc_vsync_out;

// ---------------------------------------------------------------------------
// Sound PIA signal routing
//   sndPIA0 lives in Qix_CPU; sndPIA1 lives in Qix_Sound.
// ---------------------------------------------------------------------------
wire [7:0] snd_cmd;              // Qix_CPU sndPIA0 PA out → Qix_Sound sndPIA1 PA in
wire [7:0] snd_cmd_from_snd;     // Qix_Sound sndPIA1 PA out → Qix_CPU sndPIA0 PA in
wire [7:0] snd_vol;              // Qix_CPU sndPIA0 PB out → Qix_Sound vol_data
wire        snd_irq_cpu2snd;     // Qix_CPU sndPIA0 CA2 → Qix_Sound sndPIA1 CA1
wire        snd_irq_snd2cpu;     // Qix_Sound sndPIA1 CA2 → Qix_CPU sndPIA0 CA1
wire        flip;                // Qix_CPU sndPIA0 CB2 → Qix_Video flip

// ---------------------------------------------------------------------------
// PIA input assembly — per-game, from the qix.cpp INPUT_PORTS
//
// Joystick bit order from wrapper: p1_joystick = {R,L,D,U} = [3:2:1:0]
// PIA byte bits [3:0] need L,D,R,U order = [2],[1],[3],[0]
//
// game_id: 00=Qix/Qix2  01=ComplexX  02=SpaceDungeon
//          03=Kram       04=ZooKeep   05=Slither  06=ElecYoYo
// ---------------------------------------------------------------------------

wire [7:0] coin_pia = {1'b1, 1'b1, coin[1], coin[0], service4, service3, service2, service};

// Joystick nibble shorthand (active-low, L/D/R/U → PIA [3:0])
wire [3:0] p1_joy_pia = {p1_joystick[2], p1_joystick[1], p1_joystick[3], p1_joystick[0]};
wire [3:0] p2_joy_pia = {p2_joystick[2], p2_joystick[1], p2_joystick[3], p2_joystick[0]};

// PIA1 port A: trackball Y on Slither, unused (pulled high) on every other game.
wire [7:0] spare_pia = is_slither_top ? trak_y : 8'hFF;

reg [7:0] p1_pia;
reg [7:0] p2_pia;
reg [7:0] in0_pia;  // PIA1 port B — normally 0xFF, Space Dungeon uses [1:0] for starts

always @(*) begin
    // defaults
    p1_pia  = 8'hFF;
    p2_pia  = 8'hFF;
    in0_pia = 8'hFF;

    case (game_id)

        8'h00,          // Qix / Qix II
        8'h03: begin    // Kram (same P1 layout; Kram P2 has BTN2 at [4] but [6:5]=11 so we reuse p2_btn2)
            // P1: [7]=BTN1 [6]=S1 [5]=S2 [4]=BTN2 [3:0]=L,D,R,U
            p1_pia = {p1_btn1, start_buttons[0], start_buttons[1], p1_btn2, p1_joy_pia};
            // P2: [7]=BTN1 [6:4]=111 [3:0]=L,D,R,U  (Kram has BTN2 at [4], map p2_btn2 there)
            p2_pia = {p2_btn1, 3'b111, p2_joy_pia};
        end

        8'h01: begin    // Complex X
            // P1 (left stick): [7]=1(unused) [6]=S1 [5]=S2 [4]=BTN1(jump) [3:0]=L,D,R,U
            p1_pia = {1'b1, start_buttons[0], start_buttons[1], p1_btn1, p1_joy_pia};
            // P2 (right stick = fire direction): [7:4]=L,D,R,U  [3:0]=1111
            p2_pia = {p2_joystick[2], p2_joystick[1], p2_joystick[3], p2_joystick[0], 4'hF};
        end

        8'h02: begin    // Space Dungeon
            // P1: [7:4]=RIGHT_L,D,R,U  [3:0]=LEFT_L,D,R,U  (both sticks in PIA0 PA)
            // We reuse p2_joystick as right stick, p1_joystick as left stick
            p1_pia  = {p2_joystick[2], p2_joystick[1], p2_joystick[3], p2_joystick[0],
                       p1_joy_pia};
            // P2 (cocktail — left stick same mapping)
            p2_pia  = {p2_joystick[2], p2_joystick[1], p2_joystick[3], p2_joystick[0],
                       p1_joy_pia};
            // START1/START2 live in PIA1 port B bits [1:0]
            in0_pia = {6'b111111, start_buttons[1], start_buttons[0]};
        end

        8'h04: begin    // Zoo Keeper
            // P1: [7]=1 [6]=BTN1 [5]=S2 [4]=S1 [3:0]=L,D,R,U
            p1_pia = {1'b1, p1_btn1, start_buttons[1], start_buttons[0], p1_joy_pia};
            // P2: [7]=1 [6]=BTN1 [5:4]=11 [3:0]=L,D,R,U
            p2_pia = {1'b1, p2_btn1, 2'b11, p2_joy_pia};
        end

        8'h05: begin    // Slither (trackball)
            // P1: [7]=BTN1 [6]=S1 [5]=S2 [4]=BTN2 [3:0]=unused — matches INPUT_PORTS(slither)
            p1_pia = {p1_btn1, start_buttons[0], start_buttons[1], p1_btn2, 4'hF};
            // PIA2 port A is the trackball X axis, NOT player-2 buttons
            // (slither(machine_config&): m_pia[2]->readpa_handler = trak_r<0> = AN0).
            p2_pia = trak_x;
        end

        8'h06: begin    // Electric Yo-Yo
            // P1: [7]=1 [6]=S1 [5]=S2 [4]=1 [3:0]=L,D,R,U
            p1_pia = {1'b1, start_buttons[0], start_buttons[1], 1'b1, p1_joy_pia};
            // P2: [7:4]=1111 [3:0]=L,D,R,U
            p2_pia = {4'hF, p2_joy_pia};
        end

        default: begin  // fallback = Qix
            p1_pia = {p1_btn1, start_buttons[0], start_buttons[1], p1_btn2, p1_joy_pia};
            p2_pia = {p2_btn1, 3'b111, p2_joy_pia};
        end

    endcase
end

// ---------------------------------------------------------------------------
// Qix_CPU — data CPU board
// ---------------------------------------------------------------------------
Qix_CPU cpu_board (
    .clk_20m         (clk_20m),
    .reset           (reset),
    .ce_E_fall       (ce_E_fall),
    .ce_Q_fall       (ce_Q_fall),

    .shared_addr     (cpu_sh_addr),
    .shared_din      (cpu_sh_din),
    .shared_dout     (cpu_sh_dout),
    .shared_we       (cpu_sh_we),
    .shared_cs_o     (cpu_sh_cs),

    .video_firq      (cpu_firq_assert),
    .data_firq_ack   (cpu_firq_ack),
    .data_firq_n     (data_firq_n),

    .p1_input        (p1_pia),
    .coin_input      (coin_pia),
    .spare_input     (spare_pia),
    .in0_input       (in0_pia),
    .p2_input        (p2_pia),

    .sn_i            (sn76489_i),
    .sn_o            (sn76489_o),

    .crtc_vsync      (crtc_vsync_out),

    .snd_data_out    (snd_cmd),
    .snd_data_in     (snd_cmd_from_snd),
    .snd_vol_out     (snd_vol),
    .snd_irq_to_snd  (snd_irq_cpu2snd),
    .snd_irq_from_snd(snd_irq_snd2cpu),
    .flip_screen     (flip),

    .rom_a           (rom_a[0]),
    .rom_d           (rom_d[0]),

    .mcu_rom_addr    (mcu_ioctl_addr),
    .mcu_rom_data    (ioctl_data),
    .mcu_rom_wr      (mcu_ioctl_wr),
    .mcu_i           (mc68705p3_0_i),
    .mcu_o           (mc68705p3_0_o),

    .m6809_i         (m6809r_0_i),
    .m6809_o         (m6809r_0_o),
    .pia_i           (pia6821_i[3:0]),
    .pia_o           (pia6821_o[3:0]),
    .lr_pr_i         (pr_i[3]),
    .lr_pr_o         (pr_o[3]),

    .pause           (pause),
    .game_id         (game_id)
);

// ---------------------------------------------------------------------------
// Qix_Video — video CPU board
//
// Video CPU reset is staggered after the data CPU to fix a shared RAM race:
// EYY (and potentially others) hang when the video CPU reads $8013
// before the data CPU has initialised it.
// At 20MHz, 2000 cycles ≈ 0.1ms gives the data CPU time to clear $03F3
// before the video CPU boots and asserts its init FIRQ. The window must be
// LESS than ~10000 cycles (when the data CPU sets $03F3=$80 for game-running).
reg [16:0] vid_reset_cnt = 17'd0;
always @(posedge clk_20m) begin
    if (reset) vid_reset_cnt <= 17'd0;
    else if (vid_reset_cnt != 17'h1FFFF) vid_reset_cnt <= vid_reset_cnt + 17'd1;
end
wire video_reset = reset | (vid_reset_cnt < 17'd2000);

Qix_Video video_board (
    .clk_20m         (clk_20m),
    .reset           (video_reset),
    .ce_E_fall       (ce_E_fall),
    .ce_Q_fall       (ce_Q_fall),

    .shared_addr     (vid_sh_addr),
    .shared_dout     (vid_sh_din),    // video CPU write data → shared RAM port B
    .shared_din      (vid_sh_dout),   // shared RAM port B read → video CPU
    .shared_we       (vid_sh_we),
    .shared_cs_o     (vid_sh_cs),

    .data_firq       (vid_firq_assert),
    .video_firq_ack  (vid_firq_ack),
    .video_firq_n    (video_firq_n),

    .hsync           (video_hsync),
    .vsync           (video_vsync),
    .hblank          (video_hblank),
    .vblank          (video_vblank),
    .ce_pix          (ce_pix),
    .video_r         (video_r),
    .video_g         (video_g),
    .video_b         (video_b),
    .crtc_vsync      (crtc_vsync_out),

    .rom_a           (rom_a[1]),
    .rom_d           (rom_d[1]),

    .hs_address      (hs_address),
    .hs_data_out     (hs_data_out),
    .hs_data_in      (hs_data_in),
    .hs_write        (hs_write),

    .m6809_i         (m6809r_1_i),
    .m6809_o         (m6809r_1_o),
    .mc6845_i        (mc6845_0_i),
    .mc6845_o        (mc6845_0_o),
    .nv_pr_i         (pr_i[0]),
    .nv_pr_o         (pr_o[0]),
    .vram_pr_i       (pr_i[1]),
    .vram_pr_o       (pr_o[1]),
    .pal_pr_i        (pr_i[4]),
    .pal_pr_o        (pr_o[4]),

    .pause           (pause),
//    .pause           (pause | cpu_sh_cs),
    .flip            (flip ^ crt_flip),
    .h_center        (h_center),
    .v_center        (v_center),
    .game_id         (game_id)
);

// ---------------------------------------------------------------------------
// Audio output select
//
// Slither has no 6802/DAC board — its audio is the two SN76489s hung off PIA1/
// PIA2 port B (mono, summed). Every other game keeps the DAC board untouched.
// ---------------------------------------------------------------------------
wire signed [15:0] dac_audio_l, dac_audio_r;

// Each PSG 0..1020, the real chip's level with its DC (MiSTer's audio DC blocker plays the coupling capacitor)
wire        [11:0] sn_sum = {1'b0, sn76489_o[0].sound} + {1'b0, sn76489_o[1].sound};   // 0..2040
wire signed [15:0] sn_mix = $signed({4'd0, sn_sum}) <<< 4;                            // 0..32640

assign sound_l = is_slither_top ? sn_mix : dac_audio_l;
assign sound_r = is_slither_top ? sn_mix : dac_audio_r;

// ---------------------------------------------------------------------------
// Qix_Sound — audio board
// ---------------------------------------------------------------------------
Qix_Sound sound_board (
    .clk_20m          (clk_20m),
    .reset            (reset),

    .snd_data_in      (snd_cmd),
    .snd_data_out     (snd_cmd_from_snd), // sound→data CPU reply path
    .snd_irq_from_cpu (snd_irq_cpu2snd),
    .snd_irq_to_cpu   (snd_irq_snd2cpu),

    .vol_data         (snd_vol),

    .audio_l          (dac_audio_l),
    .audio_r          (dac_audio_r),

    .rom_a            (rom_a[2]),
    .rom_d            (rom_d[2]),

    .pause            (pause),
    .pia_i            (pia6821_i[5:4]),
    .pia_o            (pia6821_o[5:4]),
    .tms_i            (tms5220rsws_0_i),
    .tms_o            (tms5220rsws_0_o)
);

endmodule
