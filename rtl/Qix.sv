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
//   - ROM ioctl address-range dispatch to each subsystem
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

    input         pause,
    output reg    shared_debug_led
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
// Explicit dpram_dc to guarantee M10K inference.
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

dpram_dc #(.widthad_a(10)) shared_ram_inst (
    .clock_a    (clk_20m),
    .address_a  (cpu_sh_cs ? cpu_sh_addr : cpu_sh_addr_held),
    .data_a     (cpu_sh_din),
    .wren_a     (cpu_sh_we),
    .q_a        (cpu_sh_dout),

    .clock_b    (~clk_20m),
    .address_b  (vid_sh_cs ? vid_sh_addr : vid_sh_addr_held),
    .data_b     (vid_sh_din),
    .wren_b     (vid_sh_we),
    .q_b        (vid_sh_dout)
);

// ---------------------------------------------------------------------------
// ROM ioctl address-range dispatch (concatenated ROM, all ioctl_index == 0)
//
// MRA layout (Qix set 2 example, all sets follow same slot structure):
//   $00000-$05FFF : data CPU  (24KB, 6× $1000 slots covering $A000-$FFFF)
//   $06000-$0BFFF : video CPU (24KB, 6× $1000 slots covering $A000-$FFFF)
//   $0C000-$0EFFF : audio CPU (12KB, covers $D000-$FFFF platform max)
//                  Qix: qq27.u27 is 2KB at $F800, padded to land at $0E800
// ---------------------------------------------------------------------------
wire cpu_ioctl_wr = ioctl_wr & (ioctl_addr < 25'h06000);                             // 24KB
wire vid_ioctl_wr = ioctl_wr & (ioctl_addr >= 25'h06000) & (ioctl_addr < 25'h0C000); // 24KB
wire snd_ioctl_wr = ioctl_wr & (ioctl_addr >= 25'h0C000) & (ioctl_addr < 25'h0F000); // 12KB ($D000-$FFFF)
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

// ---------------------------------------------------------------------------
// FIRQ delay configuration
//   MODE 0: No delay (direct latch, current behavior)
//   MODE 1: Half E-cycle delay (~400ns at 1.25 MHz E clock)
//   MODE 2: 1 E-cycle delay (~800ns)
//   MODE 3: 2 E-cycle delay (~1600ns)
//   MODE 4: Alternating execution (MAME-style interleave)
// Change this parameter and recompile to test each mode:
// ---------------------------------------------------------------------------
localparam FIRQ_DELAY_MODE = 0;  // <-- CHANGE THIS TO TEST: 0,1,2,3,4

reg data_firq_latch;
reg video_firq_latch;

generate
if (FIRQ_DELAY_MODE == 0) begin : firq_nodelay
    // --- MODE 0: No delay (current behavior) ---
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

end else if (FIRQ_DELAY_MODE == 5) begin : firq_mame_scanline
    // --- MODE 5: MAME-style end-of-phase FIRQs ---
    reg data_firq_pending;
    reg video_firq_pending;

    always @(posedge clk_20m) begin
        if (reset) begin
            data_firq_pending  <= 1'b0;
            video_firq_pending <= 1'b0;
            data_firq_latch    <= 1'b0;
            video_firq_latch   <= 1'b0;
        end else begin
            // --- Data CPU ---
            if (vid_firq_assert)
                data_firq_pending <= 1'b1;
            if (cpu_E && data_firq_pending) begin
                data_firq_latch  <= 1'b1;
                data_firq_pending <= 1'b0;
            end
            if (cpu_firq_ack)
                data_firq_latch <= 1'b0;

            // --- Video CPU ---
            if (cpu_firq_assert)
                video_firq_pending <= 1'b1;
            if (~cpu_E && video_firq_pending) begin
                video_firq_latch   <= 1'b1;
                video_firq_pending <= 1'b0;
            end
            if (vid_firq_ack)
                video_firq_latch <= 1'b0;
        end
    end

end else if (FIRQ_DELAY_MODE == 4) begin : firq_interleave
    // --- MODE 4: Alternating execution (MAME-style) ---
    // Use E clock phase to alternate: data CPU acts on E=1 half,
    // video CPU acts on E=0 half. FIRQ only asserts when target
    // CPU's phase is active, guaranteeing shared RAM write has settled.
    always @(posedge clk_20m) begin
        if (reset) begin
            data_firq_latch  <= 1'b0;
            video_firq_latch <= 1'b0;
        end else begin
            if (cpu_firq_ack)
                data_firq_latch <= 1'b0;
            else if (vid_firq_assert && cpu_E)
                data_firq_latch <= 1'b1;

            if (vid_firq_ack)
                video_firq_latch <= 1'b0;
            else if (cpu_firq_assert && ~cpu_E)
                video_firq_latch <= 1'b1;
        end
    end

end else begin : firq_delayed
    // --- MODES 1-3: Delayed FIRQ assertion ---
    // Shift register delays the assert pulse by N E-half-cycles
    localparam DELAY_TAPS = (FIRQ_DELAY_MODE == 1) ? 8 :   // half E-cycle (~8 clk_20m ticks)
                            (FIRQ_DELAY_MODE == 2) ? 16 :  // 1 E-cycle (~16 clk_20m ticks)
                                                     32;   // 2 E-cycles (~32 clk_20m ticks)

    reg [31:0] vid_assert_sr;
    reg [31:0] cpu_assert_sr;

    wire vid_assert_delayed = vid_assert_sr[DELAY_TAPS-1];
    wire cpu_assert_delayed = cpu_assert_sr[DELAY_TAPS-1];

    always @(posedge clk_20m) begin
        if (reset) begin
            vid_assert_sr <= 32'd0;
            cpu_assert_sr <= 32'd0;
            data_firq_latch  <= 1'b0;
            video_firq_latch <= 1'b0;
        end else begin
            vid_assert_sr <= {vid_assert_sr[30:0], vid_firq_assert};
            cpu_assert_sr <= {cpu_assert_sr[30:0], cpu_firq_assert};

            if (cpu_firq_ack)           data_firq_latch <= 1'b0;
            else if (vid_assert_delayed) data_firq_latch <= 1'b1;

            if (vid_firq_ack)             video_firq_latch <= 1'b0;
            else if (cpu_assert_delayed)  video_firq_latch <= 1'b1;
        end
    end
end
endgenerate

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
// PIA input assembly — per-game, sourced from MAME qix.cpp INPUT_PORTS
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

        8'h05: begin    // Slither (trackball — joystick not connected, spare inputs handle it)
            // P1: [7]=BTN1 [6]=S1 [5]=S2 [4]=BTN2 [3:0]=1111
            p1_pia = {p1_btn1, start_buttons[0], start_buttons[1], p1_btn2, 4'hF};
            // P2: [7]=BTN1 [6:5]=11 [4]=BTN2 [3:0]=1111
            p2_pia = {p2_btn1, 2'b11, p2_btn2, 4'hF};
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
    .spare_input     (8'hFF),
    .in0_input       (in0_pia),
    .p2_input        (p2_pia),

    .crtc_vsync      (crtc_vsync_out),

    .snd_data_out    (snd_cmd),
    .snd_data_in     (snd_cmd_from_snd),
    .snd_vol_out     (snd_vol),
    .snd_irq_to_snd  (snd_irq_cpu2snd),
    .snd_irq_from_snd(snd_irq_snd2cpu),
    .flip_screen     (flip),

    .ioctl_addr      (ioctl_addr),
    .ioctl_data      (ioctl_data),
    .ioctl_wr        (cpu_ioctl_wr),

    .pause           (pause)
//    .pause           (pause | vid_sh_cs)
);

// ---------------------------------------------------------------------------
// Qix_Video — video CPU board
// ---------------------------------------------------------------------------
Qix_Video video_board (
    .clk_20m         (clk_20m),
    .reset           (reset),
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

    .ioctl_addr      (ioctl_addr),
    .ioctl_data      (ioctl_data),
    .ioctl_wr        (vid_ioctl_wr),

    .hs_address      (hs_address),
    .hs_data_out     (hs_data_out),
    .hs_data_in      (hs_data_in),
    .hs_write        (hs_write),

    .pause           (pause),
//    .pause           (pause | cpu_sh_cs),
    .flip            (flip)
);

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

    .audio_l          (sound_l),
    .audio_r          (sound_r),

    .ioctl_addr       (ioctl_addr),
    .ioctl_data       (ioctl_data),
    .ioctl_wr         (snd_ioctl_wr),

    .pause            (pause)
);

endmodule
