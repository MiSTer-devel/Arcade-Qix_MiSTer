//============================================================================
//
// Qix Platform for MiSTer
// Copyright (C) 2026 Rodimus
// Based on Tutankham core structure
//
//  Permission is hereby granted, free of charge, to any person obtaining a
//  copy of this software and associated documentation files (the "Software"),
//  to deal in the Software without restriction, including without limitation
//  the rights to use, copy, modify, merge, publish, distribute, sublicense,
//  and/or sell copies of the Software, and to permit persons to whom the
//  Software is furnished to do so, subject to the following conditions:
//
//  The above copyright notice and this permission notice shall be included in
//  all copies or substantial portions of the Software.
//
//  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
//  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
//  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
//  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
//  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
//  FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
//  DEALINGS IN THE SOFTWARE.
//
//============================================================================

module emu
(
    `include "sys/emu_ports.vh"
);

assign ADC_BUS  = 'Z;
assign USER_OUT = '1;
assign {UART_RTS, UART_TXD, UART_DTR} = 0;
assign {SD_SCK, SD_MOSI, SD_CS} = 'Z;
assign {SDRAM_DQ, SDRAM_A, SDRAM_BA, SDRAM_CLK, SDRAM_CKE, SDRAM_DQML, SDRAM_DQMH, SDRAM_nWE, SDRAM_nCAS, SDRAM_nRAS, SDRAM_nCS} = 'Z;

assign VGA_F1 = 0;
assign VGA_SCALER = 0;
assign VGA_DISABLE = 0;
assign FB_FORCE_BLANK = 0;
assign HDMI_FREEZE = 0;
assign HDMI_BLACKOUT = 0;
assign HDMI_BOB_DEINT = 0;

wire signed [15:0] audio_l, audio_r;
assign AUDIO_L = audio_l;
assign AUDIO_R = audio_r;
assign AUDIO_S = 1;   // signed
assign AUDIO_MIX = 0; // no mix, true stereo

assign LED_DISK  = 0;
assign LED_POWER = 0;
//assign LED_USER  = ioctl_download;
assign LED_USER  = ioctl_download | shared_debug_led_w;
assign BUTTONS = 0;

wire shared_debug_led_w;

///////////////////////////////////////////////////

// VIDEO_ARX/ARY are assigned below, next to the orientation logic, because the
// aspect ratio must follow the GAME's orientation (the `horizontal` wire from
// game_id) — NOT the status[12] monitor toggle alone. game_id/horizontal are
// declared further down, so the assigns live there to stay in scope.

`include "build_id.v"
localparam CONF_STR = {
	"QIX;;",
	"P1,Video Options;",
	"P1ODE,Aspect Ratio,Original,Full screen,[ARC1],[ARC2];",
	"P1OC,Orientation,Vert,Horz;",
	"P1OB,HDMI Flip,Off,On;",
	"P1OM,CRT Flip,Off,On;",
	"P1OFH,Scandoubler Fx,None,HQ2x,CRT 25%,CRT 50%,CRT 75%;",
	"-;",
	"OR,Autosave NVRAM,Off,On;",
	"-;",
	"P2,Pause Options;",
	"P2OP,Pause when OSD is open,On,Off;",
	"P2OQ,Dim video after 10s,On,Off;",
	"-;",
	"P3,Screen Centering;",
	"P3O36,H Center,0,-1,-2,-3,-4,-5,-6,-7,+7,+6,+5,+4,+3,+2,+1;",
	"P3O7A,V Center,0,-1,-2,-3,-4,-5,-6,-7,+7,+6,+5,+4,+3,+2,+1;",
	"-;",
	"H1P4,Game Options;",
	"H1P4OIJ,Trackball Speed,Normal,Fast,Slow;",
	"H1-;",
	"R0,Reset;",
	"J1,Button 1,Button 2,Test Advance,Start 2P(alt),Coin,Start 1P,Start 2P,Pause;",
	"jn,A,B,X,Y,Select,Start,R,L;",
	"V,v",`BUILD_DATE
};

wire        forced_scandoubler;
wire  [1:0] buttons;
wire [31:0] status;
wire [10:0] ps2_key;

wire        ioctl_download;
wire        ioctl_upload;
wire        ioctl_upload_req;
wire  [7:0] ioctl_index;
wire        ioctl_wr;
wire [24:0] ioctl_addr;
wire  [7:0] ioctl_dout;
wire  [7:0] ioctl_din;

wire [15:0] joystick_0, joystick_1;
wire [15:0] joystick_l_analog_0;
wire [24:0] ps2_mouse;

reg [7:0] game_id = 8'h00;
wire [15:0] joy = joystick_0 | joystick_1;
wire [15:0] joystick_r_analog_0;   // right analog stick: [15:8]=Y signed, [7:0]=X signed

wire [21:0] gamma_bus;
wire        direct_video;

hps_io #(.CONF_STR(CONF_STR)) hps_io
(
	.clk_sys(CLK_20M),
	.HPS_BUS(HPS_BUS),
	.EXT_BUS(),
	.gamma_bus(gamma_bus),
	.direct_video(direct_video),
	.video_rotated(video_rotated),

	.forced_scandoubler(forced_scandoubler),

	.buttons(buttons),
	.status(status),
	.status_menumask({game_id != 8'h05, direct_video}),

	.ioctl_download(ioctl_download),
	.ioctl_upload(ioctl_upload),
	.ioctl_upload_req(ioctl_upload_req),
	.ioctl_wr(ioctl_wr),
	.ioctl_addr(ioctl_addr),
	.ioctl_dout(ioctl_dout),
	.ioctl_din(ioctl_din),
	.ioctl_index(ioctl_index),

	.joystick_0(joystick_0),
	.joystick_1(joystick_1),
	.joystick_r_analog_0(joystick_r_analog_0),
	.joystick_l_analog_0(joystick_l_analog_0),
	.ps2_mouse(ps2_mouse),
	.ps2_key(ps2_key)
);

////////////////////   CLOCKS   ///////////////////
wire CLK_40M;
wire CLK_20M;
wire locked;

wire [63:0] reconfig_to_pll;
wire [63:0] reconfig_from_pll;

pll pll
(
    .refclk(CLK_50M),
    .rst(0),
    .outclk_0(CLK_40M),
    .outclk_1(CLK_20M),
    .locked(locked),
    .reconfig_to_pll(reconfig_to_pll),
    .reconfig_from_pll(reconfig_from_pll)
);

pll_cfg pll_cfg
(
    .mgmt_clk(CLK_50M),
    .mgmt_reset(0),
    .mgmt_waitrequest(),
    .mgmt_read(0),
    .mgmt_readdata(),
    .mgmt_write(0),
    .mgmt_address(0),
    .mgmt_writedata(0),
    .reconfig_to_pll(reconfig_to_pll),
    .reconfig_from_pll(reconfig_from_pll)
);

assign CLK_VIDEO = CLK_40M;   // HDMI needs the 40 MHz reference

wire reset = RESET | status[0] | buttons[1] | ioctl_download;

// game_id: latched from ioctl index 1 (single byte written by MRA)
always @(posedge CLK_20M) begin
    if (ioctl_wr && ioctl_index == 8'd1)
        game_id <= ioctl_dout;
end
// 00=Qix  01=ComplexX  02=SpaceDungeon  03=Kram  04=ZooKeep  05=Slither  06=ElecYoYo

wire pause_cpu;                     // CTL2-2026-10-25: declared here (was at the pause instance; the keyboard uses it)

///////////////////         Keyboard           //////////////////

// CTL-2026-10-24: original block commented out below (replaced by rtl/mister_custom/controllers.sv modules after it)
// reg btn_up       = 0;
// reg btn_down     = 0;
// reg btn_left     = 0;
// reg btn_right    = 0;
// reg btn_fire     = 0;
// reg btn_fire2    = 0;
// reg btn_coin1    = 0;
// reg btn_coin2    = 0;
// reg btn_1p_start = 0;
// reg btn_2p_start = 0;
// reg btn_pause    = 0;
// reg btn_service  = 0;
// reg btn_service2 = 0;
// reg btn_service3 = 0;
// reg btn_service4 = 0;
//
// wire pressed = ~ps2_key[9];
// wire [7:0] code = ps2_key[7:0];
// always @(posedge CLK_20M) begin
// 	reg old_state;
// 	old_state <= ps2_key[10];
// 	if(old_state != ps2_key[10]) begin
// 		case(code)
// 			'h16: btn_1p_start <= pressed; // 1 = Player 1 Start
// 			'h1E: btn_2p_start <= pressed; // 2 = Player 2 Start
// 			'h2E: btn_coin1    <= pressed; // 5 = Coin Input 1
// 			'h36: btn_coin2    <= pressed; // 6 = Coin Input 2
// 			'h4D: btn_pause    <= pressed; // P = Pause
// 			'h46: btn_service  <= pressed; // 9 = Test Advance
// 			'h45: btn_service2 <= pressed; // 0 = Test Next Line
// 			'h44: btn_service3 <= pressed; // O = Test Slew Up  
// 			'h4B: btn_service4 <= pressed; // L = Test Slew Down
//
// 			'h75: btn_up       <= pressed; // up         = Up
// 			'h72: btn_down     <= pressed; // down       = Down
// 			'h6B: btn_left     <= pressed; // left       = Left
// 			'h74: btn_right    <= pressed; // right      = Right
// 			'h14: btn_fire     <= pressed; // ctrl       = Draw Slow
// 			'h12: btn_fire2    <= pressed; // left shift = Draw Fast
// 		endcase 
// 	end
// end
// CTL-2026-10-24: keyboard via ctl_keyboard (keys act on press; the old ~ps2_key[9] acted on release)
// CTL2-2026-10-25: original below (extra keys now in ctl_keyboard's table)
// /* verilator lint_off UNUSED */
// wire [12:0] kb;                    // kb[5] Alt / kb[11] T are not used by Qix
// /* verilator lint_on UNUSED */
// ctl_keyboard u_kb(.clk(CLK_20M), .ps2_key(ps2_key), .kb(kb));
// kbx: [0] left Shift = Draw Fast, [1] 0 = Test Next Line, [2] O = Test Slew Up, [3] L = Test Slew Down
/* verilator lint_off UNUSED */
wire [12:0] kb;                    // kb[5] Alt / kb[11] T are not used by Qix
/* verilator lint_on UNUSED */
wire  [3:0] kbx;
ctl_keyboard #(.NX(4), .NXO(4), .XKEYS({24'h10_4B_03, 24'h10_44_02, 24'h10_45_01, 24'h10_12_00})) u_kb
(
	.clk(CLK_20M), .ps2_key(ps2_key), .pause(pause_cpu), .kb(kb), .kbx(kbx)
);
wire btn_up       = kb[0];   // up         = Up
wire btn_down     = kb[1];   // down       = Down
wire btn_left     = kb[2];   // left       = Left
wire btn_right    = kb[3];   // right      = Right
wire btn_fire     = kb[4];   // ctrl       = Draw Slow
wire btn_1p_start = kb[6];   // 1 = Player 1 Start
wire btn_2p_start = kb[7];   // 2 = Player 2 Start
wire btn_coin1    = kb[8];   // 5 = Coin Input 1
wire btn_coin2    = kb[9];   // 6 = Coin Input 2 (m_coin2 is never assigned, unchanged)
wire btn_service  = kb[10];  // 9 = Test Advance
wire btn_pause    = kb[12];  // P = Pause
// CTL2-2026-10-25: original local extra-key block commented out below (now ctl_keyboard kbx)
// // keys ctl_keyboard does not carry (left shift, 0, O, L): local, same on-press polarity
// reg btn_fire2     = 0;
// reg btn_service2  = 0;
// reg btn_service3  = 0;
// reg btn_service4  = 0;
// reg kbx_old_state = 1'b0;
// always @(posedge CLK_20M) begin
// 	kbx_old_state <= ps2_key[10];
// 	if (kbx_old_state != ps2_key[10]) begin
// 		case (ps2_key[7:0])
// 			8'h12: btn_fire2    <= ps2_key[9]; // left shift = Draw Fast
// 			8'h45: btn_service2 <= ps2_key[9]; // 0 = Test Next Line
// 			8'h44: btn_service3 <= ps2_key[9]; // O = Test Slew Up
// 			8'h4B: btn_service4 <= ps2_key[9]; // L = Test Slew Down
// 			default: ;
// 		endcase
// 	end
// end
wire btn_fire2    = kbx[0];  // left shift = Draw Fast
wire btn_service2 = kbx[1];  // 0 = Test Next Line
wire btn_service3 = kbx[2];  // O = Test Slew Up
wire btn_service4 = kbx[3];  // L = Test Slew Down

//////////////////  Arcade Buttons/Interfaces   ///////////////////////////

//Player 1
wire m_up1      = btn_up        | joystick_0[3];
wire m_down1    = btn_down      | joystick_0[2];
wire m_left1    = btn_left      | joystick_0[1];
wire m_right1   = btn_right     | joystick_0[0];
wire m_btn1_p1  = btn_fire      | joystick_0[4];
wire m_btn2_p1  = btn_fire2     | joystick_0[5];

//Player 2
wire m_up2      = btn_up        | joystick_1[3];
wire m_down2    = btn_down      | joystick_1[2];
wire m_left2    = btn_left      | joystick_1[1];
wire m_right2   = btn_right     | joystick_1[0];
wire m_btn1_p2  = btn_fire      | joystick_1[4];
wire m_btn2_p2  = btn_fire2     | joystick_1[5];

// Right analog stick → 8 directional fire bits (threshold = 64/127)
// joystick_r_analog_0[7:0]  = X axis (signed), positive = right
// joystick_r_analog_0[15:8] = Y axis (signed), positive = down
// CTL-2026-10-24: original block commented out below (replaced by rtl/mister_custom/controllers.sv modules after it)
// wire signed [7:0] rstick_x = joystick_r_analog_0[7:0];
// wire signed [7:0] rstick_y = joystick_r_analog_0[15:8];
// wire r_fire_right = (rstick_x >  8'sd64);
// wire r_fire_left  = (rstick_x < -8'sd64);
// wire r_fire_down  = (rstick_y >  8'sd64);
// wire r_fire_up    = (rstick_y < -8'sd64);
// CTL-2026-10-24: right stick -> fire directions via ctl_stick_dirs (per axis past +-64, {U, D, L, R})
wire [3:0] r_fire;
ctl_stick_dirs #(.MODE(0), .THRESH(7'd64)) u_rstick(.stick(joystick_r_analog_0), .dir(r_fire));
wire r_fire_up    = r_fire[3];
wire r_fire_down  = r_fire[2];
wire r_fire_left  = r_fire[1];
wire r_fire_right = r_fire[0];

// Dual-stick fire: right analog OR joystick_1 DPAD
// Used by Complex X (game_id 01) and Space Dungeon (game_id 02)
wire m_fire_up    = r_fire_up    | joystick_1[3];
wire m_fire_down  = r_fire_down  | joystick_1[2];
wire m_fire_left  = r_fire_left  | joystick_1[1];
wire m_fire_right = r_fire_right | joystick_1[0];

// For dual-stick games the jump/action button moves to Button 2 (R shoulder)
// so the right thumb is free for the fire stick.
// We use the registered game_id from our own latch above.
wire dual_stick_game = (game_id == 8'h01) || (game_id == 8'h02);

// Dual-stick games: jump on R1 [8] so right thumb stays free for fire stick
// Normal games: btn1 = A [4] as usual
wire m_jump     = btn_fire | joystick_0[10];   // R1 = jump for dual-stick
wire m_p1_btn1  = dual_stick_game ? m_jump : m_btn1_p1;
wire m_p1_btn2  = m_btn2_p1;

//Start/Coin
wire m_coin1    = btn_coin1     | joystick_0[8] ;
wire m_start1   = btn_1p_start  | joystick_0[9] ;
wire m_start2   = btn_2p_start  | joystick_0[10];
wire m_pause    = btn_pause     | joystick_0[11];   // L1 = Pause

//Service Mode
wire m_service  = btn_service   | joystick_0[6] ; // X button = Test Advance
wire m_service2 = btn_service2                  ;
wire m_service3 = btn_service3                  ;
wire m_service4 = btn_service4                  ;

// PAUSE SYSTEM
// CTL2-2026-10-25: original below (pause_cpu is now declared above the keyboard block)
// wire pause_cpu;
wire [23:0] rgb_out;
pause #(8,8,8,20) pause
(
	.*,
	.clk_sys(CLK_20M),
	.user_button(m_pause),
	.pause_request(1'b0),
	.options(~status[26:25])
);

///////////////                 Video                  ////////////////

wire hblank, vblank;
wire hs, vs;
wire ce_pix;

// ============================================================================
wire [7:0]  r, g, b;


// ROT270 (CCW): Qix, ComplexX, SpaceDungeon, ElecYoYo, Slither
// ROT0 (none):  Kram, Zoo Keeper
wire horizontal = (game_id == 8'h03) | (game_id == 8'h04);  // Kram or Zoo Keeper
wire rotate_ccw = 1;
wire no_rotate  = (status[12] ^ horizontal) | direct_video;
wire flip       = status[11];

// --- Output aspect ratio: follow the GAME orientation, not status[12] alone ---
// The image is landscape exactly when screen_rotate passes through, i.e. the
// SAME term that drives no_rotate: (status[12] ^ horizontal). Horz game -> 4:3,
// Vert game -> 3:4 (the arcade MONITOR ratio, NOT the framebuffer pixel ratio).
// status[12] only flips it when the user rotates the monitor, so the AR hint
// always tracks what is actually displayed.
// See Common-Pitfalls/"Aspect ratio set to framebuffer pixel ratio".
wire [1:0] ar = status[14:13];
wire landscape = status[12] ^ horizontal;
assign VIDEO_ARX = landscape ? ((!ar) ? 12'd4 : (ar - 1'd1)) : ((!ar) ? 12'd3 : (ar - 1'd1));
assign VIDEO_ARY = landscape ? ((!ar) ? 12'd3 : 12'd0)       : ((!ar) ? 12'd4 : 12'd0);

screen_rotate screen_rotate(.*);

arcade_video #(256,24) arcade_video
(
	.*,

	.clk_video(CLK_40M),

	.RGB_in(rgb_out),
	.HBlank(hblank),
	.VBlank(vblank),
	.HSync(hs),
	.VSync(vs),

	.fx(status[17:15])
);

// CTL-2026-10-24: original block commented out below (replaced by rtl/mister_custom/controllers.sv modules after it)
// // Slither trackball: mouse deltas as they arrive, left stick (proportional) or d-pad once a frame.
// // Port byte = counter scaled by Trackball Speed: Normal = MAME's 50%, Fast = 1:1, Slow = 1:4
// reg  [9:0] trak_x_c = 10'd0, trak_y_c = 10'd0;
// reg        mouse_tog = 1'b0, trak_vbl = 1'b0;
// wire signed [7:0] ana_x = joystick_l_analog_0[7:0];
// wire signed [7:0] ana_y = joystick_l_analog_0[15:8];
// wire [9:0] trak_dx = (ana_x > 8'sd12 || ana_x < -8'sd12) ? {{5{ana_x[7]}}, ana_x[7:3]} :
//                      m_right1 ? 10'd10 : m_left1 ? -10'd10 : 10'd0;
// wire [9:0] trak_dy = (ana_y > 8'sd12 || ana_y < -8'sd12) ? -{{5{ana_y[7]}}, ana_y[7:3]} :    // stick up is negative
//                      m_up1 ? 10'd10 : m_down1 ? -10'd10 : 10'd0;
// wire [9:0] mouse_dx = {{2{ps2_mouse[4]}}, ps2_mouse[15:8]};
// wire [9:0] mouse_dy = {{2{ps2_mouse[5]}}, ps2_mouse[23:16]};                                  // PS/2 Y is positive upward
// always @(posedge CLK_20M) begin
// 	mouse_tog <= ps2_mouse[24];
// 	trak_vbl  <= vblank;
// 	if (mouse_tog != ps2_mouse[24]) begin
// 		trak_x_c <= trak_x_c + mouse_dx;
// 		trak_y_c <= trak_y_c + mouse_dy;
// 	end
// 	else if (vblank & ~trak_vbl) begin
// 		trak_x_c <= trak_x_c + trak_dx;
// 		trak_y_c <= trak_y_c + trak_dy;
// 	end
// end
// wire [7:0] trak_x = status[19] ? trak_x_c[9:2] : status[18] ? trak_x_c[7:0] : trak_x_c[8:1];
// wire [7:0] trak_y = status[19] ? trak_y_c[9:2] : status[18] ? trak_y_c[7:0] : trak_y_c[8:1];
// CTL-2026-10-24: Slither trackball via ctl_mouse + ctl_tick + ctl_trackball (10-bit counters, Y up +,
// Speed status[19:18]: Normal [8:1], Fast [7:0], Slow [9:2]; all sources on one clock are summed)
wire       mouse_ev;
wire [9:0] mouse_dx, mouse_dy;
/* verilator lint_off UNUSED */
wire [2:0] mouse_btn;
wire [1:0] trak_dir;
/* verilator lint_on UNUSED */
ctl_mouse u_mouse(.clk(CLK_20M), .ps2_mouse(ps2_mouse), .ev(mouse_ev), .dx(mouse_dx), .dy(mouse_dy), .btn(mouse_btn));
wire trak_tick;
// CTL2-2026-10-25: original below (tick and trackball now stop while paused)
// ctl_tick #(.MODE(0)) u_trak_tick(.clk(CLK_20M), .pause(1'b0), .vblank(vblank), .period(18'd0), .tick(trak_tick));
ctl_tick #(.MODE(0)) u_trak_tick(.clk(CLK_20M), .pause(pause_cpu), .vblank(vblank), .period(18'd0), .tick(trak_tick));
wire [7:0] trak_x, trak_y;
ctl_trackball #(.CW(10), .OUT_W(8), .MOUSE_SH(0), .SPIN_SH(0), .STICK_RSH(3), .DPAD_STEP(10), .DZ(12), .LIVE_ANY(0)) u_trak
(
	// CTL2-2026-10-25: original below
// 	.clk(CLK_20M), .reset(1'b0), .pause(1'b0), .rev(2'b10), .unit_sh(2'd0), .base_sh(4'd1),
	.clk(CLK_20M), .reset(1'b0), .pause(pause_cpu), .rev(2'b10), .unit_sh(2'd0), .base_sh(4'd1),
	.speed(status[19] ? 2'd2 : {1'b0, status[18]}),
	.mouse_ev(mouse_ev), .mouse_dx(mouse_dx), .mouse_dy(mouse_dy), .spinner(9'd0), .quad_ce(2'b00), .quad_dir(2'b00),
	.tick(trak_tick), .stick_en(1'b1), .stick(joystick_l_analog_0), .dpad({m_up1, m_down1, m_left1, m_right1}),
	.x(trak_x), .y(trak_y), .dir(trak_dir)
);

//Instantiate Qix top-level platform module
qix_dispatcher QIX_inst
(
	.reset(reset),

	.clk_20m(CLK_20M),

	.game_id(game_id),
	.crt_flip(status[22]),
	.h_center(status[6:3]),
	.v_center(status[10:7]),

	.coin({~m_coin2, ~m_coin1}),
	.start_buttons({~m_start2, ~m_start1}),

	.p1_joystick({~m_right1, ~m_left1, ~m_down1, ~m_up1}),
	.trak_x(trak_x),
	.trak_y(trak_y),
	.p2_joystick(dual_stick_game
	                 ? {~m_fire_right, ~m_fire_left, ~m_fire_down, ~m_fire_up}
	                 : {~m_right2,     ~m_left2,     ~m_down2,     ~m_up2}),
	.p1_btn1(~m_p1_btn1),
	.p1_btn2(~m_p1_btn2),
	.p2_btn1(~m_btn1_p2),
	.p2_btn2(~m_btn2_p2),
	
	.service(~m_service),
	.service2(~m_service2),
	.service3(~m_service3),
	.service4(~m_service4),
	
	.video_hsync(hs),
	.video_vsync(vs),
	.video_vblank(vblank),
	.video_hblank(hblank),
	.ce_pix(ce_pix),

	.video_r(r),
	.video_g(g),
	.video_b(b),

	.sound_l(audio_l),
	.sound_r(audio_r),

    .ioctl_addr(ioctl_addr),
    .ioctl_wr(ioctl_wr && ((ioctl_index == 8'd0) || (ioctl_index == 8'd2))),
    .ioctl_data(ioctl_dout),
    .ioctl_index(ioctl_index),

	.pause(pause_cpu),
	.shared_debug_led(shared_debug_led_w),

	.hs_address(nvram_address),
	.hs_data_out(nvram_data_out),
	.hs_data_in(nvram_data_in),
	.hs_write(nvram_write)
);

// DIRECT NVRAM LOAD/SAVE
wire nvram_download = ioctl_download && (ioctl_index == 8'd4);
wire nvram_upload   = ioctl_upload && (ioctl_index == 8'd4);
wire [15:0] nvram_address = ioctl_addr[15:0];
wire [7:0]  nvram_data_in = ioctl_dout;
wire        nvram_write   = nvram_download && ioctl_wr;
wire [7:0]  nvram_data_out;

assign ioctl_din = nvram_data_out;
assign ioctl_upload_req = status[27] & ~ioctl_download;

endmodule
