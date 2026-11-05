//============================================================================
//  qix_dispatcher - Qix shared pools and board select
//  Copyright (C) 2026 Rodimus
//============================================================================

module qix_dispatcher
(
    input  logic        clk_20m,
    input  logic        reset,

    input  logic  [7:0] game_id,

    input  logic  [1:0] coin,
    input  logic  [1:0] start_buttons,
    input  logic  [3:0] p1_joystick,
    input  logic  [3:0] p2_joystick,
    input  logic  [7:0] trak_x,
    input  logic  [7:0] trak_y,
    input  logic        p1_btn1,
    input  logic        p1_btn2,
    input  logic        p2_btn1,
    input  logic        p2_btn2,

    input  logic        service,
    input  logic        service2,
    input  logic        service3,
    input  logic        service4,

    output logic        video_hsync,
    output logic        video_vsync,
    output logic        video_vblank,
    output logic        video_hblank,
    output logic        ce_pix,
    output logic  [7:0] video_r,
    output logic  [7:0] video_g,
    output logic  [7:0] video_b,

    output logic signed [15:0] sound_l,
    output logic signed [15:0] sound_r,

    input  logic [24:0] ioctl_addr,
    input  logic  [7:0] ioctl_data,
    input  logic        ioctl_wr,
    input  logic  [7:0] ioctl_index,

    input  logic [15:0] hs_address,
    input  logic  [7:0] hs_data_in,
    output logic  [7:0] hs_data_out,
    input  logic        hs_write,

    input  logic        pause,
    input  logic        crt_flip,
    input  logic  [3:0] h_center,
    input  logic  [3:0] v_center
);

/////////////////////////////  Shared CPU / sound chip pool  /////////////////////////////

//   m6809r_0  data CPU     m6809r_1  video CPU
//   pia6821[0] sndPIA0  [1] PIA0  [2] PIA1  [3] PIA2  (data CPU board)   [4] sndPIA1  [5] sndPIA2  (sound board)
//   mc6845_0  CRTC (video board)   mc68705p3_0  coin / input MCU (data CPU board, MCU games)
//   sn76489[0..1]  Slither's two PSGs (data CPU board; chips-repo sn76489sv, VARIANT 0 = SN76489)
//   tms5220rsws_0  TMS5220 speech chip on sndPIA2 (sound board; chips-repo tms5220 + tms5220_rsws)
chip_pool_pkg::m6809r_i_t m6809r_0_i, m6809r_1_i;
chip_pool_pkg::m6809r_o_t m6809r_0_o, m6809r_1_o;
chip_pool_pkg::pia6821_i_t [5:0] pia6821_i;
chip_pool_pkg::pia6821_o_t [5:0] pia6821_o;
chip_pool_pkg::sn76489_i_t [1:0] sn76489_i;
chip_pool_pkg::sn76489_o_t [1:0] sn76489_o;
chip_pool_pkg::tms5220rsws_i_t tms5220rsws_0_i;
chip_pool_pkg::tms5220rsws_o_t tms5220rsws_0_o;

logic signed [15:0] tms5220rsws_0_audio;
tms5220_rsws #(.CLK_HZ(20000000), .TMS_HZ(640000)) tms5220rsws_0
(
    .clk(clk_20m), .reset(tms5220rsws_0_i.reset), .pause(tms5220rsws_0_i.pause),
    .rs_n(tms5220rsws_0_i.rs_n), .ws_n(tms5220rsws_0_i.ws_n), .d_in(tms5220rsws_0_i.d_in), .d_out(tms5220rsws_0_o.d_out),
    .ready_n(tms5220rsws_0_o.ready_n), .int_n(tms5220rsws_0_o.int_n), .audio(tms5220rsws_0_audio)
);
assign tms5220rsws_0_o.audio = tms5220rsws_0_audio;
chip_pool_pkg::mc6845_i_t mc6845_0_i;
chip_pool_pkg::mc6845_o_t mc6845_0_o;

crtc6845sv_pool mc6845_0 (.CLOCK(clk_20m), .i(mc6845_0_i), .o(mc6845_0_o));

chip_pool_pkg::mc68705p3_i_t mc68705p3_0_i;
chip_pool_pkg::mc68705p3_o_t mc68705p3_0_o;

mc68705p3 mc68705p3_0
(
    .clk(clk_20m), .ce_4m(mc68705p3_0_i.ce_4m), .reset(mc68705p3_0_i.reset), .irq_n(mc68705p3_0_i.irq_n),
    .pa_in(mc68705p3_0_i.pa_in), .pa_out(mc68705p3_0_o.pa_out), .pa_latch_out(mc68705p3_0_o.pa_latch_out),
    .pa_wr_stb(mc68705p3_0_o.pa_wr_stb), .pb_in(mc68705p3_0_i.pb_in), .pb_out(mc68705p3_0_o.pb_out),
    .pb_ddr(mc68705p3_0_o.pb_ddr), .pc_in(mc68705p3_0_i.pc_in), .pc_out(mc68705p3_0_o.pc_out),
    .pc_ddr(mc68705p3_0_o.pc_ddr), .rom_wr(mc68705p3_0_i.rom_wr), .rom_addr(mc68705p3_0_i.rom_addr),
    .rom_data(mc68705p3_0_i.rom_data)
);

mc6809is m6809r_0
(
    .CLK(clk_20m), .fallE_en(m6809r_0_i.fall_e), .fallQ_en(m6809r_0_i.fall_q), .D(m6809r_0_i.d), .DOut(m6809r_0_o.dout),
    .ADDR(m6809r_0_o.a), .RnW(m6809r_0_o.rw), .BS(), .BA(), .nIRQ(m6809r_0_i.irq_n), .nFIRQ(m6809r_0_i.firq_n),
    .nNMI(m6809r_0_i.nmi_n), .AVMA(), .BUSY(), .LIC(m6809r_0_o.lic), .nHALT(m6809r_0_i.halt_n),
    .nRESET(m6809r_0_i.reset_n), .nDMABREQ(1'b1), .RegData()
);

mc6809is m6809r_1
(
    .CLK(clk_20m), .fallE_en(m6809r_1_i.fall_e), .fallQ_en(m6809r_1_i.fall_q), .D(m6809r_1_i.d), .DOut(m6809r_1_o.dout),
    .ADDR(m6809r_1_o.a), .RnW(m6809r_1_o.rw), .BS(), .BA(), .nIRQ(m6809r_1_i.irq_n), .nFIRQ(m6809r_1_i.firq_n),
    .nNMI(m6809r_1_i.nmi_n), .AVMA(), .BUSY(), .LIC(m6809r_1_o.lic), .nHALT(m6809r_1_i.halt_n),
    .nRESET(m6809r_1_i.reset_n), .nDMABREQ(1'b1), .RegData()
);

genvar pi;
generate
    for (pi = 0; pi < 6; pi++) begin : g_pia6821
        pia6821 u
        (
            .clk(clk_20m), .rst(pia6821_i[pi].rst), .ce(pia6821_i[pi].ce), .cs(pia6821_i[pi].cs), .rw(pia6821_i[pi].rw),
            .rs(pia6821_i[pi].rs), .din(pia6821_i[pi].din), .dout(pia6821_o[pi].dout),
            .irqa(pia6821_o[pi].irqa), .irqb(pia6821_o[pi].irqb),
            .pa_i(pia6821_i[pi].pa_i), .pa_pins(pia6821_i[pi].pa_pins), .pa_o(pia6821_o[pi].pa_o), .pa_oe(pia6821_o[pi].pa_oe),
            .ca1(pia6821_i[pi].ca1), .ca2_i(pia6821_i[pi].ca2_i), .ca2_o(pia6821_o[pi].ca2_o), .ca2_oe(pia6821_o[pi].ca2_oe),
            .pb_i(pia6821_i[pi].pb_i), .pb_o(pia6821_o[pi].pb_o), .pb_oe(pia6821_o[pi].pb_oe),
            .cb1(pia6821_i[pi].cb1), .cb2_i(pia6821_i[pi].cb2_i), .cb2_o(pia6821_o[pi].cb2_o), .cb2_oe(pia6821_o[pi].cb2_oe)
        );
    end
endgenerate

genvar si;
generate
    for (si = 0; si < 2; si++) begin : g_sn76489
        sn76489sv #(.VARIANT(0)) u
        (
            .rst(sn76489_i[si].rst), .clk(clk_20m), .ce(sn76489_i[si].ce), .wr_n(sn76489_i[si].wr_n),
            .cs_n(sn76489_i[si].cs_n), .din(sn76489_i[si].din), .sound(sn76489_o[si].sound), .ready(sn76489_o[si].ready)
        );
    end
endgenerate

/////////////////////////////  Shared ROM blocks  /////////////////////////////

//   block 0  32K  data CPU ROM
//   block 1  32K  video CPU ROM (Zoo Keeper: 8K bank 0, 16K fixed, 8K bank 1; the board does the bank mux)
//   block 2  16K  sound CPU ROM (12K used, $D000-$FFFF)
// Port A = ROM download (ioctl index 0, one concatenated image), port B = the board's read address.
// Download layout: Zoo Keeper (game_id 4) has 32K data + video ROMs, the other games 24K:
//   data CPU ROM   Zoo Keeper $00000-$07FFF   others $00000-$05FFF
//   video CPU ROM  Zoo Keeper $08000-$0FFFF   others $06000-$0BFFF
//   sound CPU ROM  Zoo Keeper $10000-$12FFF   others $0C000-$0EFFF
localparam int ROM_BLOCKS = 3;

logic [ROM_BLOCKS-1:0][15:0] rom_a, rom_wa;
logic [ROM_BLOCKS-1:0] [7:0] rom_d;
logic [ROM_BLOCKS-1:0]       rom_we;

function automatic int rom_aw(input int n);
    case (n)
        2:       rom_aw = 14;
        default: rom_aw = 15;
    endcase
endfunction

wire is_zookeep = (game_id == 8'h04);

assign rom_wa[0] = {1'b0, ioctl_addr[14:0]};
assign rom_we[0] = ioctl_wr & (ioctl_index == 8'd0) & (is_zookeep ? (ioctl_addr < 25'h08000) : (ioctl_addr < 25'h06000));

assign rom_wa[1] = {1'b0, is_zookeep ? ioctl_addr[14:0] : (ioctl_addr[14:0] - 15'h6000)};
assign rom_we[1] = ioctl_wr & (ioctl_index == 8'd0) & (
    is_zookeep ? (ioctl_addr >= 25'h08000 && ioctl_addr < 25'h10000)
               : (ioctl_addr >= 25'h06000 && ioctl_addr < 25'h0C000));

assign rom_wa[2] = {2'b0, ioctl_addr[13:0]};
assign rom_we[2] = ioctl_wr & (ioctl_index == 8'd0) & (
    is_zookeep ? (ioctl_addr >= 25'h10000 && ioctl_addr < 25'h13000)
               : (ioctl_addr >= 25'h0C000 && ioctl_addr < 25'h0F000));

genvar rn;
generate
    for (rn = 0; rn < ROM_BLOCKS; rn = rn + 1) begin : rom
        localparam int AW = rom_aw(rn);
        dpram_dc #(.widthad_a(AW)) rom
        (
            .clock_a(clk_20m), .address_a(rom_wa[rn][AW-1:0]), .data_a(ioctl_data), .wren_a(rom_we[rn]),
            .q_a(),
            .clock_b(clk_20m), .address_b(rom_a[rn][AW-1:0]), .data_b(8'd0), .wren_b(1'b0),
            .q_b(rom_d[rn])
        );
    end
endgenerate

/////////////////////////////  Shared RAM pool  /////////////////////////////

//   block 0   1K  NVRAM (video CPU; port B = hiscore)
//   block 1  64K  VRAM  (video CPU + Slither mask RMW; port B = display scanout)
//   block 2   1K  shared RAM (port A = data CPU, port B = video CPU on ~clk_20m)
//   block 3   1K  data CPU local RAM (port B unused)
//   block 4   1K  palette RAM (port A = video CPU, port B = display lookup)
localparam int RAM_BLOCKS = 5;

ram_pool_pkg::pram_i_t [RAM_BLOCKS-1:0] pool_i;
ram_pool_pkg::pram_o_t [RAM_BLOCKS-1:0] pool_o;

function automatic int ram_aw(input int n);
    case (n)
        1:       ram_aw = 16;
        default: ram_aw = 10;
    endcase
endfunction

// 1 = port B clocked on ~clk_20m
function automatic bit ram_binv(input int n);
    ram_binv = (n == 2);
endfunction

genvar pn;
generate
    for (pn = 0; pn < RAM_BLOCKS; pn = pn + 1) begin : rpool
        localparam int AW = ram_aw(pn);
        wire clk_b;
        if (ram_binv(pn)) begin : g_binv
            assign clk_b = ~clk_20m;
        end else begin : g_bclk
            assign clk_b = clk_20m;
        end
        dpram_dc #(.widthad_a(AW)) ram
        (
            .clock_a(clk_20m), .address_a(pool_i[pn].a_addr[AW-1:0]), .data_a(pool_i[pn].a_d), .wren_a(pool_i[pn].a_we),
            .q_a(pool_o[pn].a_q),
            .clock_b(clk_b), .address_b(pool_i[pn].b_addr[AW-1:0]), .data_b(pool_i[pn].b_d), .wren_b(pool_i[pn].b_we),
            .q_b(pool_o[pn].b_q)
        );
    end
endgenerate

/////////////////////////////  Boards  /////////////////////////////

Qix qix
(
    .clk_20m(clk_20m),
    .reset(reset),
    .game_id(game_id),
    .coin(coin),
    .start_buttons(start_buttons),
    .p1_joystick(p1_joystick),
    .p2_joystick(p2_joystick),
    .trak_x(trak_x),
    .trak_y(trak_y),
    .p1_btn1(p1_btn1),
    .p1_btn2(p1_btn2),
    .p2_btn1(p2_btn1),
    .p2_btn2(p2_btn2),
    .service(service),
    .service2(service2),
    .service3(service3),
    .service4(service4),
    .video_hsync(video_hsync),
    .video_vsync(video_vsync),
    .video_vblank(video_vblank),
    .video_hblank(video_hblank),
    .ce_pix(ce_pix),
    .video_r(video_r),
    .video_g(video_g),
    .video_b(video_b),
    .sound_l(sound_l),
    .sound_r(sound_r),
    .ioctl_addr(ioctl_addr),
    .ioctl_data(ioctl_data),
    .ioctl_wr(ioctl_wr),
    .ioctl_index(ioctl_index),
    .hs_address(hs_address),
    .hs_data_in(hs_data_in),
    .hs_data_out(hs_data_out),
    .hs_write(hs_write),
    .pause(pause),
    .crt_flip(crt_flip),
    .h_center(h_center),
    .v_center(v_center),
    .m6809r_0_i(m6809r_0_i),
    .m6809r_0_o(m6809r_0_o),
    .m6809r_1_i(m6809r_1_i),
    .m6809r_1_o(m6809r_1_o),
    .mc6845_0_i(mc6845_0_i),
    .mc6845_0_o(mc6845_0_o),
    .mc68705p3_0_i(mc68705p3_0_i),
    .mc68705p3_0_o(mc68705p3_0_o),
    .pia6821_i(pia6821_i),
    .pia6821_o(pia6821_o),
    .sn76489_i(sn76489_i),
    .sn76489_o(sn76489_o),
    .tms5220rsws_0_i(tms5220rsws_0_i),
    .tms5220rsws_0_o(tms5220rsws_0_o),
    .rom_a(rom_a),
    .rom_d(rom_d),
    .pr_i(pool_i),
    .pr_o(pool_o)
);

endmodule
