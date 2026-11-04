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
    input  logic  [3:0] v_center,
    output logic        shared_debug_led
);

/////////////////////////////  Shared CPU / sound chip pool  /////////////////////////////

//   m6809r_0  data CPU     m6809r_1  video CPU
chip_pool_pkg::m6809r_i_t m6809r_0_i, m6809r_1_i;
chip_pool_pkg::m6809r_o_t m6809r_0_o, m6809r_1_o;

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

/////////////////////////////  Shared ROM blocks  /////////////////////////////

/////////////////////////////  Shared RAM pool  /////////////////////////////

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
    .shared_debug_led(shared_debug_led),
    .m6809r_0_i(m6809r_0_i),
    .m6809r_0_o(m6809r_0_o),
    .m6809r_1_i(m6809r_1_i),
    .m6809r_1_o(m6809r_1_o)
);

endmodule
