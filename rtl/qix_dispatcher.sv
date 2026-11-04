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
//   pia6821[0] sndPIA0  [1] PIA0  [2] PIA1  [3] PIA2  (data CPU board)   [4] sndPIA1  [5] sndPIA2  (sound board)
chip_pool_pkg::m6809r_i_t m6809r_0_i, m6809r_1_i;
chip_pool_pkg::m6809r_o_t m6809r_0_o, m6809r_1_o;
chip_pool_pkg::pia6821_i_t [5:0] pia6821_i;
chip_pool_pkg::pia6821_o_t [5:0] pia6821_o;

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

/////////////////////////////  Shared ROM blocks  /////////////////////////////

/////////////////////////////  Shared RAM pool  /////////////////////////////

//   block 0   1K  NVRAM (video CPU; port B = hiscore)
//   block 1  64K  VRAM  (video CPU + Slither mask RMW; port B = display scanout)
localparam int RAM_BLOCKS = 2;

ram_pool_pkg::pram_i_t [RAM_BLOCKS-1:0] pool_i;
ram_pool_pkg::pram_o_t [RAM_BLOCKS-1:0] pool_o;

function automatic int ram_aw(input int n);
    case (n)
        1:       ram_aw = 16;
        default: ram_aw = 10;
    endcase
endfunction

genvar pn;
generate
    for (pn = 0; pn < RAM_BLOCKS; pn = pn + 1) begin : rpool
        localparam int AW = ram_aw(pn);
        dpram_dc #(.widthad_a(AW)) ram
        (
            .clock_a(clk_20m), .address_a(pool_i[pn].a_addr[AW-1:0]), .data_a(pool_i[pn].a_d), .wren_a(pool_i[pn].a_we),
            .q_a(pool_o[pn].a_q),
            .clock_b(clk_20m), .address_b(pool_i[pn].b_addr[AW-1:0]), .data_b(pool_i[pn].b_d), .wren_b(pool_i[pn].b_we),
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
    .shared_debug_led(shared_debug_led),
    .m6809r_0_i(m6809r_0_i),
    .m6809r_0_o(m6809r_0_o),
    .m6809r_1_i(m6809r_1_i),
    .m6809r_1_o(m6809r_1_o),
    .pia6821_i(pia6821_i),
    .pia6821_o(pia6821_o),
    .pr_i(pool_i),
    .pr_o(pool_o)
);

endmodule
